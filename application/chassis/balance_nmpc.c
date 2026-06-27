#include "balance_nmpc.h"

#include "buzzer/buzzer_app.h"
#include "bsp_dwt.h"
#include "wbr_lqr_calc.h"

#include <math.h>
#include <string.h>

/* 200Hz NMPC 任务写入双缓冲 target，BalanceTask 读取时用序号校验避免半更新。 */
static volatile BalanceNmpcTarget nmpc_target_buffer[2];
static volatile uint8_t nmpc_target_write_index;
static volatile uint8_t nmpc_target_read_index;
static volatile uint32_t nmpc_target_seq;
static volatile BalanceNmpcStatus nmpc_status = {
    .mode = BALANCE_NMPC_DISABLED,
};
static float nmpc_last_alarm_ms;

typedef struct
{
    /* 上层规划器内部参考。它比遥控期望更平滑，直接喂给底层 LQR/VMC。 */
    float v_ref;
    float wz_ref;
    float pitch_ref;
    float leg_len_l_ref;
    float leg_len_r_ref;
    uint8_t initialized;
} BalanceNmpcPlannerState;

static BalanceNmpcPlannerState planner_state;

/* 大矩阵放在静态区，避免 512-word NMPC task 栈被模型预测占满。 */
static float nmpc_a_mat[WBR_LQR_X_DIM][WBR_LQR_X_DIM];
static float nmpc_b_mat[WBR_LQR_X_DIM][WBR_LQR_U_DIM];
static float nmpc_k_mat[WBR_LQR_U_DIM][WBR_LQR_X_DIM];

typedef struct
{
    /* 与 balance_simulate/并腿/wbr_lqr_offline.py 的 RobotParams 保持一致。 */
    float m_w;
    float m_l;
    float m_b;
    float i_w;
    float i_b;
    float i_z;
    float r_w;
    float r_l;
    float l_c;
    float leg_inertia_width;
} NmpcRobotParams;

static const NmpcRobotParams nmpc_params = {
    .m_w = BALANCE_NMPC_WHEEL_MASS,
    .m_l = BALANCE_NMPC_LEG_MASS,
    .m_b = BALANCE_NMPC_BODY_MASS,
    .i_w = BALANCE_NMPC_WHEEL_INERTIA,
    .i_b = BALANCE_NMPC_BODY_INERTIA_PITCH,
    .i_z = BALANCE_NMPC_BODY_INERTIA_YAW,
    .r_w = BALANCE_NMPC_WHEEL_RADIUS,
    .r_l = BALANCE_NMPC_HALF_TRACK,
    .l_c = BALANCE_NMPC_BODY_COM_OFFSET_X,
    .leg_inertia_width = BALANCE_NMPC_LEG_INERTIA_WIDTH,
};

typedef struct
{
    /* 单步闭环预测：x 是 10 状态误差，u 是底层 LQR 预计输出力矩。 */
    float x[WBR_LQR_X_DIM];
    float u[WBR_LQR_U_DIM];
    float saturation_cost;
} NmpcPredictStep;

typedef struct
{
    /* 有限候选搜索保存的最佳上层参考。 */
    float v;
    float wz;
    float pitch;
    float cost;
} NmpcCandidateResult;

/**
 * @brief 将浮点数限制在指定闭区间内。
 *
 * @param value 输入值。
 * @param min 下限。
 * @param max 上限。
 * @return 限幅后的值。
 */
static float NmpcClampFloat(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 检查浮点数是否为有限值。
 *
 * @param value 待检查的浮点数。
 * @return 1 表示有限值，0 表示 NaN 或 Inf。
 */
static uint8_t NmpcIsFiniteFloat(float value)
{
    return isfinite(value) ? 1u : 0u;
}

/**
 * @brief 将 current 按最大步长向 desired 逼近。
 *
 * @param current 当前参考值。
 * @param desired 期望参考值。
 * @param max_delta 单周期允许变化的最大绝对值。
 * @return 限速后的新参考值。
 */
static float NmpcRateLimit(float current, float desired, float max_delta)
{
    return current + NmpcClampFloat(desired - current, -max_delta, max_delta);
}

/**
 * @brief 将角度包裹到 [-pi, pi] 区间。
 *
 * @param angle 输入角度，单位 rad。
 * @return 归一化后的角度，单位 rad。
 */
static float NmpcWrapPi(float angle)
{
    while (angle > BALANCE_PI) angle -= 2.0f * BALANCE_PI;
    while (angle < -BALANCE_PI) angle += 2.0f * BALANCE_PI;
    return angle;
}

/**
 * @brief 将普通 target 拷贝到 volatile 发布缓冲。
 *
 * @param dst 发布缓冲目标。
 * @param src 普通目标。
 */
static void NmpcCopyTargetToVolatile(volatile BalanceNmpcTarget *dst,
                                     const BalanceNmpcTarget *src)
{
    dst->target_v = src->target_v;
    dst->target_wz = src->target_wz;
    dst->target_pitch = src->target_pitch;
    dst->target_roll = src->target_roll;
    dst->target_leg_len_l = src->target_leg_len_l;
    dst->target_leg_len_r = src->target_leg_len_r;
    dst->timestamp_ms = src->timestamp_ms;
    dst->valid = src->valid;
}

/**
 * @brief 从 volatile 发布缓冲拷贝出普通 target。
 *
 * @param dst 普通目标。
 * @param src 发布缓冲目标。
 */
static void NmpcCopyTargetFromVolatile(BalanceNmpcTarget *dst,
                                       const volatile BalanceNmpcTarget *src)
{
    dst->target_v = src->target_v;
    dst->target_wz = src->target_wz;
    dst->target_pitch = src->target_pitch;
    dst->target_roll = src->target_roll;
    dst->target_leg_len_l = src->target_leg_len_l;
    dst->target_leg_len_r = src->target_leg_len_r;
    dst->timestamp_ms = src->timestamp_ms;
    dst->valid = src->valid;
}

/**
 * @brief 发布一个新的 NMPC target。
 *
 * 使用双缓冲和奇偶序号：写入期间序号为奇数，写完后变回偶数。
 *
 * @param target 待发布目标。
 */
static void NmpcPublishTarget(const BalanceNmpcTarget *target)
{
    const uint8_t next_index = (uint8_t)((nmpc_target_write_index ^ 1u) & 1u);

    nmpc_target_seq++;
    NmpcCopyTargetToVolatile(&nmpc_target_buffer[next_index], target);
    nmpc_target_read_index = next_index;
    nmpc_target_write_index = next_index;
    nmpc_target_seq++;
}

/**
 * @brief 清除已发布的 NMPC target。
 */
static void NmpcInvalidateTarget(void)
{
    const BalanceNmpcTarget invalid_target = {0};
    NmpcPublishTarget(&invalid_target);
}

/**
 * @brief 读取最新 NMPC target，并校验读取过程中没有被 200Hz 任务改写。
 *
 * @param target 输出目标。
 * @return 1 表示读取到稳定且有效的目标，0 表示目标无效或正在更新。
 */
static uint8_t NmpcReadPublishedTarget(BalanceNmpcTarget *target)
{
    uint32_t seq_before;
    uint32_t seq_after;
    uint8_t read_index;

    if (target == 0) return 0u;

    seq_before = nmpc_target_seq;
    if ((seq_before & 1u) != 0u) return 0u;

    read_index = (uint8_t)(nmpc_target_read_index & 1u);
    NmpcCopyTargetFromVolatile(target, &nmpc_target_buffer[read_index]);

    seq_after = nmpc_target_seq;
    if (seq_before != seq_after) return 0u;
    if ((seq_after & 1u) != 0u) return 0u;
    return target->valid;
}

/**
 * @brief 重置 NMPC 上层规划器内部状态。
 */
static void NmpcResetPlannerState(void)
{
    memset(&planner_state, 0, sizeof(planner_state));
}

/**
 * @brief 播放 NMPC 超时/失效提示音。
 */
static void NmpcPlayTimeoutAlarm(void)
{
    const float now_ms = DWT_GetTimeline_ms();

    /* 失效可能连续发生，蜂鸣器节流避免把其它提示音完全打断。 */
    if ((now_ms - nmpc_last_alarm_ms) < 500.0f)
    {
        return;
    }

    nmpc_last_alarm_ms = now_ms;
    BuzzerAppPlay(BUZZER_APP_SOUND_NMPC_TIMEOUT);
}

/**
 * @brief 进入 NMPC fallback 状态。
 *
 * 清除 NMPC target，使底层继续使用原有平衡控制路径，同时记录 fallback 次数并触发提示音。
 */
static void NmpcEnterFallback(void)
{
    /* fallback 只撤销 NMPC 参考，不停底层平衡控制。 */
    NmpcInvalidateTarget();
    nmpc_status.active = 0u;
    nmpc_status.mode = BALANCE_NMPC_FALLBACK;
    nmpc_status.fallback_count++;
    NmpcResetPlannerState();
    NmpcPlayTimeoutAlarm();
}

#if BALANCE_NMPC_ENABLE
/**
 * @brief 计算单侧腿的 pitch 等效转动惯量。
 *
 * @param leg_len 当前腿长，单位 m。
 * @return 单侧腿等效惯量，单位 kg*m^2。
 */
static float NmpcLegInertia(float leg_len)
{
    /* 单侧腿 pitch 等效惯量：m*L^2/3 + m*d^2/12。 */
    return nmpc_params.m_l * leg_len * leg_len / 3.0f +
           nmpc_params.m_l * nmpc_params.leg_inertia_width * nmpc_params.leg_inertia_width / 12.0f;
}

/**
 * @brief 求解 5x5 线性方程 lhs * out = rhs。
 *
 * @param lhs 左侧 5x5 矩阵。
 * @param rhs 右侧 5 维向量。
 * @param out 输出解向量。
 * @return 1 表示求解成功，0 表示矩阵接近奇异。
 */
/*
 * 5x5 高斯消元，用于求解 M qdd = rhs。
 * 维度固定且很小，手写求解器比引入通用矩阵库更可控。
 */
static uint8_t NmpcSolve5x5(const float lhs[5][5], const float rhs[5], float out[5])
{
    float a[5][6];

    for (uint8_t row = 0u; row < 5u; row++)
    {
        for (uint8_t col = 0u; col < 5u; col++)
        {
            a[row][col] = lhs[row][col];
        }
        a[row][5] = rhs[row];
    }

    for (uint8_t pivot = 0u; pivot < 5u; pivot++)
    {
        uint8_t best = pivot;
        float best_abs = fabsf(a[pivot][pivot]);
        for (uint8_t row = pivot + 1u; row < 5u; row++)
        {
            const float value_abs = fabsf(a[row][pivot]);
            if (value_abs > best_abs)
            {
                best_abs = value_abs;
                best = row;
            }
        }
        if (best_abs < 1.0e-6f)
        {
            return 0u;
        }
        if (best != pivot)
        {
            for (uint8_t col = pivot; col < 6u; col++)
            {
                const float tmp = a[pivot][col];
                a[pivot][col] = a[best][col];
                a[best][col] = tmp;
            }
        }

        const float inv_pivot = 1.0f / a[pivot][pivot];
        for (uint8_t col = pivot; col < 6u; col++)
        {
            a[pivot][col] *= inv_pivot;
        }

        for (uint8_t row = 0u; row < 5u; row++)
        {
            if (row == pivot) continue;
            const float factor = a[row][pivot];
            for (uint8_t col = pivot; col < 6u; col++)
            {
                a[row][col] -= factor * a[pivot][col];
            }
        }
    }

    for (uint8_t row = 0u; row < 5u; row++)
    {
        out[row] = a[row][5];
    }
    return 1u;
}

/**
 * @brief 按当前左右腿长构建 WBR 连续状态空间模型。
 *
 * @param left_leg_len 当前左腿腿长，单位 m。
 * @param right_leg_len 当前右腿腿长，单位 m。
 * @param a_mat 输出连续系统 A 矩阵，维度 10x10。
 * @param b_mat 输出连续系统 B 矩阵，维度 10x4。
 * @return 1 表示模型构建成功，0 表示动力学矩阵不可解。
 */
static uint8_t NmpcBuildModel(float left_leg_len,
                              float right_leg_len,
                              float a_mat[WBR_LQR_X_DIM][WBR_LQR_X_DIM],
                              float b_mat[WBR_LQR_X_DIM][WBR_LQR_U_DIM])
{
    /*
     * 这里直接移植仿真脚本 build_dynamics_matrices()/build_ab()：
     * M qdd + H x + D u = 0, xdot = A x + B u。
     */
    float m[5][5] = {{0.0f}};
    float h[5][WBR_LQR_X_DIM] = {{0.0f}};
    float d[5][WBR_LQR_U_DIM] = {{0.0f}};
    float qdd_x[5][WBR_LQR_X_DIM] = {{0.0f}};
    float qdd_u[5][WBR_LQR_U_DIM] = {{0.0f}};
    float rhs[5];
    float solved[5];

    const float mw = nmpc_params.m_w;
    const float ml = nmpc_params.m_l;
    const float mb = nmpc_params.m_b;
    const float iw = nmpc_params.i_w;
    const float ib = nmpc_params.i_b;
    const float iz = nmpc_params.i_z;
    const float rw = nmpc_params.r_w;
    const float rl = nmpc_params.r_l;
    const float lc = nmpc_params.l_c;
    const float ll = NmpcClampFloat(left_leg_len, L0_MIN, L0_MAX);
    const float lr = NmpcClampFloat(right_leg_len, L0_MIN, L0_MAX);
    const float lb_l = 0.5f * ll;
    const float lb_r = 0.5f * lr;
    const float lw_l = ll - lb_l;
    const float lw_r = lr - lb_r;
    const float il_l = NmpcLegInertia(ll);
    const float il_r = NmpcLegInertia(lr);
    const float g = BALANCE_GRAVITY;

    /* 左/右腿、平移、pitch、yaw 五个动力学方程，顺序与仿真模型一致。 */
    m[0][0] = iw * ll / rw + mw * rw * ll + ml * rw * lb_l;
    m[0][2] = ml * lw_l * lb_l - il_l;
    h[0][4] = (ml * lw_l + 0.5f * mb * ll) * g;
    d[0][0] = -(1.0f + ll / rw);
    d[0][2] = 1.0f;

    m[1][1] = iw * lr / rw + mw * rw * lr + ml * rw * lb_r;
    m[1][3] = ml * lw_r * lb_r - il_r;
    h[1][6] = (ml * lw_r + 0.5f * mb * lr) * g;
    d[1][1] = -(1.0f + lr / rw);
    d[1][3] = 1.0f;

    const float wheel_group = mw * rw * rw + iw + ml * rw * rw + 0.5f * mb * rw * rw;
    m[2][0] = -wheel_group;
    m[2][1] = -wheel_group;
    m[2][2] = -(ml * rw * lw_l + 0.5f * mb * rw * ll);
    m[2][3] = -(ml * rw * lw_r + 0.5f * mb * rw * lr);
    d[2][0] = 1.0f;
    d[2][1] = 1.0f;

    const float body_wheel = mw * rw * lc + iw * lc / rw + ml * rw * lc;
    m[3][0] = body_wheel;
    m[3][1] = body_wheel;
    m[3][2] = ml * lw_l * lc;
    m[3][3] = ml * lw_r * lc;
    m[3][4] = -ib;
    h[3][8] = mb * g * lc;
    d[3][0] = -lc / rw;
    d[3][1] = -lc / rw;
    d[3][2] = -1.0f;
    d[3][3] = -1.0f;

    const float yaw_wheel = 0.5f * iz * rw / rl + iw * rl / rw;
    m[4][0] = yaw_wheel;
    m[4][1] = -yaw_wheel;
    m[4][2] = 0.5f * iz * ll / rl;
    m[4][3] = -0.5f * iz * lr / rl;
    d[4][0] = -rl / rw;
    d[4][1] = rl / rw;

    /* qdd_from_x = -solve(M, H), qdd_from_u = -solve(M, D)。 */
    for (uint8_t col = 0u; col < WBR_LQR_X_DIM; col++)
    {
        for (uint8_t row = 0u; row < 5u; row++) rhs[row] = -h[row][col];
        if (NmpcSolve5x5(m, rhs, solved) == 0u) return 0u;
        for (uint8_t row = 0u; row < 5u; row++) qdd_x[row][col] = solved[row];
    }
    for (uint8_t col = 0u; col < WBR_LQR_U_DIM; col++)
    {
        for (uint8_t row = 0u; row < 5u; row++) rhs[row] = -d[row][col];
        if (NmpcSolve5x5(m, rhs, solved) == 0u) return 0u;
        for (uint8_t row = 0u; row < 5u; row++) qdd_u[row][col] = solved[row];
    }

    memset(a_mat, 0, sizeof(float) * WBR_LQR_X_DIM * WBR_LQR_X_DIM);
    memset(b_mat, 0, sizeof(float) * WBR_LQR_X_DIM * WBR_LQR_U_DIM);

    a_mat[0][1] = 1.0f;
    a_mat[2][3] = 1.0f;
    a_mat[4][5] = 1.0f;
    a_mat[6][7] = 1.0f;
    a_mat[8][9] = 1.0f;

    /* 把轮/腿/body 的广义加速度映射回 10 状态的 A/B 行。 */
    for (uint8_t col = 0u; col < WBR_LQR_X_DIM; col++)
    {
        a_mat[1][col] = 0.5f * rw * (qdd_x[0][col] + qdd_x[1][col]);
        a_mat[3][col] = rw / (2.0f * rl) * (-qdd_x[0][col] + qdd_x[1][col]) -
                        ll / (2.0f * rl) * qdd_x[2][col] +
                        lr / (2.0f * rl) * qdd_x[3][col];
        a_mat[5][col] = qdd_x[2][col];
        a_mat[7][col] = qdd_x[3][col];
        a_mat[9][col] = qdd_x[4][col];
    }
    for (uint8_t col = 0u; col < WBR_LQR_U_DIM; col++)
    {
        b_mat[1][col] = 0.5f * rw * (qdd_u[0][col] + qdd_u[1][col]);
        b_mat[3][col] = rw / (2.0f * rl) * (-qdd_u[0][col] + qdd_u[1][col]) -
                        ll / (2.0f * rl) * qdd_u[2][col] +
                        lr / (2.0f * rl) * qdd_u[3][col];
        b_mat[5][col] = qdd_u[2][col];
        b_mat[7][col] = qdd_u[3][col];
        b_mat[9][col] = qdd_u[4][col];
    }

    return 1u;
}

/**
 * @brief 根据当前状态和候选参考组装 LQR 预测用误差状态。
 *
 * @param state 当前平衡状态。
 * @param target_v 候选目标前向速度，单位 m/s。
 * @param target_wz 候选目标 yaw 角速度，单位 rad/s。
 * @param target_pitch 候选目标 pitch，单位 rad。
 * @param x 输出 10 维 LQR 误差状态。
 */
static void NmpcAssembleErrorState(const BalanceState *state,
                                   float target_v,
                                   float target_wz,
                                   float target_pitch,
                                   float x[WBR_LQR_X_DIM])
{
    /*
     * 预测状态使用 LQR 的 10 维误差顺序。
     * 有速度/yaw-rate 指令时清零位置/yaw 误差，避免移动中被位置保持项拉回。
     */
    x[0] = state->chassis.dist - state->chassis.target_dist;
    if (fabsf(target_v) > 0.005f) x[0] = 0.0f;
    x[1] = state->chassis.vel - target_v;
    x[2] = NmpcWrapPi(state->chassis.yaw - state->chassis.target_yaw);
    if (fabsf(target_wz) > 0.05f) x[2] = 0.0f;
    x[3] = state->chassis.wz - target_wz;
    x[4] = state->left.theta;
    x[5] = state->left.theta_w;
    x[6] = state->right.theta;
    x[7] = state->right.theta_w;
    x[8] = state->chassis.pitch - target_pitch;
    x[9] = state->chassis.pitch_w;
}

/**
 * @brief 使用当前 A/B/K 预测底层 LQR 闭环的一个 NMPC 步长。
 *
 * @param a_mat 连续系统 A 矩阵。
 * @param b_mat 连续系统 B 矩阵。
 * @param k_mat 当前腿长插值得到的 LQR K 矩阵。
 * @param step 输入/输出预测步状态，函数会更新 x、u 和饱和代价。
 */
static void NmpcClosedLoopStep(const float a_mat[WBR_LQR_X_DIM][WBR_LQR_X_DIM],
                               const float b_mat[WBR_LQR_X_DIM][WBR_LQR_U_DIM],
                               const float k_mat[WBR_LQR_U_DIM][WBR_LQR_X_DIM],
                               NmpcPredictStep *step)
{
    float x_dot[WBR_LQR_X_DIM] = {0.0f};

    /* 预测底层 LQR: u = -Kx，并把超出力矩限幅的部分计入软约束代价。 */
    step->saturation_cost = 0.0f;
    for (uint8_t row = 0u; row < WBR_LQR_U_DIM; row++)
    {
        float u_raw = 0.0f;
        const float limit = (row < 2u) ? WBR_LQR_WHEEL_TORQUE_LIMIT : WBR_LQR_VIRTUAL_HIP_TORQUE_LIMIT;
        for (uint8_t col = 0u; col < WBR_LQR_X_DIM; col++)
        {
            u_raw -= k_mat[row][col] * step->x[col];
        }
        step->u[row] = NmpcClampFloat(u_raw, -limit, limit);
        if (fabsf(u_raw) > limit)
        {
            const float over = (fabsf(u_raw) - limit) / limit;
            step->saturation_cost += over * over;
        }
    }

    for (uint8_t row = 0u; row < WBR_LQR_X_DIM; row++)
    {
        for (uint8_t col = 0u; col < WBR_LQR_X_DIM; col++)
        {
            x_dot[row] += a_mat[row][col] * step->x[col];
        }
        for (uint8_t col = 0u; col < WBR_LQR_U_DIM; col++)
        {
            x_dot[row] += b_mat[row][col] * step->u[col];
        }
    }

    for (uint8_t row = 0u; row < WBR_LQR_X_DIM; row++)
    {
        step->x[row] += BALANCE_NMPC_DT * x_dot[row];
    }
    step->x[2] = NmpcWrapPi(step->x[2]);
}

/**
 * @brief 计算超过软约束后的归一化平方惩罚。
 *
 * @param abs_value 被约束量的绝对值。
 * @param limit 软约束阈值。
 * @return 超限惩罚，未超限时为 0。
 */
static float NmpcHingeCost(float abs_value, float limit)
{
    /* 软约束：限制内不罚，超过后按归一化平方惩罚。 */
    if (abs_value <= limit) return 0.0f;
    const float over = (abs_value - limit) / limit;
    return over * over;
}

/**
 * @brief 评价一个候选上层参考在预测域内的闭环代价。
 *
 * @param state 当前平衡状态。
 * @param target_v 候选目标前向速度，单位 m/s。
 * @param target_wz 候选目标 yaw 角速度，单位 rad/s。
 * @param target_pitch 候选目标 pitch，单位 rad。
 * @param a_mat 连续系统 A 矩阵。
 * @param b_mat 连续系统 B 矩阵。
 * @param k_mat 当前 LQR K 矩阵。
 * @return 候选参考对应的预测代价，越小越优。
 */
static float NmpcEvaluateCandidate(const BalanceState *state,
                                   float target_v,
                                   float target_wz,
                                   float target_pitch,
                                   const float a_mat[WBR_LQR_X_DIM][WBR_LQR_X_DIM],
                                   const float b_mat[WBR_LQR_X_DIM][WBR_LQR_U_DIM],
                                   const float k_mat[WBR_LQR_U_DIM][WBR_LQR_X_DIM])
{
    NmpcPredictStep step = {0};
    float cost = 0.0f;

    NmpcAssembleErrorState(state, target_v, target_wz, target_pitch, step.x);

    /* 向前滚动预测，评价该参考交给底层 LQR 后的姿态、腿角和力矩压力。 */
    for (uint8_t i = 0u; i < BALANCE_NMPC_HORIZON; i++)
    {
        cost += 3.0f * step.x[1] * step.x[1];
        cost += 1.2f * step.x[3] * step.x[3];
        cost += 8.0f * step.x[4] * step.x[4];
        cost += 0.8f * step.x[5] * step.x[5];
        cost += 8.0f * step.x[6] * step.x[6];
        cost += 0.8f * step.x[7] * step.x[7];
        cost += 22.0f * step.x[8] * step.x[8];
        cost += 1.5f * step.x[9] * step.x[9];
        cost += 40.0f * NmpcHingeCost(fabsf(step.x[4]), 0.35f);
        cost += 40.0f * NmpcHingeCost(fabsf(step.x[6]), 0.35f);
        cost += 70.0f * NmpcHingeCost(fabsf(step.x[8]), 0.22f);
        NmpcClosedLoopStep(a_mat, b_mat, k_mat, &step);
        cost += 0.04f * (step.u[0] * step.u[0] + step.u[1] * step.u[1]);
        cost += 0.01f * (step.u[2] * step.u[2] + step.u[3] * step.u[3]);
        cost += 30.0f * step.saturation_cost;
    }

    return cost;
}

/**
 * @brief 评价候选参考并在更优时更新最佳结果。
 *
 * @param best 当前最佳候选结果，可能被本函数更新。
 * @param state 当前平衡状态。
 * @param candidate_v 候选目标前向速度，单位 m/s。
 * @param candidate_wz 候选目标 yaw 角速度，单位 rad/s。
 * @param candidate_pitch 候选目标 pitch，单位 rad。
 * @param extra_cost 参考移动和平滑项的额外代价。
 * @param a_mat 连续系统 A 矩阵。
 * @param b_mat 连续系统 B 矩阵。
 * @param k_mat 当前 LQR K 矩阵。
 */
static void NmpcTryCandidate(NmpcCandidateResult *best,
                             const BalanceState *state,
                             float candidate_v,
                             float candidate_wz,
                             float candidate_pitch,
                             float extra_cost,
                             const float a_mat[WBR_LQR_X_DIM][WBR_LQR_X_DIM],
                             const float b_mat[WBR_LQR_X_DIM][WBR_LQR_U_DIM],
                             const float k_mat[WBR_LQR_U_DIM][WBR_LQR_X_DIM])
{
    /* extra_cost 用于同时考虑“离遥控期望有多远”和“参考变化有多猛”。 */
    const float cost = NmpcEvaluateCandidate(state,
                                             candidate_v,
                                             candidate_wz,
                                             candidate_pitch,
                                             a_mat,
                                             b_mat,
                                             k_mat) +
                       extra_cost;
    if (cost < best->cost)
    {
        best->cost = cost;
        best->v = candidate_v;
        best->wz = candidate_wz;
        best->pitch = candidate_pitch;
    }
}

/**
 * @brief 求解本周期 NMPC 上层参考。
 *
 * @param state 当前平衡状态。
 * @param cmd 当前底盘控制命令。
 * @param start_ms 本周期 NMPC 开始时间，用于中途超时截断。
 * @param target 输出给底层控制的 NMPC 目标参考。
 * @return 1 表示求解成功且 target 已填充，0 表示本周期求解失败。
 */
static uint8_t NmpcSolveReference(const BalanceState *state,
                                  const Chassis_Ctrl_Cmd_s *cmd,
                                  float start_ms,
                                  BalanceNmpcTarget *target)
{
    float desired_v;
    float desired_wz;
    float desired_leg_l;
    float desired_leg_r;
    float desired_pitch;
    float v_plan;
    float wz_plan;
    float pitch_plan;
    float leg_l_plan;
    float leg_r_plan;
    NmpcCandidateResult best = {0};
    float v_set[3];
    float wz_set[3];
    float pitch_set[3];

    if (cmd == 0 || target == 0) return 0u;
    if (cmd->chassis_mode != CHASSIS_STAND) return 0u;
    if (state == 0) return 0u;

    /* 把遥控/上层命令先裁剪成 NMPC 允许搜索的目标。 */
    desired_v = NmpcClampFloat(cmd->vx, -BALANCE_NMPC_V_LIMIT, BALANCE_NMPC_V_LIMIT);
    desired_wz = NmpcClampFloat(cmd->wz, -BALANCE_NMPC_WZ_LIMIT, BALANCE_NMPC_WZ_LIMIT);
    desired_leg_l = NmpcClampFloat((cmd->leg_height_l > 0.001f) ? cmd->leg_height_l : L0_INIT,
                                   L0_MIN,
                                   L0_MAX);
    desired_leg_r = NmpcClampFloat((cmd->leg_height_r > 0.001f) ? cmd->leg_height_r : L0_INIT,
                                   L0_MIN,
                                   L0_MAX);
    desired_pitch = NmpcClampFloat(BALANCE_TARGET_PITCH - BALANCE_NMPC_PITCH_FROM_V_GAIN * desired_v,
                                   -BALANCE_NMPC_PITCH_LIMIT,
                                   BALANCE_NMPC_PITCH_LIMIT);
    best.v = desired_v;
    best.wz = desired_wz;
    best.pitch = desired_pitch;
    best.cost = 1.0e30f;

    if (planner_state.initialized == 0u)
    {
        /* 首次进入时从当前底层目标接管，避免 NMPC 使能瞬间目标跳变。 */
        planner_state.v_ref = state->chassis.target_v;
        planner_state.wz_ref = state->chassis.target_wz;
        planner_state.pitch_ref = state->chassis.target_pitch;
        planner_state.leg_len_l_ref = state->left.target_len;
        planner_state.leg_len_r_ref = state->right.target_len;
        planner_state.initialized = 1u;
    }

    if (NmpcBuildModel(state->left.leg_len, state->right.leg_len, nmpc_a_mat, nmpc_b_mat) == 0u)
    {
        return 0u;
    }
    WbrLqrInterpolateK(state->left.leg_len, state->right.leg_len, nmpc_k_mat);

    /*
     * 3x3x3 有限候选搜索：
     * 保持当前参考 / 按单周期变化率前进一步 / 直接看向期望值。
     */
    v_set[0] = planner_state.v_ref;
    v_set[1] = NmpcRateLimit(planner_state.v_ref,
                             desired_v,
                             BALANCE_NMPC_V_RATE_LIMIT * BALANCE_NMPC_DT);
    v_set[2] = desired_v;

    wz_set[0] = planner_state.wz_ref;
    wz_set[1] = NmpcRateLimit(planner_state.wz_ref,
                              desired_wz,
                              BALANCE_NMPC_WZ_RATE_LIMIT * BALANCE_NMPC_DT);
    wz_set[2] = desired_wz;

    pitch_set[0] = desired_pitch;
    pitch_set[1] = NmpcClampFloat(desired_pitch + 0.025f,
                                  -BALANCE_NMPC_PITCH_LIMIT,
                                  BALANCE_NMPC_PITCH_LIMIT);
    pitch_set[2] = NmpcClampFloat(desired_pitch - 0.025f,
                                  -BALANCE_NMPC_PITCH_LIMIT,
                                  BALANCE_NMPC_PITCH_LIMIT);

    for (uint8_t vi = 0u; vi < 3u; vi++)
    {
        for (uint8_t wi = 0u; wi < 3u; wi++)
        {
            for (uint8_t pi = 0u; pi < 3u; pi++)
            {
                /* 给外层 2ms 超时留一点余量，避免本周期算完才发现超时。 */
                if ((DWT_GetTimeline_ms() - start_ms) > (0.85f * BALANCE_NMPC_TIMEOUT_MS))
                {
                    return 0u;
                }
                const float v_candidate = NmpcClampFloat(v_set[vi],
                                                         -BALANCE_NMPC_V_LIMIT,
                                                         BALANCE_NMPC_V_LIMIT);
                const float wz_candidate = NmpcClampFloat(wz_set[wi],
                                                          -BALANCE_NMPC_WZ_LIMIT,
                                                          BALANCE_NMPC_WZ_LIMIT);
                const float pitch_candidate = pitch_set[pi];
                const float ref_move_cost =
                    0.03f * (v_candidate - planner_state.v_ref) * (v_candidate - planner_state.v_ref) +
                    0.02f * (wz_candidate - planner_state.wz_ref) * (wz_candidate - planner_state.wz_ref) +
                    0.06f * (pitch_candidate - planner_state.pitch_ref) * (pitch_candidate - planner_state.pitch_ref) +
                    0.18f * (v_candidate - desired_v) * (v_candidate - desired_v) +
                    0.08f * (wz_candidate - desired_wz) * (wz_candidate - desired_wz) +
                    0.12f * (pitch_candidate - desired_pitch) * (pitch_candidate - desired_pitch);
                NmpcTryCandidate(&best,
                                 state,
                                 v_candidate,
                                 wz_candidate,
                                 pitch_candidate,
                                 ref_move_cost,
                                 nmpc_a_mat,
                                 nmpc_b_mat,
                                 nmpc_k_mat);
            }
        }
    }

    /* 最终输出仍再过一次 rate limit，保证下发给底层的参考连续。 */
    v_plan = NmpcRateLimit(planner_state.v_ref,
                           best.v,
                           BALANCE_NMPC_V_RATE_LIMIT * BALANCE_NMPC_DT);
    wz_plan = NmpcRateLimit(planner_state.wz_ref,
                            best.wz,
                            BALANCE_NMPC_WZ_RATE_LIMIT * BALANCE_NMPC_DT);
    pitch_plan = NmpcRateLimit(planner_state.pitch_ref,
                               best.pitch,
                               BALANCE_NMPC_PITCH_RATE_LIMIT * BALANCE_NMPC_DT);
    leg_l_plan = NmpcRateLimit(planner_state.leg_len_l_ref,
                               desired_leg_l,
                               BALANCE_NMPC_LEG_RATE_LIMIT * BALANCE_NMPC_DT);
    leg_r_plan = NmpcRateLimit(planner_state.leg_len_r_ref,
                               desired_leg_r,
                               BALANCE_NMPC_LEG_RATE_LIMIT * BALANCE_NMPC_DT);

    planner_state.v_ref = v_plan;
    planner_state.wz_ref = wz_plan;
    planner_state.pitch_ref = pitch_plan;
    planner_state.leg_len_l_ref = leg_l_plan;
    planner_state.leg_len_r_ref = leg_r_plan;

    target->target_v = v_plan;
    target->target_wz = wz_plan;
    target->target_pitch = pitch_plan;
    target->target_roll = BALANCE_TARGET_ROLL;
    target->target_leg_len_l = leg_l_plan;
    target->target_leg_len_r = leg_r_plan;
    target->timestamp_ms = DWT_GetTimeline_ms();
    target->valid = 1u;
    return 1u;
}
#endif

/**
 * @brief 检查 NMPC 输出目标是否可用于底层控制。
 *
 * @param target 待检查的 NMPC 目标。
 * @return 1 表示目标安全有效，0 表示无效或越界。
 */
static uint8_t NmpcTargetIsSafe(const BalanceNmpcTarget *target)
{
    /* 所有跨任务传递的目标都必须通过有限值、范围和腿长检查。 */
    if (target == 0 || target->valid == 0u) return 0u;
    if (!NmpcIsFiniteFloat(target->target_v) ||
        !NmpcIsFiniteFloat(target->target_wz) ||
        !NmpcIsFiniteFloat(target->target_pitch) ||
        !NmpcIsFiniteFloat(target->target_roll) ||
        !NmpcIsFiniteFloat(target->target_leg_len_l) ||
        !NmpcIsFiniteFloat(target->target_leg_len_r))
    {
        return 0u;
    }
    if (fabsf(target->target_v) > BALANCE_NMPC_V_LIMIT) return 0u;
    if (fabsf(target->target_wz) > BALANCE_NMPC_WZ_LIMIT) return 0u;
    if (fabsf(target->target_pitch) > BALANCE_NMPC_PITCH_LIMIT) return 0u;
    if (fabsf(target->target_roll) > BALANCE_NMPC_ROLL_LIMIT) return 0u;
    if (target->target_leg_len_l < L0_MIN || target->target_leg_len_l > L0_MAX) return 0u;
    if (target->target_leg_len_r < L0_MIN || target->target_leg_len_r > L0_MAX) return 0u;
    return 1u;
}

/**
 * @brief 200Hz NMPC 任务周期更新入口。
 *
 * @param state 当前平衡状态。
 * @param cmd 当前底盘控制命令。
 */
void BalanceNmpcTaskUpdate(const BalanceState *state, const Chassis_Ctrl_Cmd_s *cmd)
{
#if BALANCE_NMPC_ENABLE
    BalanceNmpcTarget next_target = {0};
    const float start_ms = DWT_GetTimeline_ms();
    uint8_t ok;
    float elapsed_ms;
    uint32_t fail_count;

    /* 校准或非站立模式下禁用 NMPC，底层按原有模式处理。 */
    if (state == 0 || cmd == 0 ||
        state->chassis.cali_flag != 0u ||
        cmd->chassis_mode != CHASSIS_STAND)
    {
        NmpcInvalidateTarget();
        nmpc_status.active = 0u;
        nmpc_status.mode = BALANCE_NMPC_DISABLED;
        NmpcResetPlannerState();
        return;
    }

    nmpc_status.mode = BALANCE_NMPC_WARMUP;
    ok = NmpcSolveReference(state, cmd, start_ms, &next_target);
    elapsed_ms = DWT_GetTimeline_ms() - start_ms;
    nmpc_status.last_elapsed_ms = elapsed_ms;

    /* 硬超时直接 fallback；连续无效解达到阈值后也 fallback。 */
    if (elapsed_ms > BALANCE_NMPC_TIMEOUT_MS)
    {
        nmpc_status.timeout_count++;
        NmpcEnterFallback();
        return;
    }
    if (ok == 0u || !NmpcTargetIsSafe(&next_target))
    {
        nmpc_status.invalid_count++;
        fail_count = nmpc_status.invalid_count + nmpc_status.timeout_count;
        if (fail_count >= BALANCE_NMPC_MAX_FAIL_COUNT)
        {
            NmpcEnterFallback();
        }
        return;
    }

    NmpcPublishTarget(&next_target);
    nmpc_status.active = 1u;
    nmpc_status.mode = BALANCE_NMPC_ACTIVE;
    nmpc_status.last_update_ms = next_target.timestamp_ms;
    nmpc_status.update_count++;
    nmpc_status.invalid_count = 0u;
    nmpc_status.timeout_count = 0u;
#else
    (void)state;
    (void)cmd;
    NmpcInvalidateTarget();
    nmpc_status.active = 0u;
    nmpc_status.mode = BALANCE_NMPC_DISABLED;
    NmpcResetPlannerState();
#endif
}

/**
 * @brief 在底层控制执行前应用最新 NMPC 目标。
 *
 * @param state 待写入目标参考的平衡状态。
 */
void BalanceNmpcApplyTarget(BalanceState *state)
{
#if BALANCE_NMPC_ENABLE
    BalanceNmpcTarget target;
    float now_ms;

    if (state == 0) return;
    if (nmpc_status.active == 0u) return;
    if (NmpcReadPublishedTarget(&target) == 0u) return;

    now_ms = DWT_GetTimeline_ms();

    /* 1000Hz/主平衡任务侧检查 200Hz NMPC 目标是否过期。 */
    if ((now_ms - target.timestamp_ms) > BALANCE_NMPC_TARGET_STALE_MS)
    {
        NmpcEnterFallback();
        return;
    }
    if (!NmpcTargetIsSafe(&target))
    {
        nmpc_status.invalid_count++;
        NmpcEnterFallback();
        return;
    }

    /* NMPC 只覆盖参考量，后续电机力矩仍由 BalanceControlUpdate() 计算。 */
    state->chassis.target_v = target.target_v;
    state->chassis.target_wz = target.target_wz;
    state->chassis.target_pitch = target.target_pitch;
    state->chassis.target_roll = target.target_roll;
    state->left.target_len = target.target_leg_len_l;
    state->right.target_len = target.target_leg_len_r;
    nmpc_status.mode = BALANCE_NMPC_ACTIVE;
#else
    (void)state;
#endif
}

/**
 * @brief 获取 NMPC 当前运行状态。
 *
 * @param status 输出状态结构体，传入空指针时不执行操作。
 */
void BalanceNmpcGetStatus(BalanceNmpcStatus *status)
{
    if (status == 0) return;
    *status = nmpc_status;
}
