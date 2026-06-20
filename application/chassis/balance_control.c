#include "balance.h"

#include <math.h>

/**
 * @brief 将浮点数限制在指定闭区间内。
 *
 * @param value 待限幅的输入值。
 * @param min   输出下限。
 * @param max   输出上限。
 * @return float 若 value 超出边界则返回对应边界值，否则返回 value。
 */
static float ClampFloat(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 保证数值远离 0，避免后续作为除数或三角函数分母时过小。
 *
 * 当 value 落在 (-min_abs, min_abs) 区间内时，按原符号钳到 +/-min_abs。
 *
 * @param value   待处理的输入值。
 * @param min_abs 最小绝对值。
 * @return float 远离 0 后的值。
 */
static float KeepAwayFromZero(float value, float min_abs)
{
    if (value >= 0.0f && value < min_abs) return min_abs;
    if (value < 0.0f && value > -min_abs) return -min_abs;
    return value;
}

/**
 * @brief 给左右腿写入基础重力支撑前馈。
 *
 * 当前首版仅按半车质量给每条腿分配腿向支撑力，输出会被限制到
 * [0, LEG_FORCE_MAX]，用于在控制器未完全接入时保持基本支撑。
 *
 * @param state 平衡状态对象。
 */
static void ApplyLegForceFeedforward(BalanceState *state)
{
    const float base_force = 0.5f * BODY_MASS * BALANCE_GRAVITY * LEG_GRAVITY_FF_GAIN;

    // 首版只给腿长支撑力基础前馈；roll/pitch 四关节静态前馈后续放在 VMC 后、限幅前处理。
    state->left.F_leg = ClampFloat(base_force, 0.0f, LEG_FORCE_MAX);
    state->right.F_leg = ClampFloat(base_force, 0.0f, LEG_FORCE_MAX);
}

/**
 * @brief 计算腿长方向 ADRC 扰动补偿力。
 *
 * ADRC 使能时使用上一周期实际写入的 F_leg 更新二阶 LESO，只取 z3 作为
 * 扰动估计并按 b0 换算为补偿力；完整 LADRC 输出仅保存用于调试观察。
 * ADRC 未使能或输入非法时返回 0。
 *
 * @param leg 待补偿的单腿状态。
 * @param dt  本次控制周期，单位 s；当前用于回路有效性门控。
 * @return float 需要叠加到腿长 PD 输出上的扰动补偿力，单位 N。
 */
static float CalcLegLengthDisturbanceCompensation(LinkNPodParam *leg, float dt)
{
#if LEG_LEN_ADRC_ENABLE
    if (leg == 0 || dt <= 0.0f || LEG_LEN_ADRC_B0 <= 0.0f)
        return 0.0f;

    // 使用上一周期实际写入的 F_leg 更新 LESO，避免用未限幅的理论输出污染扰动估计。
    // LESO 步长由初始化频率派生，不再传 dt；外层 dt 仍用作"回路是否在跑"的门控。
    leg->leg_len_adrc_output = LADRC2UpdateWithInput(&leg->leg_len_adrc,
                                                     leg->target_len,
                                                     0.0f,
                                                     leg->leg_len,
                                                     leg->F_leg);
    leg->leg_len_disturbance_acc = leg->leg_len_adrc.eso.z3;

    const float compensation = ClampFloat(-leg->leg_len_disturbance_acc / LEG_LEN_ADRC_B0,
                                          -LEG_LEN_ADRC_DISTURBANCE_MAX,
                                          LEG_LEN_ADRC_DISTURBANCE_MAX);
    leg->leg_len_disturbance_force = LEG_LEN_ADRC_DISTURBANCE_GAIN * compensation;
    return leg->leg_len_disturbance_force;
#else
    (void)leg;
    (void)dt;
    return 0.0f;
#endif
}

/**
 * @brief 计算单腿腿长方向支撑力。
 *
 * 输出由腿长 PD、重力前馈和可选 ADRC 扰动补偿组成，最终限制在
 * [0, LEG_FORCE_MAX]。
 *
 * @param leg 待计算的单腿状态。
 * @param dt  本次控制周期，单位 s。
 * @return float 单腿腿向支撑力，单位 N。
 */
static float CalcLegLengthForce(LinkNPodParam *leg, float dt)
{
    const float gravity_ff = 0.5f * BODY_MASS * BALANCE_GRAVITY * LEG_GRAVITY_FF_GAIN;
    const float len_error = leg->target_len - leg->leg_len;
    const float disturbance_compensation = CalcLegLengthDisturbanceCompensation(leg, dt);
    const float force = LEG_LEN_KP * len_error -
                        LEG_LEN_KD * leg->legd +
                        gravity_ff +
                        disturbance_compensation;

    return ClampFloat(force, 0.0f, LEG_FORCE_MAX);
}

/**
 * @brief 更新左右腿腿长支撑力。
 *
 * 分别调用单腿腿长力计算函数，并把结果写入 left.F_leg 与 right.F_leg。
 *
 * @param state 平衡状态对象。
 * @param dt    本次控制周期，单位 s。
 */
static void ApplyLegLengthControl(BalanceState *state, float dt)
{
    state->left.F_leg = CalcLegLengthForce(&state->left, dt);
    state->right.F_leg = CalcLegLengthForce(&state->right, dt);
}

/**
 * @brief 将虚拟腿向力和虚拟髋关节力矩投影到前后关节电机力矩。
 *
 * 基于五连杆几何雅可比，把 F_leg 与 T_hip 转换为 T_front/T_back。
 * 计算中会对关键分母做最小绝对值保护，避免接近奇异位形时除数过小。
 *
 * @param leg 待投影的单腿状态。
 */
static void VMCProject(LinkNPodParam *leg)
{
    const float phi12 = leg->phi1 - leg->phi2;
    const float phi34 = leg->phi3 - leg->phi4;
    float phi32 = leg->phi3 - leg->phi2;
    const float phi03 = leg->phi0 - leg->phi3;
    const float phi02 = leg->phi0 - leg->phi2;
    const float force_moment = leg->F_leg * leg->leg_len;

    phi32 = KeepAwayFromZero(phi32, LEG_VMC_SIN_MIN);
    const float denom = leg->leg_len * sinf(phi32);

    leg->T_back = -(THIGH_LEN * sinf(phi12) *
                    (force_moment * sinf(phi03) + leg->T_hip * cosf(phi03))) /
                  denom;
    leg->T_front = -(THIGH_LEN * sinf(phi34) *
                     (force_moment * sinf(phi02) + leg->T_hip * cosf(phi02))) /
                   denom;
}

/**
 * @brief 平衡控制主更新入口。
 *
 * 当前控制层计算腿长支撑力，并通过 VMC 将左右腿虚拟力投影为关节力矩。
 * 函数只更新 BalanceState 中的控制输出字段，不直接写电机。
 *
 * @param state 平衡状态对象。
 * @param dt    本次控制周期，单位 s。
 */
void BalanceControlUpdate(BalanceState *state, float dt)
{
    if (state == 0) return;

    ApplyLegLengthControl(state, dt);
    VMCProject(&state->left);
    VMCProject(&state->right);
}
