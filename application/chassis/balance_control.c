#include "balance.h"
#include "tcm_region.h"

#include <math.h>

/**
 * @brief 将浮点数限制在指定闭区间内。
 *
 * @param value 待限幅的输入值。
 * @param min   输出下限。
 * @param max   输出上限。
 * @return float 若 value 超出边界则返回对应边界值，否则返回 value。
 */
static ITCM_FUNC float ClampFloat(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 将浮点数饱和到 [0, 1] 区间。
 *
 * @param value 待饱和值。
 * @return float 限幅后的归一化值。
 */
static ITCM_FUNC float SaturateFloat(float value)
{
    return ClampFloat(value, 0.0f, 1.0f);
}

#if BALANCE_TURN_LOAD_FF_ENABLE
/**
 * @brief 一阶低通滤波更新。
 *
 * @param last  上一周期滤波输出。
 * @param input 当前输入值。
 * @param alpha 滤波系数。
 * @return float 新的滤波输出。
 */
static ITCM_FUNC float LowPassFloat(float last, float input, float alpha)
{
    return last + alpha * (input - last);
}
#endif

/**
 * @brief 将线性区间内的数值映射到 [0, 1]。
 *
 * @param value 待映射值。
 * @param low   输出 0 的下边界。
 * @param high  输出 1 的上边界。
 * @return float 映射后的归一化值。
 */
static ITCM_FUNC float MapToUnit(float value, float low, float high)
{
    if (high <= low) return 0.0f;
    return SaturateFloat((value - low) / (high - low));
}

/**
 * @brief 单腿标称支撑力。
 *
 * @return float 单腿半车重支撑力,N。
 */
static ITCM_FUNC float NominalLegForce(void)
{
    return 0.5f * BODY_MASS * BALANCE_GRAVITY;
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
static ITCM_FUNC float KeepAwayFromZero(float value, float min_abs)
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
static ITCM_FUNC void ApplyLegForceFeedforward(BalanceState *state)
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
static ITCM_FUNC float CalcLegLengthDisturbanceCompensation(LinkNPodParam *leg, float dt)
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
static ITCM_FUNC float CalcLegLengthBaseForce(LinkNPodParam *leg, float dt)
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
static ITCM_FUNC void ApplyLegLengthBaseControl(BalanceState *state, float dt)
{
    state->roll_ctrl.F_base_l = CalcLegLengthBaseForce(&state->left, dt);
    state->roll_ctrl.F_base_r = CalcLegLengthBaseForce(&state->right, dt);
    state->roll_ctrl.delta_F_total = 0.0f;

    state->left.F_leg = state->roll_ctrl.F_base_l;
    state->right.F_leg = state->roll_ctrl.F_base_r;
}

/**
 * @brief 更新转向压腿惯量前馈。
 *
 * 前馈输出为左右腿力差动项：左腿加、右腿减。宏关闭时只更新 wz_dot
 * 与前馈调试量，不改变 F_leg。
 *
 * @param state 平衡状态对象。
 * @param dt    本次控制周期，单位 s。
 */
static ITCM_FUNC void ApplyTurnLoadFeedforward(BalanceState *state, float dt)
{
    LegRollControlState *ctrl = &state->roll_ctrl;
    const float target_wz = state->chassis.target_wz;
    float delta = 0.0f;

    ctrl->wz_dot = (dt > 0.0f) ? ((target_wz - ctrl->target_wz_last) / dt) : 0.0f;
    ctrl->target_wz_last = target_wz;

#if BALANCE_TURN_LOAD_FF_ENABLE
    const float nominal_force = NominalLegForce();
    delta = BALANCE_TURN_LOAD_K_VWZ_FF * state->chassis.vel * target_wz +
            BALANCE_TURN_LOAD_K_WZ2_FF * target_wz * fabsf(target_wz) +
            BALANCE_TURN_LOAD_K_WZDOT_FF * ctrl->wz_dot;
    delta = ClampFloat(delta,
                       -BALANCE_TURN_LOAD_FF_MAX_RATIO * nominal_force,
                       BALANCE_TURN_LOAD_FF_MAX_RATIO * nominal_force);
    delta = LowPassFloat(ctrl->delta_F_turn_ff,
                         delta,
                         SaturateFloat(BALANCE_TURN_LOAD_ALPHA));
    state->left.F_leg += delta;
    state->right.F_leg -= delta;
#endif

    ctrl->delta_F_turn_ff = delta;
    ctrl->delta_F_total += delta;
}

/**
 * @brief 计算 RollTask 权重。
 *
 * 低速台阶/不等腿长时提高权重；快速转向时降低硬反馈；接触不稳定时降权。
 *
 * @param state 平衡状态对象。
 * @return float roll 反馈权重。
 */
static ITCM_FUNC float CalcRollTaskWeight(const BalanceState *state)
{
    const float leg_diff = fabsf(state->left.leg_len - state->right.leg_len);
    const float target_diff = fabsf(state->left.target_len - state->right.target_len);
    const float terrain_len = MapToUnit((leg_diff > target_diff) ? leg_diff : target_diff,
                                        BALANCE_ROLL_TERRAIN_LEG_DIFF_LOW,
                                        BALANCE_ROLL_TERRAIN_LEG_DIFF_HIGH);
    const float terrain_roll = MapToUnit(fabsf(state->chassis.roll),
                                         BALANCE_ROLL_TERRAIN_ROLL_LOW,
                                         BALANCE_ROLL_TERRAIN_ROLL_HIGH);
    const float terrain_weight = 1.0f + BALANCE_ROLL_TERRAIN_GAIN *
                                 ((terrain_len > terrain_roll) ? terrain_len : terrain_roll);
    const float left_contact = state->left.contact_slip.contact_conf;
    const float right_contact = state->right.contact_slip.contact_conf;
    const float contact_conf = (left_contact < right_contact) ? left_contact : right_contact;
    const float contact_weight = (state->left.fly_flag || state->right.fly_flag) ?
                                 (0.5f * SaturateFloat(contact_conf)) :
                                 SaturateFloat(contact_conf);
    const float turn_ratio = MapToUnit(fabsf(state->chassis.target_wz),
                                       0.0f,
                                       BALANCE_ROLL_TURN_RELEASE_WZ);
    const float turn_release_weight = 1.0f -
                                      (1.0f - BALANCE_ROLL_TURN_RELEASE_MIN) * turn_ratio;
    const float weight = BALANCE_ROLL_WEIGHT_BASE *
                         terrain_weight *
                         contact_weight *
                         turn_release_weight;

    return ClampFloat(weight, BALANCE_ROLL_WEIGHT_MIN, BALANCE_ROLL_WEIGHT_MAX);
}

/**
 * @brief 更新 RollTask 专用二阶 LESO，并计算扰动补偿力。
 *
 * LESO 对象模型为 roll_ddot = f_roll + b0 * delta_F_roll_fb。
 * 宏关闭时旁路为原始 roll/roll_w，扰动补偿清零。
 *
 * @param state 平衡状态对象。
 * @return float roll 反馈使用的角速度,rad/s。
 */
static ITCM_FUNC float UpdateRollTaskLeso(BalanceState *state)
{
    LegRollControlState *ctrl = &state->roll_ctrl;
    float roll_rate_feedback = state->chassis.roll_w;

    ctrl->roll_leso_angle = state->chassis.roll;
    ctrl->roll_leso_rate = state->chassis.roll_w;
    ctrl->roll_disturbance = 0.0f;
    ctrl->roll_disturbance_comp = 0.0f;

#if BALANCE_ROLL_TASK_LESO_ENABLE
    LESO2Update(&ctrl->roll_task_leso,
                state->chassis.roll,
                ctrl->delta_F_roll_fb_last);

    ctrl->roll_leso_angle = ctrl->roll_task_leso.z1;
    ctrl->roll_leso_rate = ctrl->roll_task_leso.z2;
    ctrl->roll_disturbance = ctrl->roll_task_leso.z3;
    roll_rate_feedback = ctrl->roll_leso_rate;

    if (BALANCE_ROLL_TASK_LESO_B0 > 0.0f)
    {
        const float nominal_force = NominalLegForce();
        const float compensation = -ctrl->roll_disturbance / BALANCE_ROLL_TASK_LESO_B0;
        ctrl->roll_disturbance_comp =
            BALANCE_ROLL_DISTURBANCE_GAIN *
            ClampFloat(compensation,
                       -BALANCE_ROLL_DISTURBANCE_MAX_RATIO * nominal_force,
                       BALANCE_ROLL_DISTURBANCE_MAX_RATIO * nominal_force);
    }
#endif

    return roll_rate_feedback;
}

/**
 * @brief 应用 RollTask 反馈补偿。
 *
 * 反馈输出为左右腿力差动项：左腿加、右腿减。宏关闭时只记录参考、
 * 误差和权重，不改变 F_leg。
 *
 * @param state 平衡状态对象。
 */
static ITCM_FUNC void ApplyRollTaskFeedback(BalanceState *state)
{
    LegRollControlState *ctrl = &state->roll_ctrl;
    const float roll_rate_feedback = UpdateRollTaskLeso(state);
    float delta = 0.0f;

    ctrl->roll_ref_turn = BALANCE_ROLL_REF_K_WZ * state->chassis.target_wz +
                          BALANCE_ROLL_REF_K_VWZ * state->chassis.vel * state->chassis.target_wz;
    ctrl->roll_err = ctrl->roll_ref_turn - state->chassis.roll;
    ctrl->roll_weight = CalcRollTaskWeight(state);

#if BALANCE_ROLL_TASK_ENABLE
    const float nominal_force = NominalLegForce();
    delta = ctrl->roll_weight *
            (BALANCE_ROLL_KP * ctrl->roll_err -
             BALANCE_ROLL_KD * roll_rate_feedback +
             ctrl->roll_disturbance_comp);
    if (state->left.contact_slip.landing_flag || state->right.contact_slip.landing_flag)
    {
        delta *= BALANCE_LANDING_ROLL_FB_SCALE;
    }
    delta = ClampFloat(delta,
                       -BALANCE_ROLL_FB_MAX_RATIO * nominal_force,
                       BALANCE_ROLL_FB_MAX_RATIO * nominal_force);
    state->left.F_leg += delta;
    state->right.F_leg -= delta;
#endif

    ctrl->delta_F_roll_fb = delta;
    ctrl->delta_F_roll_fb_last = delta;
    ctrl->delta_F_total += delta;
}

/**
 * @brief 应用最终差动力限幅和接触/落地保护。
 *
 * 默认保护宏关闭时仅对最终 F_leg 做物理限幅；开启后会降低离地侧输出，
 * 避免使用离地腿强行纠 roll。
 *
 * @param state 平衡状态对象。
 */
static ITCM_FUNC void ApplyContactLandingProtection(BalanceState *state)
{
    LegRollControlState *ctrl = &state->roll_ctrl;
    const float nominal_force = NominalLegForce();
    float delta = ctrl->delta_F_total;

    delta = ClampFloat(delta,
                       -BALANCE_FINAL_DELTA_F_MAX_RATIO * nominal_force,
                       BALANCE_FINAL_DELTA_F_MAX_RATIO * nominal_force);
    ctrl->delta_F_total = delta;
    state->left.F_leg = ctrl->F_base_l + delta;
    state->right.F_leg = ctrl->F_base_r - delta;

#if BALANCE_CONTACT_PROTECT_ENABLE
    if (state->left.contact_slip.contact_flag == 0u)
    {
        state->left.F_leg *= BALANCE_AIRBORNE_FORCE_SCALE;
    }
    if (state->right.contact_slip.contact_flag == 0u)
    {
        state->right.F_leg *= BALANCE_AIRBORNE_FORCE_SCALE;
    }
#endif

    state->left.F_leg = ClampFloat(state->left.F_leg, 0.0f, LEG_FORCE_MAX);
    state->right.F_leg = ClampFloat(state->right.F_leg, 0.0f, LEG_FORCE_MAX);
}

/**
 * @brief 将虚拟腿向力和虚拟髋关节力矩投影到前后关节电机力矩。
 *
 * 基于五连杆几何雅可比，把 F_leg 与 T_hip 转换为 T_front/T_back。
 * 计算中会对关键分母做最小绝对值保护，避免接近奇异位形时除数过小。
 *
 * @param leg 待投影的单腿状态。
 */
static ITCM_FUNC void VMCProject(LinkNPodParam *leg)
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
ITCM_FUNC void BalanceControlUpdate(BalanceState *state, float dt)
{
    if (state == 0) return;

    ApplyLegLengthBaseControl(state, dt);
    ApplyTurnLoadFeedforward(state, dt);
    ApplyRollTaskFeedback(state);
    ApplyContactLandingProtection(state);
    VMCProject(&state->left);
    VMCProject(&state->right);
}
