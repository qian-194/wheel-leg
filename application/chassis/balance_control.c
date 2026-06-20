#include "balance.h"

#include <math.h>

static float ClampFloat(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static float KeepAwayFromZero(float value, float min_abs)
{
    if (value >= 0.0f && value < min_abs) return min_abs;
    if (value < 0.0f && value > -min_abs) return -min_abs;
    return value;
}

static void ApplyLegForceFeedforward(BalanceState *state)
{
    const float base_force = 0.5f * BODY_MASS * BALANCE_GRAVITY * LEG_GRAVITY_FF_GAIN;

    // 首版只给腿长支撑力基础前馈；roll/pitch 四关节静态前馈后续放在 VMC 后、限幅前处理。
    state->left.F_leg = ClampFloat(base_force, 0.0f, LEG_FORCE_MAX);
    state->right.F_leg = ClampFloat(base_force, 0.0f, LEG_FORCE_MAX);
}

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

static void ApplyLegLengthControl(BalanceState *state, float dt)
{
    state->left.F_leg = CalcLegLengthForce(&state->left, dt);
    state->right.F_leg = CalcLegLengthForce(&state->right, dt);
}

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

void BalanceControlUpdate(BalanceState *state, float dt)
{
    if (state == 0) return;

    ApplyLegLengthControl(state, dt);
    VMCProject(&state->left);
    VMCProject(&state->right);
}
