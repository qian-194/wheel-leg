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

static float CalcLegLengthForce(const LinkNPodParam *leg)
{
    const float gravity_ff = 0.5f * BODY_MASS * BALANCE_GRAVITY * LEG_GRAVITY_FF_GAIN;
    const float len_error = leg->target_len - leg->leg_len;
    const float force = LEG_LEN_KP * len_error - LEG_LEN_KD * leg->legd + gravity_ff;

    return ClampFloat(force, 0.0f, LEG_FORCE_MAX);
}

static void ApplyLegLengthControl(BalanceState *state)
{
    state->left.F_leg = CalcLegLengthForce(&state->left);
    state->right.F_leg = CalcLegLengthForce(&state->right);
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

    ApplyLegLengthControl(state);
    VMCProject(&state->left);
    VMCProject(&state->right);

    (void)dt;
}
