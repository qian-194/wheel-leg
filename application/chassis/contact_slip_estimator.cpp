#include "contact_slip_estimator.hpp"

#include "state_math.hpp"

#include <math.h>
#include <string.h>

namespace balance::state
{

void InitContactSlipState(LegContactSlipState *state) noexcept
{
    if (state == 0) return;

    memset(state, 0, sizeof(*state));
    state->contact_conf = 1.0f;
    state->contact_flag = 1u;
    state->contact_state = LEG_CONTACT;
}

/**
 * @brief 根据打滑置信度和确认时间更新打滑标志。
 *
 * 打滑置位和清除均带时间确认，避免瞬时轮速残差导致标志抖动。
 */
static void UpdateSlipFlag(LegContactSlipState *state, float dt) noexcept
{
    if (state == 0 || dt <= 0.0f) return;

    if (state->slip_flag == 0u)
    {
        state->slip_off_time = 0.0f;
        if (state->slip_conf > BALANCE_SLIP_ON_CONF)
        {
            state->slip_on_time += dt;
            if (state->slip_on_time >= BALANCE_SLIP_ON_TIME)
            {
                state->slip_flag = 1u;
                state->slip_on_time = 0.0f;
            }
        }
        else
        {
            state->slip_on_time = 0.0f;
        }
    }
    else
    {
        state->slip_on_time = 0.0f;
        if (state->slip_conf < BALANCE_SLIP_OFF_CONF)
        {
            state->slip_off_time += dt;
            if (state->slip_off_time >= BALANCE_SLIP_OFF_TIME)
            {
                state->slip_flag = 0u;
                state->slip_off_time = 0.0f;
            }
        }
        else
        {
            state->slip_off_time = 0.0f;
        }
    }
}

/**
 * @brief 更新单腿打滑分数、打滑置信度和打滑标志。
 *
 * 通过实测轮速与差速模型预测轮速的相对残差评估打滑程度，
 * 并对打滑分数做一阶低通滤波。
 */
static void UpdateLegSlipState(LinkNPodParam *leg,
                               float predicted_wheel_w,
                               float dt) noexcept
{
    if (leg == 0 || dt <= 0.0f) return;

    const float residual = leg->wheel_w - predicted_wheel_w;
    const float denom = Max3Float(fabsf(leg->wheel_w),
                                  fabsf(predicted_wheel_w),
                                  BALANCE_SLIP_W_MIN);
    const float raw_score = fabsf(residual) / denom;

    leg->contact_slip.slip_score = LowPassFloat(leg->contact_slip.slip_score,
                                                raw_score,
                                                BALANCE_SLIP_ALPHA);
    leg->contact_slip.slip_conf = MapToConfidence(leg->contact_slip.slip_score,
                                                  BALANCE_SLIP_LOW,
                                                  BALANCE_SLIP_HIGH);
    UpdateSlipFlag(&leg->contact_slip, dt);
}

void UpdateSlipState(LinkNPodParam *left,
                     LinkNPodParam *right,
                     const ChassisParam *chassis,
                     float dt) noexcept
{
    if (left == 0 || right == 0 || chassis == 0 || dt <= 0.0f) return;

    const float left_w_pred = (chassis->vel - TWO_WHEEL_HALF_TRACK_M * chassis->wz) / WHEEL_RADIUS;
    const float right_w_pred = (chassis->vel + TWO_WHEEL_HALF_TRACK_M * chassis->wz) / WHEEL_RADIUS;

    UpdateLegSlipState(left, left_w_pred, dt);
    UpdateLegSlipState(right, right_w_pred, dt);
}

/**
 * @brief 根据接触置信度和确认时间更新二值接触标志。
 *
 * 接触丢失和恢复均带时间确认，避免支撑力估计短时波动造成离地/落地误判。
 */
static void UpdateContactFlag(LegContactSlipState *state, float dt) noexcept
{
    if (state == 0 || dt <= 0.0f) return;

    if (state->contact_flag != 0u)
    {
        state->contact_on_time = 0.0f;
        if (state->contact_conf < BALANCE_CONTACT_OFF_CONF)
        {
            state->contact_off_time += dt;
            if (state->contact_off_time >= BALANCE_CONTACT_OFF_TIME)
            {
                state->contact_flag = 0u;
                state->contact_off_time = 0.0f;
            }
        }
        else
        {
            state->contact_off_time = 0.0f;
        }
    }
    else
    {
        state->contact_off_time = 0.0f;
        if (state->contact_conf > BALANCE_CONTACT_ON_CONF)
        {
            state->contact_on_time += dt;
            if (state->contact_on_time >= BALANCE_CONTACT_ON_TIME)
            {
                state->contact_flag = 1u;
                state->contact_on_time = 0.0f;
            }
        }
        else
        {
            state->contact_on_time = 0.0f;
        }
    }
}

/**
 * @brief 更新单腿接触状态机。
 *
 * 状态机覆盖正常接触、轻载、离地和落地保护阶段；落地阶段会保持
 * landing_flag 一段保护时间，供控制层在落地瞬间做输出保护。
 */
static void UpdateContactStateMachine(LegContactSlipState *state, float dt) noexcept
{
    if (state == 0 || dt <= 0.0f) return;

    switch (state->contact_state)
    {
    case LEG_CONTACT:
        state->landing_flag = 0u;
        state->landing_time = 0.0f;
        if (state->contact_conf < BALANCE_CONTACT_LIGHT_UNLOAD_CONF)
        {
            state->contact_state = LEG_LIGHT_UNLOAD;
        }
        break;

    case LEG_LIGHT_UNLOAD:
        if (state->contact_conf >= BALANCE_CONTACT_ON_CONF)
        {
            state->contact_state = LEG_CONTACT;
        }
        else if (state->contact_conf < BALANCE_CONTACT_AIRBORNE_CONF &&
                 state->contact_flag == 0u)
        {
            state->contact_state = LEG_AIRBORNE;
        }
        break;

    case LEG_AIRBORNE:
        if (state->contact_conf > BALANCE_CONTACT_LANDING_CONF &&
            state->contact_flag != 0u)
        {
            state->contact_state = LEG_LANDING;
            state->landing_flag = 1u;
            state->landing_time = 0.0f;
        }
        break;

    case LEG_LANDING:
        state->landing_flag = 1u;
        state->landing_time += dt;
        if (state->landing_time >= BALANCE_LANDING_PROTECT_TIME)
        {
            state->landing_flag = 0u;
            state->landing_time = 0.0f;
            state->contact_state = (state->contact_conf >= BALANCE_CONTACT_LIGHT_UNLOAD_CONF) ?
                                   LEG_CONTACT :
                                   LEG_LIGHT_UNLOAD;
        }
        break;

    default:
        state->contact_state = LEG_CONTACT;
        state->landing_flag = 0u;
        state->landing_time = 0.0f;
        break;
    }
}

/**
 * @brief 更新单腿法向力、接触置信度、接触标志和离地标志。
 *
 * 当前法向力估计直接使用上一控制周期写入的 F_leg，并映射为接触置信度。
 */
static void UpdateLegContactState(LinkNPodParam *leg, float nominal_force, float dt) noexcept
{
    if (leg == 0 || dt <= 0.0f) return;

    // F_leg 在 BalanceControlUpdate() 中更新，因此这里使用的是上一控制周期的腿向力。
    leg->normal_force = leg->F_leg;

    const float support_score = (nominal_force > 0.0f) ?
                                ClampFloat(leg->normal_force / nominal_force, 0.0f, 2.0f) :
                                0.0f;
    const float contact_raw = MapToConfidence(support_score,
                                              BALANCE_CONTACT_OFF_RATIO,
                                              BALANCE_CONTACT_ON_RATIO);

    leg->contact_slip.contact_conf = LowPassFloat(leg->contact_slip.contact_conf,
                                                  contact_raw,
                                                  BALANCE_CONTACT_ALPHA);
    UpdateContactFlag(&leg->contact_slip, dt);
    UpdateContactStateMachine(&leg->contact_slip, dt);

    leg->fly_flag = (leg->contact_slip.contact_flag == 0u) ? 1u : 0u;
}

void UpdateContactState(LinkNPodParam *left,
                        LinkNPodParam *right,
                        float dt) noexcept
{
    if (left == 0 || right == 0 || dt <= 0.0f) return;

    const float nominal_force = 0.5f * BODY_MASS * BALANCE_GRAVITY;

    UpdateLegContactState(left, nominal_force, dt);
    UpdateLegContactState(right, nominal_force, dt);
}

} // namespace balance::state
