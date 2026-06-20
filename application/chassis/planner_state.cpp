#include "planner_state.h"

// 规划器状态提取：把 BalanceState 的估计结果拷成扁平 PlannerState。
// 纯读取，无估计逻辑——估计在 balance_state / 各子模块里已完成。
namespace balance::state
{

extern "C" void BalancePlannerStateExtract(const BalanceState *state, PlannerState *out)
{
    if (state == 0 || out == 0) return;

    out->v = state->chassis.vel;
    out->wz = state->chassis.wz;
    out->pitch = state->chassis.pitch;
    out->pitch_w = state->chassis.pitch_w;
    out->roll = state->chassis.roll;
    out->roll_w = state->chassis.roll_w;

    out->left_leg_len = state->left.leg_len;
    out->right_leg_len = state->right.leg_len;

    out->left_contact_conf = state->left.contact_slip.contact_conf;
    out->right_contact_conf = state->right.contact_slip.contact_conf;
    out->left_slip_conf = state->left.contact_slip.slip_conf;
    out->right_slip_conf = state->right.contact_slip.slip_conf;
    out->left_contact_state = state->left.contact_slip.contact_state;
    out->right_contact_state = state->right.contact_slip.contact_state;
}

} // namespace balance::state
