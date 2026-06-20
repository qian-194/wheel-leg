#ifndef _PLANNER_STATE_H
#define _PLANNER_STATE_H

// PlannerState：规划层（NMPC）唯一的状态消费面。
//
// 目的是把规划器与 BalanceState 的内部字段解耦——NMPC 只读这个扁平结构，
// 不再到处摸 BalanceState 的腿/底盘内部状态。字段集保持精简、稳定，
// 后续接入 NMPC 状态向量与约束输入时以此为契约。
//
// 该头为 C/C++ 双兼容（纯 POD struct + extern "C" 提取函数），
// 便于 C 或 C++ 编写的规划器直接 include。

#include "balance.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 规划层消费的扁平底盘状态快照。
 *
 * 所有量取自 BalanceState 的估计结果，符号/单位与各源字段一致：
 * 线速度 m/s，角速度 rad/s，角度 rad，腿长 m，置信度 0~1。
 */
typedef struct
{
    // 底盘运动状态
    float v;        // 底盘线速度估计，m/s
    float wz;       // 偏航角速度，rad/s
    float pitch;    // 俯仰角，rad
    float pitch_w;  // 俯仰角速度，rad/s
    float roll;     // 横滚角，rad
    float roll_w;   // 横滚角速度，rad/s

    // 左右腿长
    float left_leg_len;  // 左腿长，m
    float right_leg_len; // 右腿长，m

    // 左右腿接触/打滑置信度与状态（首版仅供规划器参考，不强约束）
    float left_contact_conf;  // 左腿接触置信度，0~1
    float right_contact_conf; // 右腿接触置信度，0~1
    float left_slip_conf;     // 左腿打滑置信度，0~1
    float right_slip_conf;    // 右腿打滑置信度，0~1
    LegContactState_e left_contact_state;  // 左腿接触状态机状态
    LegContactState_e right_contact_state; // 右腿接触状态机状态
} PlannerState;

/**
 * @brief 从 BalanceState 提取规划器消费面。
 *
 * 只读 state，把规划所需字段拷进 out；不修改 state，不做任何估计计算。
 * state 或 out 为空时安全返回（out 不被写入）。
 *
 * @param state 已完成本周期估计的平衡状态对象（输入）。
 * @param out   待填充的规划器状态快照（输出）。
 */
void BalancePlannerStateExtract(const BalanceState *state, PlannerState *out);

#ifdef __cplusplus
}
#endif

#endif
