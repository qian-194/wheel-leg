#pragma once

// 接触/打滑估计：从支撑力与轮速残差推算每条腿的接触置信度、打滑置信度，
// 带滞回的二值标志，以及接触状态机（接触/轻载/离地/落地保护）。
// 首版只输出状态，不直接参与控制。

#include "balance.h"

namespace balance::state
{

/**
 * @brief 初始化单腿接触与打滑状态。
 *
 * 默认认为腿已接触地面，接触置信度为 1，打滑相关计时与标志清零。
 *
 * @param state 待初始化的接触/打滑状态对象。
 */
void InitContactSlipState(LegContactSlipState *state) noexcept;

/**
 * @brief 更新左右腿打滑状态。
 *
 * 根据两轮差速模型从底盘线速度和 yaw 角速度推算左右轮理论轮速，
 * 再分别更新左右腿打滑检测状态。
 *
 * @param left    左腿状态。
 * @param right   右腿状态。
 * @param chassis 底盘状态。
 * @param dt      本次状态更新时间，单位 s。
 */
void UpdateSlipState(LinkNPodParam *left,
                     LinkNPodParam *right,
                     const ChassisParam *chassis,
                     float dt) noexcept;

/**
 * @brief 更新左右腿接触状态。
 *
 * 使用半车重作为标称单腿支撑力，分别更新左右腿接触置信度、状态机和离地标志。
 *
 * @param left  左腿状态。
 * @param right 右腿状态。
 * @param dt    本次状态更新时间，单位 s。
 */
void UpdateContactState(LinkNPodParam *left,
                        LinkNPodParam *right,
                        float dt) noexcept;

} // namespace balance::state
