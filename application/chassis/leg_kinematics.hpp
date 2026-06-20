#pragma once

// 五连杆腿部运动学：从关节角与底盘 pitch 解算单腿末端、腿长及相关速度。
// 纯几何计算，无状态、无副作用（仅写入传入的 LinkNPodParam）。

#include "balance.h"

namespace balance::state
{

/**
 * @brief 根据五连杆几何关系计算单腿末端、腿长和相关速度状态。
 *
 * 输入为已装配的关节角与底盘 pitch，输出腿部关键角度、端点坐标、
 * 腿长、腿长速度、摆杆角 theta、theta_w、高度与高度速度。
 *
 * @param p       待更新的单腿状态。
 * @param chassis 当前底盘状态，用于扣除机体 pitch 与 pitch_w。
 */
void Link2Leg(LinkNPodParam *p, const ChassisParam *chassis) noexcept;

} // namespace balance::state
