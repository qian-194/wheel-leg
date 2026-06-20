#pragma once

// 底盘/速度估计：IMU 状态装配、机体速度与里程估计、pitch/roll 姿态 LESO、
// 左右轮速 LESO。LESO 步长由初始化频率派生，update 仅用 dt 做回路门控。

#include "balance.h"

namespace balance::state
{

/**
 * @brief 从 IMU 数据装配底盘姿态、角速度和加速度状态。
 *
 * @param chassis 待写入的底盘状态。
 * @param imu     当前 IMU 姿态数据。
 */
void AssembleImuState(ChassisParam *chassis, const attitude_t *imu) noexcept;

/**
 * @brief 估计左右腿轮速、机体速度、底盘线速度和里程积分。
 *
 * @param left    左腿状态。
 * @param right   右腿状态。
 * @param chassis 底盘状态。
 * @param dt      本次状态更新时间，单位 s。
 */
void EstimateSpeed(LinkNPodParam *left,
                   LinkNPodParam *right,
                   ChassisParam *chassis,
                   float dt) noexcept;

/**
 * @brief 更新 pitch/roll 的 LESO 估计状态或旁路原始 IMU 状态。
 *
 * @param state 平衡状态对象。
 * @param dt    本次状态更新时间，单位 s；此处主要作为回路有效性门控。
 */
void UpdateAttitudeLesoState(BalanceState *state, float dt) noexcept;

/**
 * @brief 更新左右轮速 LESO 估计状态或旁路原始轮速。
 *
 * @param left  左腿状态。
 * @param right 右腿状态。
 * @param dt    本次状态更新时间，单位 s；此处主要作为回路有效性门控。
 */
void UpdateWheelSpeedLesoState(LinkNPodParam *left,
                               LinkNPodParam *right,
                               float dt) noexcept;

} // namespace balance::state
