#pragma once

// 平衡状态估计层共享的标量工具函数。
//
// 都是无副作用的纯函数，做成 header-only inline，供 kinematics / estimator /
// contact 各子模块复用。位于 balance::state 命名空间，不导出全局符号。

#include <math.h>

namespace balance::state
{

/**
 * @brief 将浮点数限制在指定闭区间内。
 */
inline float ClampFloat(float value, float min, float max) noexcept
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 计算非负安全平方根。
 *
 * 运动学计算中可能因浮点误差出现很小的负数，本函数将负输入钳到 0，
 * 避免 sqrtf() 产生 NaN 并污染后续状态。
 */
inline float SafeSqrt(float x) noexcept
{
    return sqrtf(x > 0.0f ? x : 0.0f);
}

/**
 * @brief 返回三个浮点数中的最大值。
 */
inline float Max3Float(float a, float b, float c) noexcept
{
    float max = (a > b) ? a : b;
    return (max > c) ? max : c;
}

/**
 * @brief 将浮点数饱和到 [0, 1] 区间。
 */
inline float SaturateFloat(float value) noexcept
{
    return ClampFloat(value, 0.0f, 1.0f);
}

/**
 * @brief 一阶低通滤波更新。
 *
 * @param last  上一周期滤波输出。
 * @param input 当前输入值。
 * @param alpha 滤波系数，通常位于 [0, 1]，越大响应越快。
 */
inline float LowPassFloat(float last, float input, float alpha) noexcept
{
    return last + alpha * (input - last);
}

/**
 * @brief 将线性区间内的数值映射为 [0, 1] 置信度。
 *
 * value <= low 时输出 0，value >= high 时输出 1，中间线性插值。
 * 若 high <= low，返回 0 以避免除零。
 */
inline float MapToConfidence(float value, float low, float high) noexcept
{
    if (high <= low) return 0.0f;
    return SaturateFloat((value - low) / (high - low));
}

} // namespace balance::state
