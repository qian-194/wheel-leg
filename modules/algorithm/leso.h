#ifndef _LESO_H
#define _LESO_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 一阶线性扩张状态观测器
 *
 * 适用于一阶被控对象：y_dot = f + b0 * u。
 * z1 估计输出 y，z2 估计总扰动 f。
 */
typedef struct
{
    float b0;    // 控制增益估计值，表示输入 u 对 y_dot 的作用强度
    float wo;    // 观测器带宽，越大跟踪越快但越容易放大噪声
    float beta1; // 一阶 LESO 反馈增益：2 * wo
    float beta2; // 一阶 LESO 反馈增益：wo^2

    float z1; // 输出 y 的估计值
    float z2; // 总扰动 f 的估计值
} LESO1Instance;

typedef struct
{
    float b0;      // 控制增益估计值
    float wo;      // 观测器带宽
    float z1_init; // z1 初值，通常设为当前测量值 y
    float z2_init; // z2 初值，通常设为 0
} LESO1_Init_Config_s;

/**
 * @brief 二阶线性扩张状态观测器
 *
 * 适用于二阶被控对象：y_ddot = f + b0 * u。
 * z1 估计输出 y，z2 估计 y_dot，z3 估计总扰动 f。
 */
typedef struct
{
    float b0;    // 控制增益估计值，表示输入 u 对 y_ddot 的作用强度
    float wo;    // 观测器带宽
    float beta1; // 二阶 LESO 反馈增益：3 * wo
    float beta2; // 二阶 LESO 反馈增益：3 * wo^2
    float beta3; // 二阶 LESO 反馈增益：wo^3

    float z1; // 输出 y 的估计值
    float z2; // 输出速度 y_dot 的估计值
    float z3; // 总扰动 f 的估计值
} LESO2Instance;

typedef struct
{
    float b0;      // 控制增益估计值
    float wo;      // 观测器带宽
    float z1_init; // z1 初值，通常设为当前测量值 y
    float z2_init; // z2 初值，通常设为当前速度估计或 0
    float z3_init; // z3 初值，通常设为 0
} LESO2_Init_Config_s;

/**
 * @brief 初始化一阶 LESO
 */
void LESO1Init(LESO1Instance *leso, const LESO1_Init_Config_s *config);

/**
 * @brief 重置一阶 LESO 状态，不改变 b0 和带宽参数
 */
void LESO1Reset(LESO1Instance *leso, float z1, float z2);

/**
 * @brief 设置一阶 LESO 带宽，并按标准极点配置更新 beta
 */
void LESO1SetBandwidth(LESO1Instance *leso, float wo);

/**
 * @brief 更新一阶 LESO
 * @param y  当前测量输出
 * @param u  实际作用到对象上的控制输入
 * @param dt 控制周期，单位 s
 */
void LESO1Update(LESO1Instance *leso, float y, float u, float dt);

/**
 * @brief 初始化二阶 LESO
 */
void LESO2Init(LESO2Instance *leso, const LESO2_Init_Config_s *config);

/**
 * @brief 重置二阶 LESO 状态，不改变 b0 和带宽参数
 */
void LESO2Reset(LESO2Instance *leso, float z1, float z2, float z3);

/**
 * @brief 设置二阶 LESO 带宽，并按标准极点配置更新 beta
 */
void LESO2SetBandwidth(LESO2Instance *leso, float wo);

/**
 * @brief 更新二阶 LESO
 * @param y  当前测量输出
 * @param u  实际作用到对象上的控制输入
 * @param dt 控制周期，单位 s
 */
void LESO2Update(LESO2Instance *leso, float y, float u, float dt);

#ifdef __cplusplus
}
#endif

#endif
