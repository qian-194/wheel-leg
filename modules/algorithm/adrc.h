#ifndef _ADRC_H
#define _ADRC_H

#include "leso.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 一阶线性 ADRC 初始化参数
 */
typedef struct
{
    float b0;      // 控制增益估计值，应与对象 y_dot = f + b0 * u 的 b0 一致
    float wo;      // LESO 观测器带宽
    float freq;    // 控制计算频率，Hz，透传给 LESO 派生 dt = 1/freq；<=0 视为非法
    float kp;      // 一阶误差反馈增益
    float max_out; // 输出绝对值限幅，<= 0 表示不启用限幅
    float z1_init; // LESO z1 初值
    float z2_init; // LESO z2 初值
} LADRC1_Init_Config_s;

/**
 * @brief 一阶线性 ADRC
 *
 * 控制律：u = (kp * (ref - z1) - z2) / b0。
 * 其中 z2 为总扰动估计，控制输出中会主动抵消该扰动。
 */
typedef struct
{
    LESO1Instance eso; // 一阶 LESO，估计 y 和总扰动

    float b0;      // 控制增益估计值
    float kp;      // 误差反馈增益
    float max_out; // 输出绝对值限幅

    float err;    // 当前跟踪误差：ref - z1
    float u0;     // 未补偿扰动前的虚拟控制量
    float output; // 最终控制输出
} LADRC1Instance;

/**
 * @brief 二阶线性 ADRC 初始化参数
 */
typedef struct
{
    float b0;      // 控制增益估计值，应与对象 y_ddot = f + b0 * u 的 b0 一致
    float wo;      // LESO 观测器带宽
    float freq;    // 控制计算频率，Hz，透传给 LESO 派生 dt = 1/freq；<=0 视为非法
    float kp;      // 位置误差反馈增益
    float kd;      // 速度误差反馈增益
    float max_out; // 输出绝对值限幅，<= 0 表示不启用限幅
    float z1_init; // LESO z1 初值
    float z2_init; // LESO z2 初值
    float z3_init; // LESO z3 初值
} LADRC2_Init_Config_s;

/**
 * @brief 二阶线性 ADRC
 *
 * 控制律：u = (kp * (ref - z1) + kd * (ref_dot - z2) - z3) / b0。
 * 其中 z3 为总扰动估计。
 */
typedef struct
{
    LESO2Instance eso; // 二阶 LESO，估计 y、y_dot 和总扰动

    float b0;      // 控制增益估计值
    float kp;      // 位置误差反馈增益
    float kd;      // 速度误差反馈增益
    float max_out; // 输出绝对值限幅

    float err;    // 位置误差：ref - z1
    float derr;   // 速度误差：ref_dot - z2
    float u0;     // 未补偿扰动前的虚拟控制量
    float output; // 最终控制输出
} LADRC2Instance;

/**
 * @brief 初始化一阶 ADRC
 */
void LADRC1Init(LADRC1Instance *adrc, const LADRC1_Init_Config_s *config);

/**
 * @brief 重置一阶 ADRC 状态，不改变参数
 */
void LADRC1Reset(LADRC1Instance *adrc, float z1, float z2);

/**
 * @brief 更新一阶 ADRC，默认认为上次输出已经作用到对象上
 *
 * 离散步长由初始化时的 freq 决定，无需传入 dt。
 */
float LADRC1Update(LADRC1Instance *adrc, float ref, float y);

/**
 * @brief 更新一阶 ADRC，并显式传入实际执行的控制输入
 *
 * 当输出经过电机限幅、斜坡限制或安全裁剪后，应传入实际作用值 applied_u，
 * 这样 LESO 才能用真实输入估计扰动。离散步长由初始化时的 freq 决定。
 */
float LADRC1UpdateWithInput(LADRC1Instance *adrc, float ref, float y, float applied_u);

/**
 * @brief 初始化二阶 ADRC
 */
void LADRC2Init(LADRC2Instance *adrc, const LADRC2_Init_Config_s *config);

/**
 * @brief 重置二阶 ADRC 状态，不改变参数
 */
void LADRC2Reset(LADRC2Instance *adrc, float z1, float z2, float z3);

/**
 * @brief 更新二阶 ADRC，默认认为上次输出已经作用到对象上
 *
 * 离散步长由初始化时的 freq 决定，无需传入 dt。
 */
float LADRC2Update(LADRC2Instance *adrc, float ref, float ref_dot, float y);

/**
 * @brief 更新二阶 ADRC，并显式传入实际执行的控制输入
 *
 * 离散步长由初始化时的 freq 决定，无需传入 dt。
 */
float LADRC2UpdateWithInput(LADRC2Instance *adrc, float ref, float ref_dot, float y, float applied_u);

#ifdef __cplusplus
}
#endif

#endif
