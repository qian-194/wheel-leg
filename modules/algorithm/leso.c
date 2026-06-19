#include "leso.h"

/**
 * @brief 设置一阶 LESO 带宽
 *
 * 一阶 LESO 的特征多项式按 (s + wo)^2 配置：
 * beta1 = 2 * wo, beta2 = wo^2。
 *
 * @param leso 一阶 LESO 实例
 * @param wo   观测器带宽，必须大于 0；小于等于 0 时关闭观测反馈
 */
void LESO1SetBandwidth(LESO1Instance *leso, float wo)
{
    if (leso == 0) return;

    leso->wo = wo;
    if (wo > 0.0f)
    {
        leso->beta1 = 2.0f * wo;
        leso->beta2 = wo * wo;
    }
    else
    {
        leso->beta1 = 0.0f;
        leso->beta2 = 0.0f;
    }
}

/**
 * @brief 初始化一阶 LESO
 *
 * 初始化会写入控制增益估计值 b0，按 wo 计算 beta，并设置 z1/z2 初值。
 * 常见用法是 z1_init = 当前测量 y，z2_init = 0。
 *
 * @param leso   一阶 LESO 实例
 * @param config 初始化参数
 */
void LESO1Init(LESO1Instance *leso, const LESO1_Init_Config_s *config)
{
    if (leso == 0 || config == 0) return;

    leso->b0 = config->b0;
    LESO1SetBandwidth(leso, config->wo);
    LESO1Reset(leso, config->z1_init, config->z2_init);
}

/**
 * @brief 重置一阶 LESO 状态
 *
 * 只重置观测状态 z1/z2，不改变 b0、wo 和 beta。适合模式切换、
 * 刚使能控制器、传感器重新校准后使用。
 *
 * @param leso 一阶 LESO 实例
 * @param z1   输出 y 的估计初值
 * @param z2   总扰动估计初值
 */
void LESO1Reset(LESO1Instance *leso, float z1, float z2)
{
    if (leso == 0) return;

    leso->z1 = z1;
    leso->z2 = z2;
}

/**
 * @brief 更新一阶 LESO 状态
 *
 * 对象模型：y_dot = f + b0 * u。
 * z1 估计 y，z2 估计总扰动 f。函数内部采用显式欧拉离散，
 * 因此调用方应传入稳定、真实的控制周期 dt。
 *
 * @param leso 一阶 LESO 实例
 * @param y    当前测量输出
 * @param u    实际作用到对象上的控制输入
 * @param dt   控制周期，单位 s
 */
void LESO1Update(LESO1Instance *leso, float y, float u, float dt)
{
    if (leso == 0 || dt <= 0.0f) return;

    // 观测误差使用 z1 - y，反馈项写成 -beta * e。
    const float e = leso->z1 - y;

    // 一阶对象模型：y_dot = f + b0 * u，其中 z2 用来估计总扰动 f。
    const float z1_dot = leso->z2 + leso->b0 * u - leso->beta1 * e;
    const float z2_dot = -leso->beta2 * e;

    // 当前实现采用显式欧拉离散，调用方必须传入稳定的控制周期 dt。
    leso->z1 += z1_dot * dt;
    leso->z2 += z2_dot * dt;
}

/**
 * @brief 设置二阶 LESO 带宽
 *
 * 二阶 LESO 的特征多项式按 (s + wo)^3 配置：
 * beta1 = 3 * wo, beta2 = 3 * wo^2, beta3 = wo^3。
 *
 * @param leso 二阶 LESO 实例
 * @param wo   观测器带宽，必须大于 0；小于等于 0 时关闭观测反馈
 */
void LESO2SetBandwidth(LESO2Instance *leso, float wo)
{
    if (leso == 0) return;

    leso->wo = wo;
    if (wo > 0.0f)
    {
        leso->beta1 = 3.0f * wo;
        leso->beta2 = 3.0f * wo * wo;
        leso->beta3 = wo * wo * wo;
    }
    else
    {
        leso->beta1 = 0.0f;
        leso->beta2 = 0.0f;
        leso->beta3 = 0.0f;
    }
}

/**
 * @brief 初始化二阶 LESO
 *
 * 初始化会写入控制增益估计值 b0，按 wo 计算 beta，并设置 z1/z2/z3 初值。
 * 常见用法是 z1_init = 当前测量 y，z2_init = 当前速度估计或 0，z3_init = 0。
 *
 * @param leso   二阶 LESO 实例
 * @param config 初始化参数
 */
void LESO2Init(LESO2Instance *leso, const LESO2_Init_Config_s *config)
{
    if (leso == 0 || config == 0) return;

    leso->b0 = config->b0;
    LESO2SetBandwidth(leso, config->wo);
    LESO2Reset(leso, config->z1_init, config->z2_init, config->z3_init);
}

/**
 * @brief 重置二阶 LESO 状态
 *
 * 只重置观测状态 z1/z2/z3，不改变 b0、wo 和 beta。
 * 适合模式切换、落地/离地状态切换、控制器重新启用后使用。
 *
 * @param leso 二阶 LESO 实例
 * @param z1   输出 y 的估计初值
 * @param z2   输出速度 y_dot 的估计初值
 * @param z3   总扰动估计初值
 */
void LESO2Reset(LESO2Instance *leso, float z1, float z2, float z3)
{
    if (leso == 0) return;

    leso->z1 = z1;
    leso->z2 = z2;
    leso->z3 = z3;
}

/**
 * @brief 更新二阶 LESO 状态
 *
 * 对象模型：y_ddot = f + b0 * u。
 * z1 估计 y，z2 估计 y_dot，z3 估计总扰动 f。函数内部采用显式欧拉离散，
 * 因此调用方应传入稳定、真实的控制周期 dt。
 *
 * @param leso 二阶 LESO 实例
 * @param y    当前测量输出
 * @param u    实际作用到对象上的控制输入
 * @param dt   控制周期，单位 s
 */
void LESO2Update(LESO2Instance *leso, float y, float u, float dt)
{
    if (leso == 0 || dt <= 0.0f) return;

    // 二阶对象模型：y_ddot = f + b0 * u，z3 用来估计总扰动 f。
    const float e = leso->z1 - y;
    const float z1_dot = leso->z2 - leso->beta1 * e;
    const float z2_dot = leso->z3 + leso->b0 * u - leso->beta2 * e;
    const float z3_dot = -leso->beta3 * e;

    // 当前实现采用显式欧拉离散，便于在 500Hz/1kHz 控制循环里直接调用。
    leso->z1 += z1_dot * dt;
    leso->z2 += z2_dot * dt;
    leso->z3 += z3_dot * dt;
}
