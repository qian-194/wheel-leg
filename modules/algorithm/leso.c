#include "leso.h"

// 显式欧拉离散的 wo*dt 经验稳定上界：
// 一阶 (s+wo)^2 配置稍宽松，二阶 (s+wo)^3 极点更靠近不稳定边界，取更保守值。
#define LESO_WO_DT_MAX_1ST 0.2f
#define LESO_WO_DT_MAX_2ND 0.1f

/**
 * @brief 刷新 wo*dt 稳定性自检结果
 *
 * 仅当 wo>0 且 dt>0 时，比较 wo*dt 与建议上界，置 stable_flag；
 * 否则视为不积分状态，stable_flag 清零。
 *
 * @param wo         观测器带宽
 * @param dt         离散步长
 * @param wo_dt_max  该阶数对应的 wo*dt 经验上界
 * @param wo_dt_out  输出 wo*dt
 * @return uint8_t   stable_flag
 */
static uint8_t LESOUpdateStability(float wo, float dt, float wo_dt_max, float *wo_dt_out)
{
    if (wo <= 0.0f || dt <= 0.0f)
    {
        *wo_dt_out = 0.0f;
        return 0u;
    }

    *wo_dt_out = wo * dt;
    return (*wo_dt_out <= wo_dt_max) ? 1u : 0u;
}

/**
 * @brief 设置一阶 LESO 带宽
 *
 * 一阶 LESO 的特征多项式按 (s + wo)^2 配置：
 * beta1 = 2 * wo, beta2 = wo^2。同步刷新稳定性自检。
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

    leso->stable_flag = LESOUpdateStability(leso->wo, leso->dt, LESO_WO_DT_MAX_1ST, &leso->wo_dt);
}

/**
 * @brief 设置一阶 LESO 控制频率
 *
 * 按 dt = 1/freq 刷新内部步长，并同步稳定性自检。
 * freq <= 0 时 dt 置 0，更新时不积分。
 *
 * @param leso 一阶 LESO 实例
 * @param freq 控制计算频率，Hz
 */
void LESO1SetFreq(LESO1Instance *leso, float freq)
{
    if (leso == 0) return;

    leso->dt = (freq > 0.0f) ? (1.0f / freq) : 0.0f;
    leso->stable_flag = LESOUpdateStability(leso->wo, leso->dt, LESO_WO_DT_MAX_1ST, &leso->wo_dt);
}

/**
 * @brief 初始化一阶 LESO
 *
 * 初始化会写入控制增益估计值 b0，按 freq 派生步长 dt，按 wo 计算 beta 并做
 * 稳定性自检，最后设置 z1/z2 初值。常见用法是 z1_init = 当前测量 y，z2_init = 0。
 *
 * @param leso   一阶 LESO 实例
 * @param config 初始化参数
 */
void LESO1Init(LESO1Instance *leso, const LESO1_Init_Config_s *config)
{
    if (leso == 0 || config == 0) return;

    leso->b0 = config->b0;
    // 先派生 dt，再设带宽，保证 SetBandwidth 中的稳定性自检拿到正确的 dt。
    LESO1SetFreq(leso, config->freq);
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
 * z1 估计 y，z2 估计总扰动 f。函数内部采用显式欧拉离散，步长 dt 由初始化时的
 * freq 派生（dt = 1/freq），调用方无需再传入控制周期。
 *
 * @param leso 一阶 LESO 实例
 * @param y    当前测量输出
 * @param u    实际作用到对象上的控制输入
 */
void LESO1Update(LESO1Instance *leso, float y, float u)
{
    // dt 在初始化时由 freq 派生；freq 非法时 dt<=0，此处不积分。
    if (leso == 0 || leso->dt <= 0.0f) return;

    // 观测误差使用 z1 - y，反馈项写成 -beta * e。
    const float e = leso->z1 - y;

    // 一阶对象模型：y_dot = f + b0 * u，其中 z2 用来估计总扰动 f。
    const float z1_dot = leso->z2 + leso->b0 * u - leso->beta1 * e;
    const float z2_dot = -leso->beta2 * e;

    // 使用内部固定步长 dt 做显式欧拉积分。
    leso->z1 += z1_dot * leso->dt;
    leso->z2 += z2_dot * leso->dt;
}

/**
 * @brief 设置二阶 LESO 带宽
 *
 * 二阶 LESO 的特征多项式按 (s + wo)^3 配置：
 * beta1 = 3 * wo, beta2 = 3 * wo^2, beta3 = wo^3。同步刷新稳定性自检。
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

    leso->stable_flag = LESOUpdateStability(leso->wo, leso->dt, LESO_WO_DT_MAX_2ND, &leso->wo_dt);
}

/**
 * @brief 设置二阶 LESO 控制频率
 *
 * 按 dt = 1/freq 刷新内部步长，并同步稳定性自检。
 * freq <= 0 时 dt 置 0，更新时不积分。
 *
 * @param leso 二阶 LESO 实例
 * @param freq 控制计算频率，Hz
 */
void LESO2SetFreq(LESO2Instance *leso, float freq)
{
    if (leso == 0) return;

    leso->dt = (freq > 0.0f) ? (1.0f / freq) : 0.0f;
    leso->stable_flag = LESOUpdateStability(leso->wo, leso->dt, LESO_WO_DT_MAX_2ND, &leso->wo_dt);
}

/**
 * @brief 初始化二阶 LESO
 *
 * 初始化会写入控制增益估计值 b0，按 freq 派生步长 dt，按 wo 计算 beta 并做
 * 稳定性自检，最后设置 z1/z2/z3 初值。常见用法是 z1_init = 当前测量 y，
 * z2_init = 当前速度估计或 0，z3_init = 0。
 *
 * @param leso   二阶 LESO 实例
 * @param config 初始化参数
 */
void LESO2Init(LESO2Instance *leso, const LESO2_Init_Config_s *config)
{
    if (leso == 0 || config == 0) return;

    leso->b0 = config->b0;
    // 先派生 dt，再设带宽，保证 SetBandwidth 中的稳定性自检拿到正确的 dt。
    LESO2SetFreq(leso, config->freq);
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
 * z1 估计 y，z2 估计 y_dot，z3 估计总扰动 f。函数内部采用显式欧拉离散，步长 dt
 * 由初始化时的 freq 派生（dt = 1/freq），调用方无需再传入控制周期。
 *
 * @param leso 二阶 LESO 实例
 * @param y    当前测量输出
 * @param u    实际作用到对象上的控制输入
 */
void LESO2Update(LESO2Instance *leso, float y, float u)
{
    // dt 在初始化时由 freq 派生；freq 非法时 dt<=0，此处不积分。
    if (leso == 0 || leso->dt <= 0.0f) return;

    // 二阶对象模型：y_ddot = f + b0 * u，z3 用来估计总扰动 f。
    const float e = leso->z1 - y;
    const float z1_dot = leso->z2 - leso->beta1 * e;
    const float z2_dot = leso->z3 + leso->b0 * u - leso->beta2 * e;
    const float z3_dot = -leso->beta3 * e;

    // 使用内部固定步长 dt 做显式欧拉积分（500Hz/1kHz 控制循环可直接调用）。
    leso->z1 += z1_dot * leso->dt;
    leso->z2 += z2_dot * leso->dt;
    leso->z3 += z3_dot * leso->dt;
}
