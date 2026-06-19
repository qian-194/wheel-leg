#include "adrc.h"

#include <math.h>

#define ADRC_FLOAT_EPS 1.0e-6f

/**
 * @brief 控制输出绝对值限幅
 *
 * max_abs <= 0 时保持原值，便于调试阶段临时关闭输出限幅。
 *
 * @param value   待限幅数值
 * @param max_abs 最大绝对值
 * @return float  限幅后的数值
 */
static float ADRCClamp(float value, float max_abs)
{
    if (max_abs <= 0.0f) return value;
    if (value > max_abs) return max_abs;
    if (value < -max_abs) return -max_abs;
    return value;
}

/**
 * @brief 初始化一阶线性 ADRC
 *
 * 初始化内部的一阶 LESO，并写入一阶误差反馈增益 kp、控制增益估计 b0
 * 和输出限幅 max_out。
 *
 * @param adrc   一阶 ADRC 实例
 * @param config 初始化参数
 */
void LADRC1Init(LADRC1Instance *adrc, const LADRC1_Init_Config_s *config)
{
    if (adrc == 0 || config == 0) return;

    LESO1_Init_Config_s eso_config = {
        .b0 = config->b0,
        .wo = config->wo,
        .z1_init = config->z1_init,
        .z2_init = config->z2_init,
    };

    LESO1Init(&adrc->eso, &eso_config);
    adrc->b0 = config->b0;
    adrc->kp = config->kp;
    adrc->max_out = config->max_out;
    adrc->err = 0.0f;
    adrc->u0 = 0.0f;
    adrc->output = 0.0f;
}

/**
 * @brief 重置一阶线性 ADRC 状态
 *
 * 重置 LESO 的 z1/z2，同时清空 err、u0 和 output。参数 b0、kp、max_out 不变。
 *
 * @param adrc 一阶 ADRC 实例
 * @param z1   LESO 输出估计初值，通常设为当前测量值
 * @param z2   LESO 总扰动估计初值，通常设为 0
 */
void LADRC1Reset(LADRC1Instance *adrc, float z1, float z2)
{
    if (adrc == 0) return;

    LESO1Reset(&adrc->eso, z1, z2);
    adrc->err = 0.0f;
    adrc->u0 = 0.0f;
    adrc->output = 0.0f;
}

/**
 * @brief 更新一阶线性 ADRC
 *
 * 简化接口，默认上一次 output 就是实际作用到对象的输入。
 * 如果后级有电机限幅、斜坡限制、安全裁剪，优先使用 LADRC1UpdateWithInput。
 *
 * @param adrc 一阶 ADRC 实例
 * @param ref  目标输出
 * @param y    当前测量输出
 * @param dt   控制周期，单位 s
 * @return float 控制输出
 */
float LADRC1Update(LADRC1Instance *adrc, float ref, float y, float dt)
{
    if (adrc == 0) return 0.0f;

    // 简化调用：假设上一次输出就是实际作用到对象的输入。
    return LADRC1UpdateWithInput(adrc, ref, y, adrc->output, dt);
}

/**
 * @brief 使用真实执行输入更新一阶线性 ADRC
 *
 * 控制律：u = (kp * (ref - z1) - z2) / b0。
 * 先用 applied_u 更新 LESO，再用 z1 计算误差，用 z2 做扰动补偿。
 *
 * @param adrc      一阶 ADRC 实例
 * @param ref       目标输出
 * @param y         当前测量输出
 * @param applied_u 上一周期实际作用到对象上的控制输入
 * @param dt        控制周期，单位 s
 * @return float    控制输出
 */
float LADRC1UpdateWithInput(LADRC1Instance *adrc, float ref, float y, float applied_u, float dt)
{
    if (adrc == 0) return 0.0f;

    // 先用真实输入和当前测量值更新 ESO，再基于估计状态计算控制量。
    LESO1Update(&adrc->eso, y, applied_u, dt);
    adrc->err = ref - adrc->eso.z1;
    adrc->u0 = adrc->kp * adrc->err;

    // z2 是总扰动估计，在线性 ADRC 中通过 -z2/b0 前馈抵消。
    if (fabsf(adrc->b0) > ADRC_FLOAT_EPS)
    {
        adrc->output = (adrc->u0 - adrc->eso.z2) / adrc->b0;
    }
    else
    {
        adrc->output = 0.0f;
    }

    adrc->output = ADRCClamp(adrc->output, adrc->max_out);
    return adrc->output;
}

/**
 * @brief 初始化二阶线性 ADRC
 *
 * 初始化内部的二阶 LESO，并写入 kp/kd、控制增益估计 b0 和输出限幅 max_out。
 *
 * @param adrc   二阶 ADRC 实例
 * @param config 初始化参数
 */
void LADRC2Init(LADRC2Instance *adrc, const LADRC2_Init_Config_s *config)
{
    if (adrc == 0 || config == 0) return;

    LESO2_Init_Config_s eso_config = {
        .b0 = config->b0,
        .wo = config->wo,
        .z1_init = config->z1_init,
        .z2_init = config->z2_init,
        .z3_init = config->z3_init,
    };

    LESO2Init(&adrc->eso, &eso_config);
    adrc->b0 = config->b0;
    adrc->kp = config->kp;
    adrc->kd = config->kd;
    adrc->max_out = config->max_out;
    adrc->err = 0.0f;
    adrc->derr = 0.0f;
    adrc->u0 = 0.0f;
    adrc->output = 0.0f;
}

/**
 * @brief 重置二阶线性 ADRC 状态
 *
 * 重置 LESO 的 z1/z2/z3，同时清空 err、derr、u0 和 output。
 * 参数 b0、kp、kd、max_out 不变。
 *
 * @param adrc 二阶 ADRC 实例
 * @param z1   LESO 输出估计初值，通常设为当前测量值
 * @param z2   LESO 速度估计初值，通常设为当前速度估计或 0
 * @param z3   LESO 总扰动估计初值，通常设为 0
 */
void LADRC2Reset(LADRC2Instance *adrc, float z1, float z2, float z3)
{
    if (adrc == 0) return;

    LESO2Reset(&adrc->eso, z1, z2, z3);
    adrc->err = 0.0f;
    adrc->derr = 0.0f;
    adrc->u0 = 0.0f;
    adrc->output = 0.0f;
}

/**
 * @brief 更新二阶线性 ADRC
 *
 * 简化接口，默认上一次 output 就是实际作用到对象的输入。
 * 如果后级有电机限幅、斜坡限制、安全裁剪，优先使用 LADRC2UpdateWithInput。
 *
 * @param adrc    二阶 ADRC 实例
 * @param ref     目标输出
 * @param ref_dot 目标输出速度，未知时可传 0
 * @param y       当前测量输出
 * @param dt      控制周期，单位 s
 * @return float  控制输出
 */
float LADRC2Update(LADRC2Instance *adrc, float ref, float ref_dot, float y, float dt)
{
    if (adrc == 0) return 0.0f;

    // 简化调用：假设上一次输出就是实际作用到对象的输入。
    return LADRC2UpdateWithInput(adrc, ref, ref_dot, y, adrc->output, dt);
}

/**
 * @brief 使用真实执行输入更新二阶线性 ADRC
 *
 * 控制律：u = (kp * (ref - z1) + kd * (ref_dot - z2) - z3) / b0。
 * 先用 applied_u 更新 LESO，再用 z1/z2 计算位置和速度误差，用 z3 做扰动补偿。
 *
 * @param adrc      二阶 ADRC 实例
 * @param ref       目标输出
 * @param ref_dot   目标输出速度，未知时可传 0
 * @param y         当前测量输出
 * @param applied_u 上一周期实际作用到对象上的控制输入
 * @param dt        控制周期，单位 s
 * @return float    控制输出
 */
float LADRC2UpdateWithInput(LADRC2Instance *adrc, float ref, float ref_dot, float y, float applied_u, float dt)
{
    if (adrc == 0) return 0.0f;

    // 二阶 ADRC 使用 ESO 的 z1/z2 作为位置和速度反馈，避免直接差分测量噪声。
    LESO2Update(&adrc->eso, y, applied_u, dt);
    adrc->err = ref - adrc->eso.z1;
    adrc->derr = ref_dot - adrc->eso.z2;
    adrc->u0 = adrc->kp * adrc->err + adrc->kd * adrc->derr;

    // z3 是总扰动估计，对其进行补偿后再除以 b0 得到实际输入。
    if (fabsf(adrc->b0) > ADRC_FLOAT_EPS)
    {
        adrc->output = (adrc->u0 - adrc->eso.z3) / adrc->b0;
    }
    else
    {
        adrc->output = 0.0f;
    }

    adrc->output = ADRCClamp(adrc->output, adrc->max_out);
    return adrc->output;
}
