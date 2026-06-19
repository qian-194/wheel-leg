# LESO 和 ADRC 调用说明

本文档说明 `modules/algorithm/leso.h` 与 `modules/algorithm/adrc.h` 的典型调用方式。

## 选择一阶还是二阶

- 一阶 LESO / LADRC1：对象近似为 `y_dot = f + b0 * u`，例如速度环、温度一类一阶响应。
- 二阶 LESO / LADRC2：对象近似为 `y_ddot = f + b0 * u`，例如位置、角度、腿长这类需要位置和速度反馈的对象。

`b0` 是输入增益估计值，不要求完全精确，但符号必须正确，量级要接近真实对象。`wo` 是观测器带宽，越大响应越快，也越容易放大测量噪声。

## 一阶 LESO 独立调用

```c
#include "leso.h"

static LESO1Instance vel_eso;

void VelEsoInit(float current_vel)
{
    LESO1_Init_Config_s config = {
        .b0 = 1.0f,
        .wo = 30.0f,
        .z1_init = current_vel,
        .z2_init = 0.0f,
    };

    LESO1Init(&vel_eso, &config);
}

void VelEsoUpdate(float measured_vel, float applied_u, float dt)
{
    LESO1Update(&vel_eso, measured_vel, applied_u, dt);

    /* vel_eso.z1: 速度估计
     * vel_eso.z2: 速度环总扰动估计
     */
}
```

## 二阶 LESO 独立调用

```c
#include "leso.h"

static LESO2Instance angle_eso;

void AngleEsoInit(float current_angle)
{
    LESO2_Init_Config_s config = {
        .b0 = 1.0f,
        .wo = 40.0f,
        .z1_init = current_angle,
        .z2_init = 0.0f,
        .z3_init = 0.0f,
    };

    LESO2Init(&angle_eso, &config);
}

void AngleEsoUpdate(float measured_angle, float applied_torque, float dt)
{
    LESO2Update(&angle_eso, measured_angle, applied_torque, dt);

    /* angle_eso.z1: 角度估计
     * angle_eso.z2: 角速度估计
     * angle_eso.z3: 总扰动估计
     */
}
```

## 一阶 LADRC 调用

```c
#include "adrc.h"

static LADRC1Instance vel_adrc;
static float vel_applied_u;

void VelAdrcInit(float current_vel)
{
    LADRC1_Init_Config_s config = {
        .b0 = 1.0f,
        .wo = 30.0f,
        .kp = 20.0f,
        .max_out = 100.0f,
        .z1_init = current_vel,
        .z2_init = 0.0f,
    };

    LADRC1Init(&vel_adrc, &config);
    vel_applied_u = 0.0f;
}

float VelAdrcUpdate(float target_vel, float measured_vel, float dt)
{
    float u = LADRC1UpdateWithInput(&vel_adrc,
                                    target_vel,
                                    measured_vel,
                                    vel_applied_u,
                                    dt);

    /* 如果后面还有电机限幅或安全裁剪，vel_applied_u 应记录真实执行值。 */
    vel_applied_u = u;
    return u;
}
```

## 二阶 LADRC 调用

```c
#include "adrc.h"

static LADRC2Instance leg_len_adrc;
static float leg_len_applied_force;

void LegLenAdrcInit(float current_len)
{
    LADRC2_Init_Config_s config = {
        .b0 = 1.0f,
        .wo = 35.0f,
        .kp = 400.0f,
        .kd = 30.0f,
        .max_out = 200.0f,
        .z1_init = current_len,
        .z2_init = 0.0f,
        .z3_init = 0.0f,
    };

    LADRC2Init(&leg_len_adrc, &config);
    leg_len_applied_force = 0.0f;
}

float LegLenAdrcUpdate(float target_len, float measured_len, float dt)
{
    const float target_len_dot = 0.0f;
    float force = LADRC2UpdateWithInput(&leg_len_adrc,
                                        target_len,
                                        target_len_dot,
                                        measured_len,
                                        leg_len_applied_force,
                                        dt);

    /* 如果 force 后续被限幅，下一周期传入的 leg_len_applied_force 应为限幅后的值。 */
    leg_len_applied_force = force;
    return force;
}
```

## 在模式切换时重置

控制器从失能切到使能、机器人落地、腿长目标突变、传感器重新校准时，建议重置观测器状态：

```c
LADRC1Reset(&vel_adrc, current_vel, 0.0f);
LADRC2Reset(&leg_len_adrc, current_len, current_len_dot, 0.0f);
```

## 调参建议

1. 先确认 `b0` 符号正确。符号错会直接正反馈。
2. 先小带宽 `wo` 起步，观察 `z1` 是否跟随测量值，再逐步增大。
3. LADRC1 先调 `kp`，LADRC2 先调 `kp/kd`，再回头提高 `wo`。
4. `max_out` 建议始终保留，特别是接入电机、腿长力或轮端力矩时。
5. 若控制输出经过限幅或安全裁剪，优先使用 `UpdateWithInput` 并传入真实执行值。
