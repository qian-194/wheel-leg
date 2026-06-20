# LESO 和 ADRC 调用说明

本文档说明 `modules/algorithm/leso.h` 与 `modules/algorithm/adrc.h` 的典型调用方式。

## 选择一阶还是二阶

- 一阶 LESO / LADRC1：对象近似为 `y_dot = f + b0 * u`，例如速度环、温度一类一阶响应。
- 二阶 LESO / LADRC2：对象近似为 `y_ddot = f + b0 * u`，例如位置、角度、腿长这类需要位置和速度反馈的对象。

`b0` 是输入增益估计值，不要求完全精确，但符号必须正确，量级要接近真实对象。`wo` 是观测器带宽，越大响应越快，也越容易放大测量噪声。

## 关于控制频率 freq

初始化时通过 `freq`（单位 Hz）传入控制计算频率，模块内部据此固定离散步长 `dt = 1/freq`，
之后所有 `Update` 调用都不再需要传 `dt`。这样既能让每个观测器/控制器适配自己所在回路的频率，
也避免实测周期抖动（或标定早退后的一次大步长冲击）破坏显式欧拉离散的稳定性。

`freq <= 0` 视为非法：内部 `dt` 置 0，`Update` 直接跳过积分，并把实例里的 `stable_flag` 置 0。
若回路频率会在运行时改变，可调用 `LESO1SetFreq` / `LESO2SetFreq` 同步刷新步长。

## 一阶 LESO 独立调用

```c
#include "leso.h"

static LESO1Instance vel_eso;

void VelEsoInit(float current_vel)
{
    LESO1_Init_Config_s config = {
        .b0 = 1.0f,
        .wo = 30.0f,
        .freq = 500.0f,
        .z1_init = current_vel,
        .z2_init = 0.0f,
    };

    LESO1Init(&vel_eso, &config);
}

void VelEsoUpdate(float measured_vel, float applied_u)
{
    LESO1Update(&vel_eso, measured_vel, applied_u);

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
        .freq = 500.0f,
        .z1_init = current_angle,
        .z2_init = 0.0f,
        .z3_init = 0.0f,
    };

    LESO2Init(&angle_eso, &config);
}

void AngleEsoUpdate(float measured_angle, float applied_torque)
{
    LESO2Update(&angle_eso, measured_angle, applied_torque);

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
        .freq = 500.0f,
        .kp = 20.0f,
        .max_out = 100.0f,
        .z1_init = current_vel,
        .z2_init = 0.0f,
    };

    LADRC1Init(&vel_adrc, &config);
    vel_applied_u = 0.0f;
}

float VelAdrcUpdate(float target_vel, float measured_vel)
{
    float u = LADRC1UpdateWithInput(&vel_adrc,
                                    target_vel,
                                    measured_vel,
                                    vel_applied_u);

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
        .freq = 500.0f,
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

float LegLenAdrcUpdate(float target_len, float measured_len)
{
    const float target_len_dot = 0.0f;
    float force = LADRC2UpdateWithInput(&leg_len_adrc,
                                        target_len,
                                        target_len_dot,
                                        measured_len,
                                        leg_len_applied_force);

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
6. `wo` 与频率有关。模块采用显式欧拉离散，`wo * dt = wo / freq` 不宜过大：
   一阶建议 `< ~0.2`，二阶更保守 `< ~0.1`，否则可能数值发散。初始化后可读实例里的
   `stable_flag`：为 0 即表示 `wo*dt` 越界，应降低 `wo` 或提高 `freq`。
   例如 `wo=35, freq=500 → wo*dt=0.07`，二阶上界内，`stable_flag=1`。
