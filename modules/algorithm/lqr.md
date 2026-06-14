# LQR 通用控制器使用说明

`lqr.h / lqr.c` 提供一个可复用的 LQR 计算模块。模块本身只负责：

- 绑定应用层提供的静态工作区
- 导入当前反馈矩阵 `K`
- 根据注册模式计算控制输出
- 可选前馈和输出限幅

模块不负责：

- 求解 LQR 矩阵
- 腿长插值
- 状态量组装
- 物理符号修正
- 电机输出映射

这些应放在 app 层完成。

---

## 1. 矩阵尺寸约定

注册时通过：

```c
.StateSize = n,
.ControlSize = m,
```

确定 LQR 尺寸。

反馈矩阵 `K` 的形状固定为：

```c
float K[m][n];
```

即：

```text
K[ControlSize][StateSize]
```

行主序存储。

例如 WBR 整车 LQR：

```c
StateSize = 10;
ControlSize = 4;
float K[4][10];
```

状态向量：

```c
float x[10];
```

输出向量：

```c
float u[4];
```

---

## 2. 计算模式

初始化时通过 `Mode` 选择计算方式。

### 2.1 调节模式

```c
.Mode = LQR_MODE_REGULATION
```

计算：

```text
u = K * x
```

适合 app 层已经把状态组装成误差量的场景。

例如：

```c
x[0] = chassis->dist - chassis->target_dist;
x[1] = chassis->vel - chassis->target_v;
x[2] = yaw_error;
x[3] = chassis->wz - chassis->target_wz;
```

### 2.2 参考跟踪模式

```c
.Mode = LQR_MODE_TRACK_REFERENCE
```

计算：

```text
u = K * (x - ref)
```

适合模块内部统一处理参考状态的场景。

### 2.3 符号约定与 `LQR_NEGATE_GAIN`

模块默认**不对 `K` 取负**，即直接计算 `u = Kx` 或 `u = K(x - ref)`，因此导入的 `K` 必须已经包含你希望的控制符号。

若希望使用标准负反馈 `u = -Kx`（即导入未取负的 `K`，由模块内部加负号），在初始化时开启特性位：

```c
.Features = LQR_NEGATE_GAIN,
```

此时计算结果为：

```text
u = -K * x            // 调节模式
u = -K * (x - ref)    // 参考跟踪模式
```

取负发生在前馈累加之前，因此前馈不会被取负：

```text
u = -K * x + ff
```

两种符号约定可与任意模式自由组合，按位或叠加其他特性，例如：

```c
.Features = LQR_NEGATE_GAIN | LQR_USE_OUTPUT_LIMIT,
```

---

## 3. app 层静态工作区

本模块**不进行任何动态内存分配**。所有缓冲必须由 app 层提供静态工作区并通过 `Workspace` 注入；`LQRInit` 在 `Workspace` 为空、或工作区中任一缓冲指针为空时返回 `LQR_ERR_NULL`。

以 `10状态 / 4输入` 为例：

```c
#include "lqr.h"

static LQRInstance wbr_lqr;

static float wbr_lqr_state[10];
static float wbr_lqr_reference[10];
static float wbr_lqr_state_error[10];
static float wbr_lqr_control[4];
static float wbr_lqr_feedforward[4];
static float wbr_lqr_gain[4 * 10];

static LQR_Workspace_s wbr_lqr_workspace = {
    .State = wbr_lqr_state,
    .Reference = wbr_lqr_reference,
    .StateError = wbr_lqr_state_error,
    .Control = wbr_lqr_control,
    .Feedforward = wbr_lqr_feedforward,
    .GainMatrix = wbr_lqr_gain,
};
```

数组尺寸必须满足：

```text
State       >= StateSize
Reference   >= StateSize
StateError  >= StateSize
Control     >= ControlSize
Feedforward >= ControlSize
GainMatrix  >= StateSize * ControlSize
```

---

## 4. 初始化 LQR

### 4.1 固定 K 初始化

```c
static const float output_min[4] = {
    -2.5f, -2.5f, -12.0f, -12.0f
};

static const float output_max[4] = {
     2.5f,  2.5f,  12.0f,  12.0f
};

static const float initial_k[4][10] = {
    {0.0f},
    {0.0f},
    {0.0f},
    {0.0f},
};

void WbrLqrInit(void)
{
    LQR_Init_Config_s config = {
        .StateSize = 10,
        .ControlSize = 4,
        .Mode = LQR_MODE_REGULATION,
        .Features = LQR_USE_OUTPUT_LIMIT,
        .InitialGain = (const float *)initial_k,
        .InitialReference = NULL,
        .InitialFeedforward = NULL,
        .OutputMin = output_min,
        .OutputMax = output_max,
        .Workspace = &wbr_lqr_workspace,
        .PreProcess = NULL,
        .PostProcess = NULL,
    };

    LQRInit(&wbr_lqr, &config);
}
```

### 4.2 初始化时不导入 K

如果 K 每周期由 app 层插值得到，可以初始化时不传 `InitialGain`：

```c
.InitialGain = NULL,
```

但第一次 `LQRCalculate()` 前必须调用：

```c
LQRSetGain(&wbr_lqr, (const float *)k);
```

否则 K 为全 0，输出也会为 0。

---

## 5. 每周期计算

典型调用顺序：

```c
void WbrLqrUpdate(void)
{
    float k[4][10];
    float x[10];

    WbrLqrInterpolateK(left_leg_len, right_leg_len, k);

    LQRSetGain(&wbr_lqr, (const float *)k);

    x[0] = chassis.dist - chassis.target_dist;
    x[1] = chassis.vel - chassis.target_v;
    x[2] = yaw_error;
    x[3] = chassis.wz - chassis.target_wz;
    x[4] = left.theta;
    x[5] = left.theta_w;
    x[6] = right.theta;
    x[7] = right.theta_w;
    x[8] = chassis.pitch;
    x[9] = chassis.pitch_w;

    float *u = LQRCalculate(&wbr_lqr, x);

    left.T_wheel = u[0];
    right.T_wheel = u[1];
    left.T_hip = u[2];
    right.T_hip = u[3];
}
```

---

## 6. 使用参考跟踪模式

如果希望模块内部计算 `x - ref`，初始化时：

```c
.Mode = LQR_MODE_TRACK_REFERENCE
```

每周期设置参考：

```c
float ref[10] = {0};

ref[0] = target_dist;
ref[1] = target_v;
ref[2] = target_yaw;
ref[3] = target_wz;

LQRSetReference(&wbr_lqr, ref);

float *u = LQRCalculate(&wbr_lqr, x);
```

此时模块计算：

```text
u = K * (x - ref)
```

---

## 7. 前馈使用

初始化时开启：

```c
.Features = LQR_USE_FEEDFORWARD | LQR_USE_OUTPUT_LIMIT
```

每周期设置前馈：

```c
float ff[4] = {
    wheel_ff_l,
    wheel_ff_r,
    hip_ff_l,
    hip_ff_r,
};

LQRSetFeedforward(&wbr_lqr, ff);
```

最终输出：

```text
u = K * x + ff
```

或：

```text
u = K * (x - ref) + ff
```

取决于 `Mode`。

---

## 8. 输出限幅

初始化时开启：

```c
.Features = LQR_USE_OUTPUT_LIMIT
```

并传入：

```c
.OutputMin = output_min,
.OutputMax = output_max,
```

例如：

```c
static const float output_min[4] = {
    -2.5f, -2.5f, -12.0f, -12.0f
};

static const float output_max[4] = {
     2.5f,  2.5f,  12.0f,  12.0f
};
```

限幅作用在最终输出上。

---

## 9. 如何获得反馈矩阵 K

LQR 模块不负责求解 K。K 应由离线工具或 app 层算法提供。

常见来源有三种。

### 9.1 固定工作点离线求解

适合单一模型或固定腿长。

Python / MATLAB 根据状态空间模型：

```text
x[k+1] = A x[k] + B u[k]
```

选择权重：

```text
Q: 状态误差权重
R: 控制输入权重
```

求解离散 LQR：

```text
K = dlqr(A, B, Q, R)
```

如果离线工具输出的 K 已经包含负反馈符号，固件端直接导入即可。

### 9.2 多工作点表格

适合腿长变化、负载变化等场景。

离线生成多个 K：

```c
K_TABLE[left_leg_index][right_leg_index][u][x]
```

app 层运行时插值：

```c
float k[4][10];

WbrLqrInterpolateK(left_leg_len, right_leg_len, k);
LQRSetGain(&wbr_lqr, (const float *)k);
```

LQR 模块只接收插值后的当前 K，不关心表格来源。

### 9.3 手动调试矩阵

调试早期可以先用简单 K：

```c
static const float k_test[1][2] = {
    {-15.0f, -3.0f}
};
```

确认模块计算、限幅、输出方向正确后，再接入真实离线 K。

---

## 10. 注意事项

1. `K` 的尺寸必须是 `ControlSize x StateSize`。
2. 同一个 `LQRInstance` 初始化后，尺寸固定，只能更新同尺寸 K。
3. 如果需要从 `4x10` 换成 `2x6`，应重新初始化或新建实例。
4. 默认不对 K 自动取负号，导入的 K 必须含正确符号；若需标准负反馈 `u = -Kx`，开启 `LQR_NEGATE_GAIN` 特性。
5. 模块不进行动态分配，必须由 app 层提供静态 workspace；缓冲缺失时 `LQRInit` 返回 `LQR_ERR_NULL`。
6. WBR 这种业务相关的腿长插值、状态组装、离地保护、符号宏，不应放入通用 LQR 模块。
7. `LQRReset()` 会清空状态、参考、误差、输出、前馈和计数器，但不应依赖它重新配置尺寸。
