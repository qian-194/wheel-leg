# MC02 轮腿平衡控制迁移计划：老代码架构 + SJTU 整车 LQR 建模

本文档只作为施工图，目标工程代码位于：

`C:\Users\qianc\Desktop\wheel-leg-mc02`

参考老车工程位于：

`C:\Users\qianc\Desktop\26leg\RM_YunQian_WheelLeg`

离线仿真和 LQR 表生成目录位于：

`C:\Users\qianc\Desktop\balance_simulate`

当前建议路线：保留 MC02 原麦轮 `chassis.c`，新增独立 `balance` 应用。首版只做轮腿平衡闭环，把老车的五连杆、VMC、腿长 PID、速度估计架构移植过来，但把老车左右单腿 6 状态 LQR 替换为 SJTU 风格整车 10 状态、4 输入 WBR LQR。

---

## 1. 当前状态

MC02 当前工程：

`C:\Users\qianc\Desktop\wheel-leg-mc02`

已观察到的关键文件状态：

- `C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\balance.c`：空文件。
- `C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\balance.h`：空文件。
- `C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\lqr_calc.h`：空文件。
- `C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\wbr_lqr_calc.h`：已经存在，来源是离线脚本导出的 SJTU WBR LQR 表，目前在 git 中大概率是未跟踪文件。
- `C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\chassis.c`：原麦轮底盘控制，不建议删除。
- `C:\Users\qianc\Desktop\wheel-leg-mc02\application\robot.c`：当前 `CHASSIS_BOARD` 路径调用 `ChassisInit()` 和 `ChassisTask()`。
- `C:\Users\qianc\Desktop\wheel-leg-mc02\Makefile`：当前只加入了 `application/chassis/chassis.c`，还没有加入 `application/chassis/balance.c`。

老车参考工程：

`C:\Users\qianc\Desktop\26leg\RM_YunQian_WheelLeg`

建议重点参考这些文件：

- `C:\Users\qianc\Desktop\26leg\RM_YunQian_WheelLeg\application\chassis\balance.h`
- `C:\Users\qianc\Desktop\26leg\RM_YunQian_WheelLeg\application\chassis\balance.c`
- `C:\Users\qianc\Desktop\26leg\RM_YunQian_WheelLeg\application\chassis\linkNleg.h`
- `C:\Users\qianc\Desktop\26leg\RM_YunQian_WheelLeg\application\chassis\speed_estimation.h`
- `C:\Users\qianc\Desktop\26leg\RM_YunQian_WheelLeg\application\chassis\lqr_calc.h`
- `C:\Users\qianc\Desktop\26leg\RM_YunQian_WheelLeg\application\chassis\wbr_lqr_calc.h`

---

## 2. 总体目标

第一阶段不要追求完整比赛功能，只完成稳定、可调、可验证的平衡闭环。

控制主链路固定为：

```text
BalanceTask
  -> ControlSwitch
  -> ParamAssemble
  -> Link2Leg(left)
  -> Link2Leg(right)
  -> SlipKalman
  -> CalcWbrLQR
  -> LegControl
  -> VMCProject(left)
  -> VMCProject(right)
  -> LimitAndSetMotor
```

核心原则：

- 五连杆几何、腿长控制、VMC 映射沿用老车成熟架构。
- 平衡 LQR 从老车的“左右单腿各算一次”换成“一次整车 10 状态 LQR”。
- yaw、pitch、左右腿角耦合由整车 LQR 接管，旧 yaw PID、pitch 补偿先关闭，只保留诊断开关。
- 轮毂力矩首版限制 `±2.5 N·m`。
- 关节力矩首版限制 `±12 N·m`。
- 关节前后电机建议使用成对缩放限幅，不要简单单独裁剪。

---

## 3. 新旧 LQR 的差异

老车当前逻辑：

```text
CalcLQR(&l_side, &chassis);
CalcLQR(&r_side, &chassis);
```

老车 LQR 特点：

- 单腿模型。
- 每侧腿独立计算一次。
- 每次输出该侧 `T_wheel` 和 `T_hip`。
- K 值随单侧腿长用二次多项式拟合。
- yaw 和左右腿耦合更多依赖额外补偿项。

新 SJTU WBR LQR 逻辑：

```text
CalcWbrLQR(&l_side, &r_side, &chassis);
```

新 LQR 特点：

- 整车模型。
- 左右腿一起进入状态。
- 一次输出 4 个控制量。
- K 值由左右腿长二维表双线性插值。
- yaw、左右腿角、pitch、速度、位移在同一个状态空间内耦合处理。

状态顺序必须固定为：

```text
x = [
  s,
  s_dot,
  yaw,
  yaw_dot,
  left_theta,
  left_theta_dot,
  right_theta,
  right_theta_dot,
  pitch,
  pitch_dot
]
```

控制输出顺序必须固定为：

```text
u = [
  left_wheel_torque,
  right_wheel_torque,
  left_T_hip,
  right_T_hip
]
```

---

## 4. 目标文件规划

### 4.1 `balance.h`

目标路径：

`C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\balance.h`

用途：

- 定义轮腿平衡需要的机械参数宏。
- 定义 `LinkNPodParam`。
- 定义 `ChassisParam`。
- 声明 `BalanceInit()` 和 `BalanceTask()`。
- 给 `wbr_lqr_calc.h` 提供字段依赖。

建议从老车复制结构体基础，再按 MC02 当前工程做字段精简。

必须包含的 `LinkNPodParam` 字段：

```c
typedef struct
{
    /* 关节角度和角速度，单位 rad / rad/s */
    float phi1, phi2, phi3, phi4, phi0;
    float phi1_w, phi2_w, phi3_w, phi4_w, phi0_w;

    /* 轮毂状态 */
    float w_ecd;
    float wheel_dist;
    float wheel_w;
    float body_v;
    float T_wheel;

    /* 腿部状态 */
    float theta;
    float theta_w;
    float leg_len;
    float legd;
    float height;
    float height_v;
    float target_len;

    /* 腿长控制和虚拟髋力矩 */
    float F_leg;
    float T_hip;

    /* VMC 映射后的前后关节电机力矩 */
    float T_front;
    float T_back;

    /* 离地保护 */
    float normal_force;
    uint8_t fly_flag;

    /* 五连杆关键点坐标，调试用 */
    float coord[6];
} LinkNPodParam;
```

必须包含的 `ChassisParam` 字段：

```c
typedef struct
{
    /* 位移和速度，单位 m / m/s */
    float dist;
    float target_dist;
    float vel;
    float target_v;
    float vel_m;
    float vel_predict;
    float vel_cov;

    /* yaw，单位 rad / rad/s */
    float yaw;
    float target_yaw;
    float wz;
    float target_wz;
    float offset_angle;

    /* pitch/roll，单位 rad / rad/s */
    float pitch;
    float pitch_w;
    float roll;
    float roll_w;
    float target_roll;

    /* IMU 加速度和角加速度 */
    float MotionAccel_b[3];
    float dgyro[3];
    float accx;
    float accy;
    float accz;
    float acc_m;
    float acc_last;

    /* 状态和调试 */
    uint8_t cali_flag;
    float debug[10];
} ChassisParam;
```

关键宏建议保留：

```c
#define CALF_LEN          /* 小腿长度，单位 m */
#define THIGH_LEN         /* 大腿长度，单位 m */
#define JOINT_DISTANCE    /* 髋部前后关节距离，单位 m */

#define WHEEL_RADIUS      /* 轮半径，单位 m */
#define BODY_MASS         /* 机体质量，单位 kg */

#define L0_MIN            /* 最短腿长，单位 m */
#define L0_MAX            /* 最长腿长，单位 m */
#define L0_INIT           /* 上电默认腿长，单位 m */

#define VEL_PROCESS_NOISE
#define VEL_MEASURE_NOISE

#define JOINT_CNT 4u
#define LF 0u
#define LB 1u
#define RF 2u
#define RB 3u

#define DRIVEN_CNT 2u
#define LD 0u
#define RD 1u
```

注意：

- MC02 的 `attitude_t.Pitch/Roll/Yaw/YawTotalAngle` 是角度制，来自 `C:\Users\qianc\Desktop\wheel-leg-mc02\modules\imu\ins_task.c` 中乘以 `57.295779513f` 的欧拉角转换。
- WBR LQR、VMC、五连杆计算建议统一使用弧度。
- 所以在 `ParamAssemble()` 中必须把 IMU 姿态角从 degree 转为 rad。

---

### 4.2 `wbr_lqr_calc.h`

目标路径：

`C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\wbr_lqr_calc.h`

当前来源：

`C:\Users\qianc\Desktop\balance_simulate\并腿\wbr_lqr_calc.h`

这个文件已经包含：

- `WBR_LQR_LEG_GRID[10]`
- `WBR_LQR_K_TABLE[10][10][4][10]`
- `WbrLqrFindLegInterval()`
- `WbrLqrInterpolateK()`
- `CalcWbrLQR()`

当前 LQR 表腿长网格：

```c
static const float WBR_LQR_LEG_GRID[10] = {
    0.14f, 0.165555556f, 0.191111111f, 0.216666667f, 0.242222222f,
    0.267777778f, 0.293333333f, 0.318888889f, 0.344444444f, 0.37f
};
```

当前限幅：

```c
#define WBR_LQR_WHEEL_TORQUE_LIMIT 2.5f
#define WBR_LQR_VIRTUAL_HIP_TORQUE_LIMIT 12.0f
#define WBR_LQR_JOINT_TORQUE_LIMIT 12.0f
```

需要确认：

- `WBR_LQR_LEFT_WHEEL_SIGN`
- `WBR_LQR_RIGHT_WHEEL_SIGN`
- `WBR_LQR_LEFT_HIP_SIGN`
- `WBR_LQR_RIGHT_HIP_SIGN`

建议首版全部保持 `1.0f`，上电悬空验证符号后再改。

如果后续重新生成 K 表，来源应是：

`C:\Users\qianc\Desktop\balance_simulate\并腿\wbr_lqr_offline.py`

导出后覆盖：

`C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\wbr_lqr_calc.h`

---

### 4.3 `balance.c`

目标路径：

`C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\balance.c`

这是迁移的主文件。建议不要把所有老代码一次性完整塞进去，先按以下顺序写。

必须 include：

```c
#include "balance.h"
#include "robot_def.h"
#include "wbr_lqr_calc.h"
#include "linkNleg.h"        /* 如果拆成独立头文件 */
#include "speed_estimation.h"/* 如果拆成独立头文件 */
#include "HT04.h"
#include "LK9025.h"
#include "controller.h"
#include "user_lib.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "can_comm.h"
#include "message_center.h"
#include "ins_task.h"
#include "math.h"
```

如果首版不想新建 `linkNleg.h` 和 `speed_estimation.h`，也可以把 `Link2Leg()`、`VMCProject()`、`SlipKalman()` 作为 `static` 函数直接放在 `balance.c` 中。

推荐首版函数布局：

```c
static void ControlSwitch(void);
static void ParamAssemble(void);
static void Link2Leg(LinkNPodParam *p, ChassisParam *chassis);
static void SlipKalman(LinkNPodParam *lp, LinkNPodParam *rp, ChassisParam *cp, float dt);
static void SynthesizeMotion(void);
static void LegControl(void);
static void VMCProject(LinkNPodParam *p);
static void LimitJointPair(float *front, float *back, float limit);
static void LimitAndSetMotor(void);

void BalanceInit(void);
void BalanceTask(void);
```

---

## 5. `BalanceInit()` 规划

`BalanceInit()` 负责一次性初始化硬件、状态、PID 和通信。

建议流程：

```text
BalanceInit
  -> 初始化 IMU 指针
  -> 初始化 CANComm 或 PubSub
  -> 初始化 4 个 HT04 关节电机
  -> 初始化 2 个 LK9025 轮毂电机
  -> 初始化腿长 PID
  -> 初始化 roll 补偿 PID，可先低增益
  -> 初始化目标腿长 L0_INIT
  -> 初始化 target_yaw = 当前 yaw
  -> 初始化 vel_cov
```

MC02 当前 IMU 接口：

`C:\Users\qianc\Desktop\wheel-leg-mc02\modules\imu\ins_task.h`

已有：

```c
attitude_t *INS_Init(void);
```

`attitude_t` 只有：

```c
float Gyro[3];
float Accel[3];
float Roll;
float Pitch;
float Yaw;
float YawTotalAngle;
```

但是老车 `SlipKalman()` 需要：

```c
MotionAccel_b[3]
dgyro[3]
```

首版有两种路线：

路线 A：不改 IMU 模块，先用 `attitude_t.Accel` 近似填充 `MotionAccel_b`。

优点：

- 不改 `modules/imu`。
- 移植最快。

缺点：

- 加速度含重力和安装误差，速度估计会差。
- 只能用于低速站立验证，不适合快速运动。

路线 B：给 IMU 模块新增非破坏接口 `INS_GetData()` 返回 `INS_t *`。

目标文件：

`C:\Users\qianc\Desktop\wheel-leg-mc02\modules\imu\ins_task.h`

新增声明：

```c
INS_t *INS_GetData(void);
```

目标文件：

`C:\Users\qianc\Desktop\wheel-leg-mc02\modules\imu\ins_task.c`

新增实现：

```c
INS_t *INS_GetData(void)
{
    return &INS;
}
```

优点：

- 可以拿到 `MotionAccel_b`。
- 速度估计更接近老车逻辑。

缺点：

- 要修改 IMU 模块。

建议顺序：第一版如果只做站立，可以先用路线 A；如果要验证速度阶跃和打滑估计，尽快切到路线 B。

---

## 6. `ParamAssemble()` 规划

`ParamAssemble()` 是移植中最容易出符号问题的地方。它负责把电机反馈、IMU、遥控命令统一转换成模型坐标。

输入来源：

- HT04 四个关节电机角度和角速度。
- LK9025 左右轮速度。
- IMU 姿态角和角速度。
- `Chassis_Ctrl_Cmd_s` 控制命令。

输出目标：

- `l_side.phi1/phi4/phi1_w/phi4_w/w_ecd`
- `r_side.phi1/phi4/phi1_w/phi4_w/w_ecd`
- `chassis.pitch/pitch_w/roll/roll_w/yaw/wz`
- `chassis.target_v/target_wz/target_yaw/target_dist`
- `l_side.target_len/r_side.target_len`

角度单位约定：

- HT04 电机反馈：rad。
- LK9025 电机速度：rad/s。
- MC02 `attitude_t.Pitch/Roll/Yaw/YawTotalAngle`：degree。
- `ChassisParam` 内部：rad。

IMU 转换建议：

```c
chassis.pitch = imu->Pitch * DEGREE_2_RAD;
chassis.roll  = imu->Roll  * DEGREE_2_RAD;
chassis.yaw   = imu->YawTotalAngle * DEGREE_2_RAD;

chassis.pitch_w = imu->Gyro[X];
chassis.roll_w  = imu->Gyro[Y];
chassis.wz      = imu->Gyro[Z];
```

注意：`Gyro[X/Y/Z]` 的轴定义必须实测确认。这里只是首版假设。验证时要手动抬头、右倾、自旋，观察正负方向。

目标速度映射建议：

```c
chassis.target_v = chassis_cmd_recv.vx;
chassis.target_wz = chassis_cmd_recv.wz;
```

如果 MC02 原 `robot_cmd.c` 发出的 `vx/wz` 是麦轮速度单位，而不是 m/s 和 rad/s，需要在 `ParamAssemble()` 中加比例：

```c
chassis.target_v = chassis_cmd_recv.vx * BALANCE_CMD_VX_SCALE;
chassis.target_wz = chassis_cmd_recv.wz * BALANCE_CMD_WZ_SCALE;
```

腿长目标更新：

```c
target_len += chassis_cmd_recv.delta_leglen * dt;
VAL_LIMIT(target_len, L0_MIN, L0_MAX);
l_side.target_len = target_len;
r_side.target_len = target_len;
```

如果暂时不扩展 `Chassis_Ctrl_Cmd_s`，首版固定：

```c
l_side.target_len = L0_INIT;
r_side.target_len = L0_INIT;
```

---

## 7. `Link2Leg()` 规划

参考来源：

`C:\Users\qianc\Desktop\26leg\RM_YunQian_WheelLeg\application\chassis\linkNleg.h`

功能：

- 根据前后关节角度 `phi1/phi4` 计算五连杆中间点。
- 得到腿长 `leg_len`。
- 得到腿角 `theta`。
- 用短时预测差分得到 `theta_w` 和 `legd`。

核心输出：

```c
p->leg_len
p->theta
p->theta_w
p->legd
p->height
p->height_v
p->phi2
p->phi3
p->phi2_w
p->phi3_w
```

需要注意：

- 老代码里 `p->theta = p->phi0 - 0.5 * PI - chassis->pitch`。
- 这意味着 `theta` 已经扣除了机体 pitch，是腿相对世界竖直方向的角。
- SJTU WBR LQR 状态中的 `left_theta/right_theta` 要与离线脚本定义保持一致。
- 如果仿真里用的是“腿相对竖直方向角”，这里沿用老代码是合理的。

建议首版直接搬老车几何公式，先不改。

---

## 8. `SlipKalman()` 规划

参考来源：

`C:\Users\qianc\Desktop\26leg\RM_YunQian_WheelLeg\application\chassis\speed_estimation.h`

功能：

- 用轮速和腿部运动补偿得到机体速度测量 `vel_m`。
- 用 IMU 水平加速度预测速度。
- 用二阶 Kalman 同时估计速度和打滑偏置。
- 得到 `chassis.vel` 和 `chassis.dist`。

老车速度测量：

```c
lp->wheel_w = lp->w_ecd + lp->phi2_w - cp->pitch_w;
rp->wheel_w = rp->w_ecd + rp->phi2_w - cp->pitch_w;

lp->body_v = lp->wheel_w * WHEEL_RADIUS
           + lp->leg_len * lp->theta_w
           + lp->legd * msin(lp->theta);

rp->body_v = rp->wheel_w * WHEEL_RADIUS
           + rp->leg_len * rp->theta_w
           + rp->legd * msin(rp->theta);

cp->vel_m = (lp->body_v + rp->body_v) * 0.5f;
```

首版建议：

- 站立验证阶段可以弱化 IMU 加速度，只主要信轮速。
- 速度阶跃验证阶段再调 `Qv/Qb/R`。
- 如果 `target_v` 非零，则清零距离闭环。
- 如果 `target_v` 接近零，则积分 `dist` 并回零。

推荐逻辑：

```c
if (fabsf(cp->target_v) < 0.005f)
{
    cp->target_dist = 0.0f;
    cp->dist += cp->vel * delta_t;
}
else
{
    cp->target_dist = 0.0f;
    cp->dist = 0.0f;
}
```

---

## 9. `CalcWbrLQR()` 接入规划

目标函数已经在：

`C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\wbr_lqr_calc.h`

调用方式：

```c
CalcWbrLQR(&l_side, &r_side, &chassis);
```

它内部组装状态：

```c
x[0] = chassis->dist - chassis->target_dist;
if (fabsf(chassis->target_v) > 0.005f) x[0] = 0.0f;
x[1] = chassis->vel - chassis->target_v;
x[2] = WbrLqrWrapPi(chassis->yaw - chassis->target_yaw);
if (fabsf(chassis->target_wz) > 0.05f) x[2] = 0.0f;
x[3] = chassis->wz - chassis->target_wz;
x[4] = left->theta;
x[5] = left->theta_w;
x[6] = right->theta;
x[7] = right->theta_w;
x[8] = chassis->pitch;
x[9] = chassis->pitch_w;
```

它内部计算：

```c
u[row] -= k[row][col] * x[col];
```

输出：

```c
left->T_wheel
right->T_wheel
left->T_hip
right->T_hip
```

重要检查：

- `CalcWbrLQR()` 已经做了轮毂 `±2.5 N·m` 限幅。
- `CalcWbrLQR()` 已经做了虚拟髋力矩 `±12 N·m` 限幅。
- `LimitAndSetMotor()` 里仍应对最终关节 `T_front/T_back` 做 `±12 N·m` 成对限幅。
- 如果上车测试发现某个方向扶不住，优先改 `WBR_LQR_*_SIGN`，不要先改 K 表。

---

## 10. `LegControl()` 规划

参考来源：

`C:\Users\qianc\Desktop\26leg\RM_YunQian_WheelLeg\application\chassis\balance.c`

功能：

- 腿长 PID 输出轴向支撑力 `F_leg`。
- 加重力前馈。
- 加低增益 roll 补偿。

建议首版：

```c
float gravity_ff = BODY_MASS * 9.81f * 0.5f;

l_side.F_leg = PIDCalculate(&leglen_pid_l, l_side.height, l_side.target_len) + gravity_ff + roll_comp;
r_side.F_leg = PIDCalculate(&leglen_pid_r, r_side.height, r_side.target_len) + gravity_ff - roll_comp;
```

注意：

- `F_leg` 单位是 N。
- `T_hip` 单位是 N·m。
- `F_leg` 不是 LQR 输出，它来自腿长控制。
- `T_hip` 是 LQR 输出，后续和 `F_leg` 一起进入 VMC。

上车调参顺序：

1. 先让腿长 PID 能稳定撑住车，不接 LQR 或轮子悬空。
2. 再打开 LQR 的 `T_hip`。
3. 最后打开轮毂 `T_wheel`。

---

## 11. `VMCProject()` 规划

参考来源：

`C:\Users\qianc\Desktop\26leg\RM_YunQian_WheelLeg\application\chassis\linkNleg.h`

功能：

- 把虚拟腿轴向力 `F_leg` 和虚拟髋力矩 `T_hip` 映射为前后关节电机力矩。

老车公式：

```c
float phi12 = p->phi1 - p->phi2;
float phi34 = p->phi3 - p->phi4;
float phi32 = p->phi3 - p->phi2;
float phi03 = p->phi0 - p->phi3;
float phi02 = p->phi0 - p->phi2;
float F_m_L = p->F_leg * p->leg_len;

p->T_back = -(THIGH_LEN * msin(phi12) *
    (F_m_L * msin(phi03) + p->T_hip * mcos(phi03))) /
    (p->leg_len * msin(phi32));

p->T_front = -(THIGH_LEN * msin(phi34) *
    (F_m_L * msin(phi02) + p->T_hip * mcos(phi02))) /
    (p->leg_len * msin(phi32));
```

注意：

- 这是 VMC/Jacobian 转换，不是额外控制器。
- SJTU LQR 给的是虚拟 `T_hip`，最终关节电机仍然需要 VMC/Jacobian 映射。
- 如果五连杆接近奇异位形，`msin(phi32)` 很小，会导致关节力矩放大。必须限制腿长范围并做保护。

建议加入保护：

```c
if (fabsf(msin(phi32)) < 0.05f)
{
    p->T_front = 0.0f;
    p->T_back = 0.0f;
    return;
}
```

---

## 12. `LimitAndSetMotor()` 规划

功能：

- 对关节力矩成对缩放限幅。
- 对轮毂力矩做最终限幅。
- 把物理力矩转换为电机模块输入。
- 处理离地和急停。

关节成对缩放：

```c
static void LimitJointPair(float *front, float *back, float limit)
{
    float max_abs = fmaxf(fabsf(*front), fabsf(*back));
    if (max_abs > limit)
    {
        float scale = limit / max_abs;
        *front *= scale;
        *back *= scale;
    }
}
```

HT04：

```c
HTMotorSetRef(lf, l_side.T_front);
HTMotorSetRef(lb, l_side.T_back);
HTMotorSetRef(rf, r_side.T_front);
HTMotorSetRef(rb, r_side.T_back);
```

前提：

- HT04 当前 `OPEN_LOOP` ref 按 N·m 理解。
- 若实测方向反了，使用集中符号宏，不要散落写负号。

LK9025：

`C:\Users\qianc\Desktop\wheel-leg-mc02\modules\motor\LKmotor\LK9025.h`

当前定义：

```c
#define CURRENT_TORQUE_COEF_LK 0.003645f
```

若 LK9025 `OPEN_LOOP` ref 是电流设定值，则轮毂力矩转 ref：

```c
float l_ref = l_side.T_wheel / CURRENT_TORQUE_COEF_LK;
float r_ref = r_side.T_wheel / CURRENT_TORQUE_COEF_LK;

LKMotorSetRef(l_driven, l_ref);
LKMotorSetRef(r_driven, r_ref);
```

轮毂最终限幅：

```c
VAL_LIMIT(l_side.T_wheel, -2.5f, 2.5f);
VAL_LIMIT(r_side.T_wheel, -2.5f, 2.5f);
```

急停：

```c
HTMotorStop(lf);
HTMotorStop(lb);
HTMotorStop(rf);
HTMotorStop(rb);
LKMotorStop(l_driven);
LKMotorStop(r_driven);
```

正常启用：

```c
HTMotorEnable(lf);
HTMotorEnable(lb);
HTMotorEnable(rf);
HTMotorEnable(rb);
LKMotorEnable(l_driven);
LKMotorEnable(r_driven);
```

---

## 13. `robot.c` 接入规划

目标路径：

`C:\Users\qianc\Desktop\wheel-leg-mc02\application\robot.c`

当前逻辑：

```c
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    RobotCMDInit();
    ChassisInit();
#endif
```

```c
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    RobotCMDTask();
    ChassisTask();
#endif
```

建议新增编译开关，例如：

`C:\Users\qianc\Desktop\wheel-leg-mc02\application\robot_def.h`

```c
#define BALANCE_CHASSIS
```

然后在 `robot.c` 中：

```c
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
#ifdef BALANCE_CHASSIS
#include "balance.h"
#else
#include "chassis.h"
#endif
#include "robot_cmd.h"
#endif
```

初始化：

```c
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    RobotCMDInit();
#ifdef BALANCE_CHASSIS
    BalanceInit();
#else
    ChassisInit();
#endif
#endif
```

任务：

```c
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    RobotCMDTask();
#ifdef BALANCE_CHASSIS
    BalanceTask();
#else
    ChassisTask();
#endif
#endif
```

这样原麦轮代码仍然保留，切换只靠宏。

---

## 14. `Makefile` 接入规划

目标路径：

`C:\Users\qianc\Desktop\wheel-leg-mc02\Makefile`

当前已有：

```makefile
application/chassis/chassis.c \
```

如果 `balance.c` 是独立编译单元，需要加入：

```makefile
application/chassis/balance.c \
```

建议暂时保留 `chassis.c`，不要删除：

```makefile
application/chassis/chassis.c \
application/chassis/balance.c \
```

如果两个文件没有同名全局符号，可以一起编译。任务入口由 `robot.c` 的 `BALANCE_CHASSIS` 宏决定。

---

## 15. `robot_def.h` 和命令结构规划

目标路径：

`C:\Users\qianc\Desktop\wheel-leg-mc02\application\robot_def.h`

当前 `Chassis_Ctrl_Cmd_s` 包含：

```c
float vx;
float vy;
float wz;
float offset_angle;
chassis_mode_e chassis_mode;
```

首版平衡只需要：

- `vx`：映射为 `target_v`。
- `wz`：映射为 `target_wz`。
- `offset_angle`：可用于跟随云台或设置 `target_yaw`。
- `chassis_mode`：急停和模式切换。

如果要支持腿长变化，建议扩展：

```c
float delta_leglen; /* 腿长目标变化速度，单位 m/s */
float roll;         /* 目标 roll，单位 rad 或 degree，必须注释清楚 */
uint8_t jump;       /* 跳跃触发，首版不用 */
```

注意：

- 双板 CANComm 的 `send_data_len/recv_data_len` 使用 `sizeof(Chassis_Ctrl_Cmd_s)`，扩展结构体后两块板必须同时更新固件。
- 首版为了降低风险，可以不扩展命令结构，先固定腿长 `L0_INIT`。

---

## 16. `robot_cmd.c` 规划

目标路径：

`C:\Users\qianc\Desktop\wheel-leg-mc02\application\cmd\robot_cmd.c`

首版建议：

- `W/S` 或遥控前后通道映射到 `chassis_cmd_send.vx`。
- 鼠标或遥控 yaw 映射到 `chassis_cmd_send.wz`。
- 不使用 `vy`，轮腿不支持横移。
- `CHASSIS_ZERO_FORCE` 作为急停。
- 固定腿长时不发 `delta_leglen`。

如果扩展腿长：

```c
chassis_cmd_send.delta_leglen = key_up_down * 0.05f; /* m/s */
```

建议给速度命令加斜坡：

```c
target_v += VAL_LIMIT(raw_v - target_v, -MAX_ACC_REF * dt, MAX_ACC_REF * dt);
```

也可以放在 `balance.c` 的 `SynthesizeMotion()` 中做。

---

## 17. H723 @ 480 MHz 部署性能规划

控制频率：

```text
1 kHz
```

每周期主要计算：

- `Link2Leg()` 左右各一次：若干三角函数和平方根。
- `SlipKalman()`：少量浮点运算。
- `WbrLqrInterpolateK()`：`4 * 10` 个元素双线性插值。
- `CalcWbrLQR()`：`4 * 10` 矩阵乘法。
- `VMCProject()` 左右各一次：若干三角函数。

H723 @ 480 MHz 做 1 kHz 足够。

优化建议：

- 首版先保证正确，不急着手动优化。
- 后续把 `WBR_LQR_K_TABLE` 放 Flash/常量区即可。
- 把高频临时变量保持在栈或 DTCM。
- 若要极限优化，`WbrLqrInterpolateK()` 可以只在腿长变化超过阈值时更新 K，而不是每周期更新。
- 三角函数是主要耗时点，但 1 kHz 下仍可接受。

---

## 18. TCM 使用建议

MC02 已有：

`C:\Users\qianc\Desktop\wheel-leg-mc02\application\tcm_region\tcm_region.h`

以及示例：

`C:\Users\qianc\Desktop\wheel-leg-mc02\application\tcm_region\tcm_usage_example.c`

建议：

- 不要把 `tcm_usage_example.c` 加入 Makefile。
- 首版不强制使用 TCM。
- 如果后续要优化，可以把 `BalanceTask()`、`Link2Leg()`、`VMCProject()`、`SlipKalman()` 放 ITCM。
- 把 `l_side/r_side/chassis` 等高频状态放 DTCM。
- `WBR_LQR_K_TABLE` 很大，不建议放 DTCM；它是常量表，放 Flash 更合理。

---

## 19. 离线 K 表重新生成流程

离线脚本路径建议：

`C:\Users\qianc\Desktop\balance_simulate\并腿\wbr_lqr_offline.py`

导出头文件：

`C:\Users\qianc\Desktop\balance_simulate\并腿\wbr_lqr_calc.h`

再导入 MC02：

`C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\wbr_lqr_calc.h`

离线生成时必须确认：

- `m_w`：单个轮组等效质量。
- `m_l`：单侧腿等效质量。
- `m_b`：机体和云台等效质量。
- `I_w`：轮绕轴转动惯量。
- `I_b`：机体 pitch 惯量。
- `I_z`：整车 yaw 惯量，带云台时必须重新估。
- `R_w`：轮半径。
- `R_l`：左右轮距或半轮距定义要和脚本一致。
- `I_l(l)`：腿部等效 pitch 惯量随腿长变化。
- `l_b(l)`：机体质心到髋/腿参考点的距离随腿长变化。
- `l_w(l)`：轮到腿参考点距离随腿长变化。

腿长表当前是 10 点：

```text
0.14 m -> 0.37 m, count = 10
```

固件端使用二维双线性插值：

```text
K(left_len, right_len)
  = w00*K[i0][j0]
  + w10*K[i1][j0]
  + w01*K[i0][j1]
  + w11*K[i1][j1]
```

这和老车二次多项式拟合不同：

- 老车：单腿长输入，输出单腿 K。
- 新版：左右腿长同时输入，输出整车 4x10 K。
- 新版能覆盖左右腿长不等时的耦合，但表更大。

---

## 20. 仿真验证项目

仿真目录：

`C:\Users\qianc\Desktop\balance_simulate`

建议输出这些验证：

1. 固定腿长 `0.20 m`，pitch 初始 `5 deg`，目标 0，检查 0.2 s 内是否收敛。
2. 轮毂力矩限制 `±2.5 N·m`，检查是否长期饱和。
3. 虚拟髋力矩限制 `±12 N·m`，检查是否长期饱和。
4. 左右腿角扰动，比如 `left_theta=3 deg`、`right_theta=-3 deg`。
5. 速度阶跃 `target_v = 0.5 m/s`。
6. yaw 阶跃 `target_wz = 1 rad/s`。
7. 左右腿长不等，例如 `0.18 m / 0.24 m`。
8. 腿长扫频 `0.14 m -> 0.37 m`。

判据：

- 连续或离散闭环极点稳定。
- 线性小扰动仿真不发散。
- 力矩不长期顶限。
- 插值 K 和直接求 K 的误差可接受。
- 左右腿长相等时，左右输出具有镜像对称性。

---

## 21. 上车验证顺序

### 21.1 桌面空载

目标：只验证传感器和符号。

检查项：

- 手动转 LF/LB/RF/RB，确认 `phi1/phi4` 正方向。
- 手动压缩腿，确认 `leg_len` 变短。
- 前后摆腿，确认 `theta` 正方向。
- 抬头，确认 `pitch` 正方向。
- 左右自旋，确认 `yaw/wz` 正方向。
- 手转轮，确认 `w_ecd` 正方向。

### 21.2 悬空电机输出

目标：确认电机输出方向，不让车落地。

检查项：

- 给正 `T_wheel`，观察轮子是否产生模型正方向力矩。
- 给正 `T_hip`，观察腿是否朝模型正方向摆。
- 给正 `F_leg`，观察腿是否伸长或产生支撑。
- 如果方向错，改集中符号宏。

### 21.3 支撑腿长

目标：只开腿长 PID，不开轮毂 LQR。

检查项：

- 车架悬挂或支撑保护。
- `F_leg` 能撑起目标腿长。
- 关节力矩不长期超过 `12 N·m`。

### 21.4 小倾角站立

目标：打开 LQR，`target_v=0`。

检查项：

- 初始短腿长，比如 `0.18 m ~ 0.22 m`。
- pitch 小于 `3 deg` 时再放手。
- 记录 `pitch/pitch_w/theta/theta_w/T_wheel/T_hip/T_front/T_back`。
- 如果轮毂长期顶 `2.5 N·m`，优先降低 pitch 初始扰动或重新调 Q/R。

### 21.5 速度和 yaw

目标：逐步打开运动能力。

检查项：

- `target_v` 从 `0.1 m/s` 开始。
- `target_wz` 从 `0.2 rad/s` 开始。
- 先短时间阶跃，再长时间运行。
- yaw PID 不要同时打开，避免和 WBR LQR 的 yaw 通道重复。

---

## 22. 代码落地顺序

推荐严格按这个顺序写，不要一次性搬完整老车功能。

### 第一步：最小编译骨架

修改：

- `C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\balance.h`
- `C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\balance.c`
- `C:\Users\qianc\Desktop\wheel-leg-mc02\Makefile`
- `C:\Users\qianc\Desktop\wheel-leg-mc02\application\robot.c`

目标：

- 能编译。
- `BalanceInit()` 和 `BalanceTask()` 被调用。
- 不控制电机，只打印或记录 IMU。

### 第二步：移植结构体和传感器组装

修改：

- `C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\balance.h`
- `C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\balance.c`

目标：

- `ParamAssemble()` 能填充 pitch/yaw/roll、关节角、轮速。
- 所有单位统一。

### 第三步：移植 `Link2Leg()`

修改：

- `C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\balance.c`

或新增：

- `C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\linkNleg.h`

目标：

- 得到稳定的 `leg_len/theta/theta_w/legd`。
- 手动转动关节时数值合理。

### 第四步：移植 `LegControl()` 和 `VMCProject()`

目标：

- 腿长 PID 能输出 `F_leg`。
- VMC 能输出 `T_front/T_back`。
- 悬空验证关节输出方向。

### 第五步：接入 `CalcWbrLQR()`

修改：

- `C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\balance.c`
- `C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\wbr_lqr_calc.h`

目标：

- 用整车 LQR 一次算左右输出。
- 替代老车 `CalcLQR(&l_side...)` 双调用。
- 保持 yaw/pitch 额外 PID 关闭。

### 第六步：速度估计和运动目标

修改：

- `C:\Users\qianc\Desktop\wheel-leg-mc02\application\chassis\balance.c`
- 可选：`C:\Users\qianc\Desktop\wheel-leg-mc02\application\cmd\robot_cmd.c`
- 可选：`C:\Users\qianc\Desktop\wheel-leg-mc02\application\robot_def.h`

目标：

- `target_v` 和 `target_wz` 可控。
- 速度模式下距离闭环清零。
- 静止模式下距离回零。

---

## 23. 首版建议关闭的功能

先关闭：

- 跳跃。
- 小陀螺。
- 裁判功率限制。
- UI。
- yaw 额外 PID。
- pitch 大角补偿。
- 强抗劈叉补偿。
- 飞坡/离地复杂检测。

可以保留但低增益：

- roll 补偿。
- split 防劈叉保护。
- 离地时轮毂力矩清零。

---

## 24. 常见问题和排查

### 24.1 一开 LQR 直接倒

优先检查：

- `pitch` 单位是否误用 degree。
- `pitch_w` 轴是否错。
- `theta` 正方向是否和离线模型一致。
- `T_wheel` 输出方向是否反。
- `T_hip` 输出方向是否反。

### 24.2 腿长能撑住，但接 LQR 后关节饱和

优先检查：

- `F_leg` 重力前馈是否过大。
- `leg_len` 是否接近奇异位置。
- `VMCProject()` 符号是否正确。
- `T_hip` 限幅是否过大。
- 离线 Q/R 中腿角和 pitch 权重是否过激。

### 24.3 轮毂长期顶 2.5 N·m

优先检查：

- 初始 pitch 是否太大。
- 轮半径 `WHEEL_RADIUS` 是否填错。
- `I_w/m_w` 是否离线参数不准。
- `R` 中 wheel torque 权重是否太小。
- `target_v/dist` 是否有残余误差。

### 24.4 速度估计乱飘

优先检查：

- MC02 IMU 姿态角是 degree，内部是否转换。
- `MotionAccel_b` 是否拿到了去重力后的加速度。
- 如果只用 `attitude_t.Accel`，不要期待高速效果。
- 轮速符号是否一致。
- `phi2_w` 补偿是否方向正确。

---

## 25. 最小伪代码

`BalanceTask()` 首版可以按这个结构写：

```c
void BalanceTask(void)
{
    static float last_time = 0.0f;
    float now = DWT_GetTimeline_s();
    float dt = now - last_time;
    last_time = now;
    if (dt <= 0.0f || dt > 0.01f) dt = 0.001f;

    /* 1. 接收命令 */
    GetChassisCommand();

    /* 2. 急停和使能 */
    ControlSwitch();

    /* 3. 组装模型参数和传感器状态 */
    ParamAssemble();

    /* 4. 五连杆解算 */
    Link2Leg(&l_side, &chassis);
    Link2Leg(&r_side, &chassis);

    /* 5. 速度估计 */
    SlipKalman(&l_side, &r_side, &chassis, dt);

    /* 6. 整车 LQR */
    CalcWbrLQR(&l_side, &r_side, &chassis);

    /* 7. 腿长控制 */
    LegControl();

    /* 8. VMC 映射 */
    VMCProject(&l_side);
    VMCProject(&r_side);

    /* 9. 限幅并输出电机 */
    LimitAndSetMotor();
}
```

---

## 26. 推荐最终文件结构

首版最小结构：

```text
C:\Users\qianc\Desktop\wheel-leg-mc02
  application
    chassis
      chassis.c              保留，麦轮备用
      chassis.h              保留，麦轮备用
      balance.c              新增轮腿平衡主流程
      balance.h              新增轮腿结构体和参数
      wbr_lqr_calc.h         离线导出的 SJTU WBR LQR 表
```

后续拆分结构：

```text
C:\Users\qianc\Desktop\wheel-leg-mc02
  application
    chassis
      balance.c
      balance.h
      wbr_lqr_calc.h
      linkNleg.h             五连杆和 VMC
      speed_estimation.h     速度估计
```

首版建议少拆文件，因为 MC02 当前 `balance.c/h/lqr_calc.h` 为空，先把闭环跑通比文件颗粒度更重要。

---

## 27. 最终建议

建议用“三层验证”推进：

1. 离线仿真确认 K 表和限幅合理。
2. 固件悬空确认单位、方向、限幅正确。
3. 上车短腿长、小倾角、低速度逐步打开。

当前最关键的不是代码量，而是符号一致性：

- IMU 角度 degree/rad。
- pitch/yaw/roll 轴定义。
- 左右轮正方向。
- 左右腿 `theta` 正方向。
- HT04 前后关节输出方向。
- VMC 映射中的 `T_front/T_back` 是否对应实际前后电机。

只要这些方向统一，SJTU 整车 LQR 的优势会比老车单腿 LQR 更明显：左右腿长不等、yaw、pitch 和左右腿角耦合都能在一个 `4x10` 增益里处理。
