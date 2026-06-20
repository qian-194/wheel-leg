# 轮腿新项目长期详细工单

目标主线：`C:\Users\qianc\Desktop\cloud-embedded-wheel-leg`  
老车参考：`C:\Users\qianc\Desktop\26leg\RM_YunQian_WheelLeg`  
仿真参考：`C:\Users\qianc\Desktop\26leg\wheel_leg2026`  
SJTU 参考：`C:\Users\qianc\Desktop\最早备战2027的车\new\上海交通大学RoboMaster2023平衡步兵控制系统开源`

本文档统一替代桌面上的旧迁移计划口径。旧文档 `mc02_sjtu_wbr_lqr_migration_plan.md` 中仍有效的函数级施工内容已经合并到本文档；所有目标路径统一为 `cloud-embedded-wheel-leg`。

---

## 0. 总原则

1. 新项目只以 `cloud-embedded-wheel-leg` 为主线。
2. 老项目只作为可运行经验库，不整包复制。
3. SJTU 方案只吸收整车 WBR LQR 建模思想，不替换工程框架。
4. 仿真项目作为机械参数、LQR 表、符号、动力学验证来源。
5. 控制内部统一 SI 单位：`m`、`m/s`、`rad`、`rad/s`、`N`、`N·m`。
6. 调试显示可以输出角度制，但角度制不得进入控制核心。
7. 每个功能必须可关闭、可回退、可单独验证。
8. 未完成架车符号验证前，不允许落地跑 LQR。
9. 不同时运行麦轮 `ChassisTask()` 和轮腿 `BalanceTask()`。

---

## 1. 当前状态校准

目标工程：`C:\Users\qianc\Desktop\cloud-embedded-wheel-leg`

当前已确认状态：

| 文件 | 当前状态 | 处理策略 |
|---|---|---|
| `application\chassis\chassis.c` | 已存在，原麦轮底盘 | 保留，不删除 |
| `application\chassis\chassis.h` | 已存在，原麦轮底盘 | 保留，不删除 |
| `application\chassis\balance.h` | 已存在，已有轮腿结构体雏形 | 复核字段和机械参数 |
| `application\chassis\balance.c` | 当前缺失 | 新建轮腿主流程 |
| `application\chassis\wbr_lqr_calc.h` | 当前缺失 | 后续由离线脚本生成或从有效版本迁入 |
| `application\chassis\linkNleg.h` | 当前缺失 | 可从老项目迁移或合入 `balance.c` 首版 |
| `application\chassis\speed_estimation.h` | 当前缺失 | 可从老项目迁移或合入 `balance.c` 首版 |
| `modules\algorithm\lqr.h/.c` | 已存在 | 不重复造通用 LQR |
| `application\robot.c` | 当前调用 `ChassisInit()` / `ChassisTask()` | 后续用宏切换到 `BalanceInit()` / `BalanceTask()` |
| `Makefile` | 已加入 `modules\algorithm\lqr.c`，未加入 `balance.c` | 后续加入轮腿应用源文件 |

路径修正：

- 旧迁移文档里的 `C:\Users\qianc\Desktop\wheel-leg-mc02` 统一替换为 `C:\Users\qianc\Desktop\cloud-embedded-wheel-leg`。
- 旧迁移文档里的 `C:\Users\qianc\Desktop\balance_simulate` 统一替换为 `C:\Users\qianc\Desktop\26leg\wheel_leg2026`。

---

## 2. 总体目标架构

首版不要追求完整比赛功能，只完成稳定、可调、可验证的平衡闭环。

轮腿主链路固定为：

```text
BalanceTask
  -> ControlSwitch
  -> ParamAssemble
  -> Link2Leg(left)
  -> Link2Leg(right)
  -> SpeedEstimate / SlipKalman
  -> CalcWbrLQR
  -> LegControl
  -> VMCProject(left)
  -> VMCProject(right)
  -> LimitAndSetMotor
```

核心取舍：

- 五连杆几何、腿长控制、VMC 映射沿用老车成熟架构。
- 平衡 LQR 从老车的“左右单腿各算一次”替换为“一次整车 10 状态 LQR”。
- yaw、pitch、左右腿角耦合由整车 LQR 接管。
- 旧 yaw PID、pitch overlay、抗劈叉、跳跃首版关闭，只保留诊断开关。
- 轮毂力矩首版限制 `±2.5 N·m`。
- 虚拟髋力矩首版限制 `±12 N·m`。
- 关节最终力矩使用成对缩放限幅，不做简单单独裁剪。

控制器分层规划：

```text
状态层：
  ParamAssemble()
  Link2Leg(left/right)
  SpeedEstimate()
  ContactEstimate()
  SlipEstimate()

目标层：
  TargetManager()
  负责 target_dist / target_v / target_yaw / target_wz / target_len

控制层：
  CalcWbrLQR()
  LegControl()
  VMCProject(left/right)

保护和执行层：
  ApplyLqrOutputSigns()
  ApplyLqrOutputLimit()
  ApplyContactProtection()
  ApplySlipProtection()
  LimitJointPair()
  LimitAndSetMotor()
```

边界要求：

- `CalcWbrLQR()` 只做纯计算：插值 K、组装误差状态、计算 `u = -K * error`。
- `TargetManager()` 处理速度模式、位置模式、yaw 模式、固定/动态腿长目标，不把模式判断塞进 LQR。
- 离地检测在 LQR 前完成状态估计，离地保护在 LQR 后限制轮毂输出。
- 打滑估计在 LQR 前修正 `chassis.vel/dist`，打滑保护在 LQR 后限制异常轮毂力矩。
- 限幅不写入 K 表生成，不写进 `CalcWbrLQR()`，统一在保护和执行层集中处理。
- 首版固定 `l_side.target_len = r_side.target_len = L0_INIT`，但 `CalcWbrLQR()` 仍用实时 `leg_len` 查表。
- 动态腿长稳定前只允许作为调试开关存在，不作为首版默认功能。

---

## 3. 阶段 0：基线冻结与资料校准

目标：冻结新工程当前状态，确认硬件接口、正方向和安全边界。

### 工单 0.1：冻结新工程基线

- 确认 `cloud-embedded-wheel-leg` 当前能否编译。
- 记录当前 git commit，或压缩备份一份干净工程。
- 不在此阶段引入任何控制逻辑。
- 验收：能复现当前编译结果，并且有明确回退版本。

### 工单 0.2：整理硬件接口表

- 记录 4 个关节电机型号、CAN 总线、`tx_id`、`rx_id`。
- 记录 2 个轮毂电机型号、CAN 总线、`tx_id`、`rx_id`。
- 记录 IMU 来源、坐标系方向、安装方向。
- 记录编码器来源、零点定义、正方向。
- 记录遥控器/上位机输入来源。
- 验收：任意一个电机或传感器都能从表中查到总线、ID、方向、单位。

### 工单 0.3：整理正方向表

- 定义车体 `x/y/z` 正方向。
- 定义 `yaw/pitch/roll` 正方向。
- 定义左右轮毂转矩正方向。
- 定义四个关节电机角度正方向。
- 定义左右腿 `theta` 正方向。
- 定义 LQR 输出 `T_wheel/T_hip` 正方向。
- 验收：能明确回答“pitch 前倾时，轮子应该给正还是负力矩”。

### 工单 0.4：确认单位边界

- HT04 关节角度：内部统一 `rad`。
- HT04 关节角速度：内部统一 `rad/s`。
- LK9025 轮速：内部统一 `rad/s`。
- IMU 姿态角如果来源为 degree，必须在 `ParamAssemble()` 入口转换为 `rad`。
- IMU 角速度必须实测确认单位和轴向。
- 验收：`ChassisParam` 和 `LinkNPodParam` 内部不保存 degree。

---

## 4. 阶段 1：最小应用骨架

目标：建立独立 `balance` 应用入口，不输出力矩。

### 工单 1.1：补齐 `balance.c`

目标文件：`C:\Users\qianc\Desktop\cloud-embedded-wheel-leg\application\chassis\balance.c`

首版只实现：

- `BalanceInit()`
- `BalanceTask()`
- 静态状态变量：`l_side`、`r_side`、`chassis`
- 电机指针占位
- IMU 指针占位
- 调试数组或日志出口

禁止：

- 禁止驱动电机输出。
- 禁止接入 LQR。
- 禁止落地测试。

验收：

- 工程能编译。
- `BalanceInit()` 和 `BalanceTask()` 可被调用。
- 所有电机输出为 0。

### 工单 1.2：复核 `balance.h`

目标文件：`C:\Users\qianc\Desktop\cloud-embedded-wheel-leg\application\chassis\balance.h`

必须保留或补齐：

- 机械参数宏：`CALF_LEN`、`THIGH_LEN`、`JOINT_DISTANCE`、`WHEEL_RADIUS`、`BODY_MASS`
- 腿长限制：`L0_MIN`、`L0_MAX`、`L0_INIT`
- 索引宏：`LF/LB/RF/RB`、`LD/RD`
- `LinkNPodParam`
- `ChassisParam`
- `BalanceInit()` / `BalanceTask()` 声明

验收：

- `wbr_lqr_calc.h` 需要的字段都能从结构体中取得。
- 同一物理量不出现多个名字和多个单位。

### 工单 1.3：接入 `robot.c` 宏切换

目标文件：

- `C:\Users\qianc\Desktop\cloud-embedded-wheel-leg\application\robot.c`
- `C:\Users\qianc\Desktop\cloud-embedded-wheel-leg\application\robot_def.h`

建议新增宏：

```c
#define BALANCE_CHASSIS
```

接入策略：

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

初始化切换：

```c
#ifdef BALANCE_CHASSIS
    BalanceInit();
#else
    ChassisInit();
#endif
```

任务切换：

```c
#ifdef BALANCE_CHASSIS
    BalanceTask();
#else
    ChassisTask();
#endif
```

验收：

- 只通过宏选择麦轮或轮腿。
- 不允许同一固件里同时跑 `ChassisTask()` 和 `BalanceTask()`。

### 工单 1.4：接入 `Makefile`

目标文件：`C:\Users\qianc\Desktop\cloud-embedded-wheel-leg\Makefile`

需要确认：

- `application/chassis/balance.c` 加入 `C_SOURCES`。
- `application/chassis` 已加入 `C_INCLUDES`。
- 后续若新增 `linkNleg.c`、`speed_estimation.c`，也加入 `C_SOURCES`。

验收：

- 开启 `BALANCE_CHASSIS` 后能编译。
- 关闭 `BALANCE_CHASSIS` 后麦轮旧逻辑仍能编译。

---

## 5. 阶段 2：硬件初始化与安全输出

目标：接入真实硬件，但仍以零力矩和低风险测试为主。

### 工单 2.1：`BalanceInit()` 初始化流程

建议流程：

```text
BalanceInit
  -> 初始化 IMU 指针
  -> 初始化 PubSub 或 CANComm
  -> 初始化 4 个 HT04 关节电机
  -> 初始化 2 个 LK9025 轮毂电机
  -> 初始化腿长 PID
  -> 初始化 roll 补偿 PID，首版可低增益或关闭
  -> 初始化 l_side.target_len = L0_INIT
  -> 初始化 r_side.target_len = L0_INIT
  -> 初始化 chassis.target_yaw = 当前 yaw
  -> 初始化 chassis.vel_cov
  -> 默认零力矩
```

验收：

- 初始化后电机不乱动。
- 遥控未使能时所有电机保持停止或零力矩。

### 工单 2.2：关节电机初始化

参考来源：`C:\Users\qianc\Desktop\26leg\RM_YunQian_WheelLeg\application\chassis\balance.c`

硬件沿用老项目：

| 位置 | 索引 | 类型 | CAN | tx_id | rx_id | 方向 |
|---|---:|---|---|---:|---:|---|
| 左前关节 LF | 0 | HT04 | hcan2 | 2 | 2 | `FEEDBACK_DIRECTION_NORMAL` |
| 左后关节 LB | 1 | HT04 | hcan2 | 3 | 3 | `FEEDBACK_DIRECTION_NORMAL` |
| 右前关节 RF | 2 | HT04 | hcan1 | 1 | 1 | `FEEDBACK_DIRECTION_REVERSE` |
| 右后关节 RB | 3 | HT04 | hcan1 | 4 | 4 | `FEEDBACK_DIRECTION_REVERSE` |

要求：

- 设置位置限位。
- 设置掉线检测。
- 首版只允许零力矩或极低力矩。

验收：

- 手动转动每个关节，反馈角度连续。
- 反馈方向和方向表一致。
- 掉线时控制输出清零。

### 工单 2.3：轮毂电机初始化

硬件沿用老项目：

| 位置 | 索引 | 类型 | CAN | tx_id | rx_id | 方向 |
|---|---:|---|---|---:|---:|---|
| 左轮 LD | 0 | LK9025 | hcan2 | 1 | 1 | `FEEDBACK_DIRECTION_REVERSE` |
| 右轮 RD | 1 | LK9025 | hcan2 | 2 | 2 | `FEEDBACK_DIRECTION_NORMAL` |

要求：

- 首版不跑速度闭环。
- 首版只测试反馈和极低力矩输出。

验收：

- 手动转左/右轮，轮速方向符合方向表。
- 左右轮同向滚动时，车体速度符号一致。

### 工单 2.4：当前生效关节限位

来源：老项目 `application\chassis\balance.h`

| 位置 | MIN rad | MAX rad | MIN deg | MAX deg |
|---|---:|---:|---:|---:|
| LF 左前 | -0.00145 | -1.6 | -0.083 | -91.67 |
| LB 左后 | 0.00728 | 1.6 | 0.417 | 91.67 |
| RF 右前 | 0.00145 | 1.6 | 0.083 | 91.67 |
| RB 右后 | -0.00145 | -1.6 | -0.083 | -91.67 |

被注释掉的旧限位：

| 位置 | 旧 MAX rad | 旧 MAX deg |
|---|---:|---:|
| LF 左前 | -0.99952 | -57.27 |
| LB 左后 | 0.99237 | 56.86 |
| RF 右前 | 1.00695 | 57.69 |
| RB 右后 | -1.00695 | -57.69 |

注意：

- 当前代码实际使用 `±1.6 rad` 这一组。
- 新项目首版可以沿用，但必须实测确认机械不会撞限位。

### 工单 2.5：IMU 数据接入

目标文件：`C:\Users\qianc\Desktop\cloud-embedded-wheel-leg\modules\imu\ins_task.h`

当前新工程中 IMU 接口需要实测确认。若使用 `attitude_t` 风格数据，常见字段包括：

```c
float Gyro[3];
float Accel[3];
float Roll;
float Pitch;
float Yaw;
float YawTotalAngle;
```

首版策略：

- 不破坏 `modules\imu`。
- 在 `ParamAssemble()` 中完成 degree 到 rad 转换。
- `Gyro[X/Y/Z]` 轴向只作为假设，必须实测确认。

验收：

- 手动抬头，`pitch` 正负符合方向表。
- 手动右倾，`roll` 正负符合方向表。
- 手动自旋，`yaw/wz` 正负符合方向表。

### 工单 2.6：调试观测通道

建议输出：

- `phi1/phi4`
- `theta/theta_w`
- `leg_len/legd`
- `wheel_w`
- `pitch/pitch_w`
- `yaw/wz`
- `T_wheel/T_hip`
- `T_front/T_back`

验收：

- VOFA、Ozone、串口或其他方式至少一种可实时观察核心状态。

---

## 6. 阶段 3：参数组装 `ParamAssemble()`

目标：把电机反馈、IMU、遥控命令统一转换成模型坐标。

### 工单 3.1：输入来源确认

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

验收：

- 每个输出字段都有唯一来源和单位。

### 工单 3.2：IMU 单位转换

如果姿态角来自 degree，则入口处立刻转换：

```c
chassis.pitch = imu->Pitch * DEGREE_2_RAD;
chassis.roll  = imu->Roll  * DEGREE_2_RAD;
chassis.yaw   = imu->YawTotalAngle * DEGREE_2_RAD;
```

角速度映射必须实测确认：

```c
chassis.pitch_w = imu->Gyro[X];
chassis.roll_w  = imu->Gyro[Y];
chassis.wz      = imu->Gyro[Z];
```

验收：

- `ChassisParam` 内所有角度都是 `rad`。
- `ChassisParam` 内所有角速度都是 `rad/s`。

### 工单 3.3：遥控命令映射

首版建议：

```c
chassis.target_v = chassis_cmd_recv.vx * BALANCE_CMD_VX_SCALE;
chassis.target_wz = chassis_cmd_recv.wz * BALANCE_CMD_WZ_SCALE;
```

如果暂时不扩展命令结构：

```c
l_side.target_len = L0_INIT;
r_side.target_len = L0_INIT;
```

后续腿长目标：

```c
target_len += delta_leglen * dt;
VAL_LIMIT(target_len, L0_MIN, L0_MAX);
l_side.target_len = target_len;
r_side.target_len = target_len;
```

验收：

- 遥控输入为 0 时，`target_v=0`、`target_wz=0`、腿长目标稳定。
- 遥控丢失时，控制切到零力矩。

---

## 7. 阶段 4：五连杆与腿部状态 `Link2Leg()`

目标：先让腿部几何和速度正确，再谈平衡。

### 工单 4.1：移植五连杆正解

参考来源：

- `C:\Users\qianc\Desktop\26leg\RM_YunQian_WheelLeg\application\chassis\linkNleg.h`
- `C:\Users\qianc\Desktop\26leg\wheel_leg2026\vmc.m`

核心输出：

```c
p->leg_len
p->theta
p->height
p->phi2
p->phi3
p->coord[]
```

关键注意：

- 老代码里 `p->theta = p->phi0 - 0.5 * PI - chassis->pitch`。
- 这意味着 `theta` 已经扣除了机体 `pitch`，是腿相对世界竖直方向的角。
- SJTU WBR LQR 状态中的 `left_theta/right_theta` 必须和离线脚本定义一致。

验收：

- 同一组关节角输入，C 代码和 MATLAB 输出误差在可接受范围内。
- 手动压缩腿，`leg_len` 变短。
- 前后摆腿，`theta` 正负符合方向表。

### 工单 4.2：腿部速度计算

核心输出：

```c
p->theta_w
p->legd
p->height_v
p->phi2_w
p->phi3_w
```

建议首版：

- 先用短时差分得到 `theta_w` 和 `legd`。
- 后续再加入更稳定的解析速度或滤波。

验收：

- 静止时 `theta_w/legd` 接近 0。
- 手动摆腿时方向正确，噪声可接受。

---

## 8. 阶段 5：速度估计 `SpeedEstimate / SlipKalman()`

目标：得到可用于 LQR 的 `chassis.vel` 和 `chassis.dist`。

### 工单 5.1：首版轮速测量

参考老车速度测量：

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

首版策略：

- 站立验证阶段可以弱化 IMU 加速度，主要信轮速。
- 低速阶跃验证阶段再调 Kalman 参数。

验收：

- 静止时 `vel_m` 接近 0。
- 手推前进时 `vel_m` 正负符合方向表。

### 工单 5.2：距离状态管理

推荐逻辑：

```c
if (fabsf(cp->target_v) < 0.005f)
{
    cp->dist += cp->vel * dt;
}
else
{
    cp->dist = 0.0f;
    cp->target_dist = 0.0f;
}
```

验收：

- 静止站立时距离闭环参与回中。
- 有速度目标时不让残余距离误差干扰运动。

### 工单 5.3：SlipKalman 后续增强

参考来源：`C:\Users\qianc\Desktop\26leg\RM_YunQian_WheelLeg\application\chassis\speed_estimation.h`

目标：

- 用轮速和腿部运动补偿得到速度测量。
- 用 IMU 水平加速度预测速度。
- 用二阶 Kalman 同时估计速度和打滑偏置。

验收：

- 轻微打滑时速度估计不长时间漂移。
- `debug[]` 能输出速度估计、轮速测量、打滑偏置。

---

## 9. 阶段 6：整车 WBR LQR

目标：完成 SJTU 风格整车 10 状态 / 4 输入 LQR，并在架车验证。

### 工单 6.1：确认新旧 LQR 差异

老车逻辑：

```text
CalcLQR(&l_side, &chassis);
CalcLQR(&r_side, &chassis);
```

老车特点：

- 单腿模型。
- 每侧腿独立计算一次。
- 每次输出该侧 `T_wheel` 和 `T_hip`。
- K 值随单侧腿长用二次多项式拟合。
- yaw 和左右腿耦合依赖额外补偿项。

新 WBR LQR 逻辑：

```text
CalcWbrLQR(&l_side, &r_side, &chassis);
```

新方案特点：

- 整车模型。
- 左右腿一起进入状态。
- 一次输出 4 个控制量。
- K 值由左右腿长二维表双线性插值。
- yaw、左右腿角、pitch、速度、位移在同一个状态空间内耦合处理。

### 工单 6.2：固定 LQR 状态顺序

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

推荐组装：

```c
x[0] = chassis->dist - chassis->target_dist;
if (fabsf(chassis->target_v) > 0.005f) x[0] = 0.0f;
x[1] = chassis->vel - chassis->target_v;
x[2] = WrapPi(chassis->yaw - chassis->target_yaw);
if (fabsf(chassis->target_wz) > 0.05f) x[2] = 0.0f;
x[3] = chassis->wz - chassis->target_wz;
x[4] = left->theta;
x[5] = left->theta_w;
x[6] = right->theta;
x[7] = right->theta_w;
x[8] = chassis->pitch;
x[9] = chassis->pitch_w;
```

验收：

- 代码、仿真、LQR 表生成脚本三者状态顺序完全一致。

### 工单 6.3：固定 LQR 输出顺序

输出顺序必须固定为：

```text
u = [
  left_wheel_torque,
  right_wheel_torque,
  left_T_hip,
  right_T_hip
]
```

输出写回：

```c
left->T_wheel  = sign_left_wheel  * Clamp(u[0], -2.5f, 2.5f);
right->T_wheel = sign_right_wheel * Clamp(u[1], -2.5f, 2.5f);
left->T_hip    = sign_left_hip    * Clamp(u[2], -12.0f, 12.0f);
right->T_hip   = sign_right_hip   * Clamp(u[3], -12.0f, 12.0f);
```

验收：

- 左右轮不交换。
- 左右髋不交换。
- 符号集中在宏或配置中，不散落写负号。

### 工单 6.4：生成或迁入 `wbr_lqr_calc.h`

目标文件：`C:\Users\qianc\Desktop\cloud-embedded-wheel-leg\application\chassis\wbr_lqr_calc.h`

内容要求：

- `WBR_LQR_X_DIM = 10`
- `WBR_LQR_U_DIM = 4`
- 左右腿长网格
- `K_TABLE[left_leg_grid][right_leg_grid][u][x]`
- 双线性插值函数
- `CalcWbrLQR()`
- 输出限幅和符号宏

验收：

- 腿长变化时 K 连续变化。
- 插值边界不会越界。
- 静止状态输入接近 0 时，LQR 输出接近 0。

---

## 10. 阶段 7：腿长控制 `LegControl()`

目标：用腿长 PID 产生支撑力 `F_leg`。

### 工单 7.1：腿长 PID

参考来源：老项目 `application\chassis\balance.c`

建议首版：

```c
float gravity_ff = BODY_MASS * 9.81f * 0.5f;

l_side.F_leg = PIDCalculate(&leglen_pid_l, l_side.height, l_side.target_len) + gravity_ff;
r_side.F_leg = PIDCalculate(&leglen_pid_r, r_side.height, r_side.target_len) + gravity_ff;
```

注意：

- `F_leg` 单位是 `N`。
- `T_hip` 单位是 `N·m`。
- `F_leg` 不是 LQR 输出，它来自腿长控制。
- `T_hip` 是 LQR 输出，后续和 `F_leg` 一起进入 VMC。

验收：

- 架车状态下只开腿长 PID，双腿能低力矩保持目标腿长。
- 关节力矩不长期饱和。

### 工单 7.2：roll 补偿预留

首版可以关闭或低增益：

```c
roll_comp = PIDCalculate(&roll_pid, chassis.roll, chassis.target_roll);
l_side.F_leg += roll_comp;
r_side.F_leg -= roll_comp;
```

验收：

- 首版关闭时不影响基础站立链路。
- 后续开启时不与 LQR 腿角控制打架。

---

## 11. 阶段 8：VMC 映射 `VMCProject()`

目标：把 `F_leg` 和 `T_hip` 映射为前后关节电机力矩。

### 工单 8.1：移植 VMC 映射

参考来源：

- 老项目 `linkNleg.h`
- 仿真项目 `wheel_leg2026\vmc.m`

输入：

```c
p->F_leg
p->T_hip
```

输出：

```c
p->T_front
p->T_back
```

验收：

- 给正 `F_leg`，腿应产生支撑或伸展趋势。
- 给正 `T_hip`，腿应朝模型正方向摆动。
- 同一组输入下，C 代码和 MATLAB VMC 输出方向一致。

### 工单 8.2：避开奇异区

要求：

- 检查 `leg_len` 是否接近机构奇异位置。
- 对雅可比行列式过小的情况限制输出。
- 输出异常时切零力矩或进入保护。

验收：

- 靠近机械极限时不会突然爆大力矩。

---

## 12. 阶段 9：最终限幅与电机输出 `LimitAndSetMotor()`

目标：集中处理最终限幅、符号、急停和电机输出。

### 工单 9.1：关节成对缩放限幅

推荐函数：

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

要求：

- 左腿前后关节成对缩放。
- 右腿前后关节成对缩放。
- 不要简单分别裁剪前后关节，避免改变 VMC 力方向。

验收：

- 限幅后前后关节力矩比例保持不变。

### 工单 9.2：HT04 输出

建议集中输出：

```c
HTMotorSetRef(lf, l_side.T_front);
HTMotorSetRef(lb, l_side.T_back);
HTMotorSetRef(rf, r_side.T_front);
HTMotorSetRef(rb, r_side.T_back);
```

前提：

- 确认 HT04 当前 `OPEN_LOOP` ref 是否按 `N·m` 理解。
- 若实测方向反了，改集中符号宏，不散落写负号。

验收：

- 正 `T_front/T_back` 的实际方向和模型一致。

### 工单 9.3：LK9025 输出

若 LK9025 `OPEN_LOOP` ref 是电流设定值，则用力矩系数换算：

```c
#define CURRENT_TORQUE_COEF_LK 0.003645f

float l_ref = l_side.T_wheel / CURRENT_TORQUE_COEF_LK;
float r_ref = r_side.T_wheel / CURRENT_TORQUE_COEF_LK;

LKMotorSetRef(l_driven, l_ref);
LKMotorSetRef(r_driven, r_ref);
```

要求：

- 轮毂最终力矩再次限幅 `±2.5 N·m`。
- 确认 `LKMotorSetRef()` 的输入单位。

验收：

- 给正 `T_wheel`，轮子实际力矩方向符合模型。

### 工单 9.4：急停与掉线保护

急停逻辑：

```c
HTMotorStop(lf);
HTMotorStop(lb);
HTMotorStop(rf);
HTMotorStop(rb);
LKMotorStop(l_driven);
LKMotorStop(r_driven);
```

触发条件：

- 遥控丢失。
- 电机掉线。
- IMU 掉线。
- pitch/roll 超过安全角。
- 腿长越界。
- 用户急停。

验收：

- 任意保护触发后，电机输出立即变为安全状态。

### 工单 9.5：明确 `CalcWbrLQR()` 与保护输出层边界

`CalcWbrLQR()` 必须保持为纯计算函数，只负责：

```text
1. 根据实时左右腿长插值 K
2. 组装 10 维误差状态 error = state - target
3. 计算 u = -K * error
4. 写回原始 T_wheel/T_hip
```

`CalcWbrLQR()` 不负责：

```text
- 速度模式下是否清零位移误差
- 转向模式下是否清零 yaw 角度误差
- 电机方向符号映射
- 离地轮毂清零
- 打滑保护
- 轮毂/髋关节力矩限幅
- 最终关节力矩限幅
- 电机发送
```

这些逻辑统一放在 LQR 之后的保护和输出层：

```text
CalcWbrLQR()
  -> ApplyLqrOutputSigns()
  -> ApplyLqrOutputLimit()
  -> ApplyContactProtection()
  -> ApplySlipProtection()
  -> LegControl()
  -> VMCProject()
  -> LimitJointPair()
  -> LimitAndSetMotor()
```

其中位移和 yaw 目标管理应在 `ParamAssemble()` 或独立目标状态管理函数中完成。
例如速度模式需要禁用位移闭环时，应令 `chassis.target_dist = chassis.dist`；
转向角速度模式需要禁用 yaw 角度闭环时，应令 `chassis.target_yaw = chassis.yaw`。

验收：

- `CalcWbrLQR()` 内不存在 `fly_flag` 判断。
- `CalcWbrLQR()` 内不存在输出限幅。
- `CalcWbrLQR()` 内不存在输出符号宏乘法。
- 保护逻辑关闭时可以直接观察原始 LQR 输出。

---

## 13. 阶段 10：桌面与架车验证

目标：逐项确认反馈、模型和输出方向。

### 工单 10.1：桌面空载检查

检查项：

- 手动转 `LF/LB/RF/RB`，确认 `phi1/phi4` 正方向。
- 手动压缩腿，确认 `leg_len` 变短。
- 前后摆腿，确认 `theta` 正方向。
- 抬头/低头，确认 `pitch` 正方向。
- 左右自旋，确认 `yaw/wz` 正方向。
- 手转轮，确认 `w_ecd` 正方向。

验收：

- 所有反馈方向与方向表一致。
- 不一致时先改反馈方向或集中符号配置，不改 LQR 表。

### 工单 10.2：悬空电机输出

检查项：

- 给正 `T_wheel`，观察轮子是否产生模型正方向力矩。
- 给正 `T_hip`，观察腿是否朝模型正方向摆。
- 给正 `F_leg`，观察腿是否伸长或产生支撑。

验收：

- 所有输出方向与模型一致。
- 如果方向错，改集中符号宏。

### 工单 10.3：支撑腿长

目标：

- 只开腿长 PID。
- 不开轮毂 LQR。

检查项：

- 车架悬挂或支撑保护。
- `F_leg` 能撑起目标腿长。
- 关节力矩不长期超过首版限幅。

验收：

- 双腿能保持 `L0_INIT`。
- 关闭控制后安全卸力。

### 工单 10.4：小倾角站立

目标：

- 打开 LQR。
- `target_v=0`。
- `target_wz=0`。

检查项：

- 初始短腿长，例如 `0.18 m ~ 0.22 m`。
- pitch 小于 `3 deg` 时再放手。
- 记录 `pitch/pitch_w/theta/theta_w/T_wheel/T_hip/T_front/T_back`。

验收：

- 小扰动下能扶正。
- 如果轮毂长期顶 `2.5 N·m`，优先检查符号、初始 pitch、Q/R、轮半径和目标残差。

### 工单 10.5：低速速度和 yaw

目标：

- 逐步打开运动能力。

起步范围：

- `target_v` 从 `0.1 m/s` 开始。
- `target_wz` 从 `0.2 rad/s` 开始。

要求：

- 先短时间阶跃，再长时间运行。
- yaw PID 不要同时打开，避免和 WBR LQR 的 yaw 通道重复。

验收：

- 前后低速运动后能回稳。
- 小 yaw 输入不破坏 pitch 平衡。

---

## 14. 阶段 11：工程化能力

目标：基础平衡稳定后，再加入鲁棒性、功率、安全和复杂动作。

### 工单 11.1：离地检测

参考来源：老项目 `fly_detection.h`

目标：

- 判断腿部是否离地。
- 离地时限制轮毂输出。
- 保持腿部姿态。

验收：

- 抬起机器人时轮子不会疯狂加速。

### 工单 11.2：roll 补偿

目标：

- 左右腿长差动保持机体横滚稳定。

验收：

- 轻微侧倾时能恢复水平。
- 不与 LQR 腿角控制打架。

### 工单 11.3：抗劈叉控制

目标：

- 限制左右腿相对角度异常扩大。

验收：

- 快速扰动时腿不会向相反方向失控展开。

### 工单 11.4：功率限制

输入：

- 裁判系统功率。
- 超级电容状态。
- 当前力矩输出。

策略：

- 优先保平衡。
- 再限制运动速度。
- 最后限制附加功能。

验收：

- 限功率时不直接摔车。

### 工单 11.5：动态腿长

前提：

- 固定腿长平衡已稳定。

验收：

- 腿长变化过程 pitch 不明显发散。

### 工单 11.6：跳跃

前提：

- 离地检测、腿长控制、力矩限幅可靠。

顺序：

- 先架车验证。
- 再低高度测试。

验收：

- 起跳、空中、落地三个阶段状态切换明确。

### 工单 11.7：云台/发射协同

目标：

- 平衡底盘与云台、发射互不破坏。

注意：

- 云台转动会影响机体姿态和质心。
- 发射震动可能影响 pitch。

验收：

- 云台动作和发射动作不会导致底盘明显失稳。

---

## 15. 阶段 12：仿真闭环与参数迭代

目标：让仿真、实车、代码形成闭环，而不是靠手感调车。

### 工单 12.1：机械参数版本化

记录参数：

- 轮半径。
- 腿杆长度。
- 关节间距。
- 质心位置。
- 机体质量。
- 轮子质量。
- 转动惯量。

验收：

- 每次修改参数都有记录。
- LQR 表能追溯参数来源。

### 工单 12.2：LQR 表生成流程

输入：

- MATLAB/Simulink 模型。
- Q/R 权重。
- 腿长网格。

输出：

- C 头文件形式的 K 表：`wbr_lqr_calc.h`

验收：

- 生成脚本可重复运行。
- 输出文件格式稳定。
- 生成结果能直接放入 `application\chassis`。

### 工单 12.3：实车日志回灌仿真

记录：

- 状态 `x`。
- 输出 `u`。
- `pitch/yaw/腿长`。
- 电机反馈。
- 电机输出。

目标：

- 用真实数据验证仿真模型偏差。

验收：

- 能定位“仿真稳，实车不稳”的主要原因。

### 工单 12.4：调参记录模板

每次调参记录：

```text
日期：
代码版本：
机械参数版本：
LQR 表版本：
Q/R：
限幅：
测试动作：
现象：
问题判断：
修改内容：
结果：
```

验收：

- 不再出现“调好了但不知道为什么”的情况。

---

## 16. 推荐代码落地顺序

严格按顺序写，不要一次性搬完整老车功能。

### 第一步：最小编译骨架

修改：

- `application\chassis\balance.c`
- `application\chassis\balance.h`
- `application\robot.c`
- `application\robot_def.h`
- `Makefile`

目标：

- 能编译。
- `BalanceInit()` 和 `BalanceTask()` 被调用。
- 不控制电机，只记录 IMU。

### 第二步：结构体和传感器组装

修改：

- `ParamAssemble()`
- 调试输出。

目标：

- 所有状态字段单位正确。
- 所有角度进入控制前转成 rad。

### 第三步：电机初始化和反馈检查

修改：

- HT04 初始化。
- LK9025 初始化。
- 掉线检测。
- 零力矩输出。

目标：

- 所有反馈方向正确。
- 遥控丢失可急停。

### 第四步：五连杆和腿长

修改：

- `Link2Leg()`
- 腿长/腿角/腿速计算。

目标：

- 几何输出和仿真一致。

### 第五步：腿长 PID 和 VMC

修改：

- `LegControl()`
- `VMCProject()`
- 关节限幅。

目标：

- 架车状态下能保持目标腿长。

### 第六步：WBR LQR

修改：

- `wbr_lqr_calc.h`
- `CalcWbrLQR()`
- LQR 状态装配。

目标：

- 架车符号验证通过。

### 第七步：低速站立和运动

修改：

- 速度估计。
- 目标速度。
- 目标 yaw。

目标：

- 固定腿长站立。
- 低速前后运动。
- 小 yaw 输入。

### 第八步：工程化功能

修改：

- 离地检测。
- roll 补偿。
- 抗劈叉。
- 功率限制。
- 动态腿长。
- 跳跃。

目标：

- 逐项增强，不破坏基础平衡闭环。

---

## 17. 可变腿长 K 表生成与调用

目标：按 SJTU 增益调度 LQR 思路，先离线生成 `K(left_leg_len, right_leg_len)` 二维表，固件运行时根据左右腿长双线性插值调用。

重要策略：

- 当前已经正常导出支持变腿长的 K 表。
- 但新项目首版控制逻辑不主动支持变腿长。
- 首版固定 `l_side.target_len = r_side.target_len = L0_INIT`。
- `CalcWbrLQR()` 仍然使用实时 `leg_len` 查表，这样即使腿部有小幅压缩或误差，LQR 增益也能匹配实际腿长。
- 等固定腿长站立稳定后，再开放 `target_len += delta_leglen * dt` 的主动变腿长逻辑。

结论：

```text
K 表支持变腿长
≠
首版控制逻辑主动变腿长
```

首版执行口径：

```text
K 表按可变腿长导出；
控制逻辑按固定腿长运行；
实时 leg_len 只用于 K 表插值和状态估计。
```

### 17.1 当前已生成文件

已生成固件头文件：

```text
C:\Users\qianc\Desktop\cloud-embedded-wheel-leg\application\chassis\wbr_lqr_calc.h
```

已生成离线数据备份：

```text
C:\Users\qianc\Desktop\balance_simulate\并腿\wbr_lqr_2026_halftrack024_iz04931.npz
```

使用的离线脚本：

```text
C:\Users\qianc\Desktop\balance_simulate\并腿\wbr_lqr_offline.py
```

脚本已按首版实车估算参数更新：

```text
half_track = 0.24 m
track_width = 0.48 m
body_length = 0.60 m
body_mass_without_gimbal = 10.022 kg
yaw_inertia = 0.4931 kg·m²
```

`yaw_inertia` 计算方法：

```text
Iz = (1/12) * 10.022 * (0.60^2 + 0.48^2)
   = 0.4931 kg·m²
```

### 17.2 当前 K 表规格

状态顺序固定为：

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

输出顺序固定为：

```text
u = [
  left_wheel_torque,
  right_wheel_torque,
  left_T_hip,
  right_T_hip
]
```

K 表规格：

```text
WBR_LQR_LEG_GRID_COUNT = 10
WBR_LQR_X_DIM = 10
WBR_LQR_U_DIM = 4
K_table shape = [10][10][4][10]
```

腿长网格：

```text
[0.14, 0.165555556, 0.191111111, 0.216666667, 0.242222222,
 0.267777778, 0.293333333, 0.318888889, 0.344444444, 0.37]
```

当前默认权重：

```text
Q = diag([20, 2, 25, 2, 800, 20, 800, 20, 1200, 30])
R = diag([0.5, 0.5, 0.3, 0.3])
```

当前限幅：

```text
wheel torque limit = ±2.5 N·m
virtual hip torque limit = ±12 N·m
joint torque limit = ±12 N·m
```

### 17.3 重新生成命令

在 PowerShell 中运行：

```powershell
python "C:\Users\qianc\Desktop\balance_simulate\并腿\wbr_lqr_offline.py" `
  --param-profile old_lqr_calc `
  --table `
  --fit-count 10 `
  --fit-min 0.14 `
  --fit-max 0.37 `
  --dt 0.001 `
  --export-c-header "C:\Users\qianc\Desktop\cloud-embedded-wheel-leg\application\chassis\wbr_lqr_calc.h" `
  --save "C:\Users\qianc\Desktop\balance_simulate\并腿\wbr_lqr_2026_halftrack024_iz04931.npz" `
  --scenario-report
```

生成结果应看到：

```text
K table bilinear interpolation max absolute error: 约 4.7e-02
Exported firmware header to ...\application\chassis\wbr_lqr_calc.h
```

当前场景报告参考结果：

```text
pitch initial 5 deg:
  final_pitch ≈ 0.0002 deg
  peak_wheel ≈ 0.9470 N·m
  peak_T_hip ≈ 3.5919 N·m

both leg initial 3 deg:
  final_pitch ≈ 0.0016 deg
  peak_wheel ≈ 2.3272 N·m
  peak_T_hip ≈ 0.8222 N·m

target velocity 0.5 m/s:
  final_vel ≈ 0.4901 m/s
  peak_wheel hits 2.5 N·m limit briefly

target yaw 5 deg:
  final_yaw ≈ 4.9999 deg
  peak_wheel ≈ 0.1917 N·m
  peak_T_hip ≈ 0.5024 N·m
```

### 17.4 固件侧如何调用

在 `balance.c` 中 include：

```c
#include "wbr_lqr_calc.h"
```

在 `BalanceTask()` 中，调用顺序必须满足：

```text
ParamAssemble()
  -> Link2Leg(left)
  -> Link2Leg(right)
  -> SpeedEstimate()
  -> CalcWbrLQR(&l_side, &r_side, &chassis)
  -> LegControl()
  -> VMCProject(left)
  -> VMCProject(right)
  -> LimitAndSetMotor()
```

最低调用前置条件：

```c
l_side.leg_len     // m
r_side.leg_len     // m
l_side.theta       // rad
l_side.theta_w     // rad/s
r_side.theta       // rad
r_side.theta_w     // rad/s
chassis.dist       // m
chassis.target_dist// m
chassis.vel        // m/s
chassis.target_v   // m/s
chassis.yaw        // rad
chassis.target_yaw // rad
chassis.wz         // rad/s
chassis.target_wz  // rad/s
chassis.pitch      // rad
chassis.pitch_w    // rad/s
```

调用后输出：

```c
l_side.T_wheel   // left raw LQR wheel torque, N·m
r_side.T_wheel   // right raw LQR wheel torque, N·m
l_side.T_hip     // left raw LQR virtual hip torque, N·m
r_side.T_hip     // right raw LQR virtual hip torque, N·m
```

注意：上述输出是未经符号映射、离地处理和限幅的原始 LQR 计算结果。
必须在后续保护和输出层完成安全处理后，才能发送给电机。

### 17.5 增益调度如何生效

固件中 `CalcWbrLQR()` 内部已经完成：

```text
1. 读取 left->leg_len 和 right->leg_len
2. clamp 到 [0.14, 0.37]
3. 找到左右腿长所在网格区间
4. 对四个邻近 K 矩阵做双线性插值
5. 计算 u = -Kx
6. 写回原始 T_wheel/T_hip
```

因此应用层只需要保证左右腿长实时正确，不需要手动选择 K。

### 17.6 需要重点验证的符号

如果架车测试发现方向不对，在后续 `ApplyOutputSigns()` 或
`LimitAndSetMotor()` 中统一使用这些集中符号宏：

```c
#define WBR_LQR_LEFT_WHEEL_SIGN  1.0f
#define WBR_LQR_RIGHT_WHEEL_SIGN 1.0f
#define WBR_LQR_LEFT_HIP_SIGN    1.0f
#define WBR_LQR_RIGHT_HIP_SIGN   1.0f
```

不要一开始就改 K 表，也不要在多个函数中散落负号。符号映射只能集中处理一次。

### 17.7 当前模型风险

当前 K 表可以用于首版架车验证，但仍有模型近似：

- `yaw_inertia = 0.4931 kg·m²` 是矩形体估算，不是 CAD 精确值。
- 腿部质心和腿部惯量仍是脚本里的近似函数。
- `target velocity 0.5 m/s` 场景已经短暂触碰轮毂 `2.5 N·m` 限幅，首次上车速度应从 `0.1 m/s` 开始。
- 所有姿态角必须用 `rad`，不得把 degree 送入 `CalcWbrLQR()`。

---

## 18. 常见问题排查

### 18.1 一开 LQR 直接倒

优先检查：

- `pitch` 是否误用 degree。
- `pitch_w` 轴是否错。
- `theta` 正方向是否和离线模型一致。
- `T_wheel` 输出方向是否反。
- `T_hip` 输出方向是否反。
- 左右腿是否交换。

### 18.2 腿长能撑住，但接 LQR 后关节饱和

优先检查：

- `F_leg` 重力前馈是否过大。
- `leg_len` 是否接近奇异位置。
- `VMCProject()` 符号是否正确。
- `T_hip` 限幅是否过大。
- 离线 Q/R 中腿角和 pitch 权重是否过激。

### 18.3 轮毂长期顶 `2.5 N·m`

优先检查：

- 初始 pitch 是否太大。
- 轮半径 `WHEEL_RADIUS` 是否填错。
- 离线质量和转动惯量是否不准。
- R 中 wheel torque 权重是否太小。
- `target_v/dist` 是否有残余误差。

### 18.4 速度估计乱飘

优先检查：

- IMU 姿态角是否转换为 rad。
- IMU 加速度是否去重力。
- 如果只用原始 `Accel`，不要期待高速效果。
- 轮速符号是否一致。
- `phi2_w` 补偿是否方向正确。

---

## 19. 最小伪代码

`BalanceTask()` 首版结构：

```c
void BalanceTask(void)
{
    static float last_time = 0.0f;
    float now = DWT_GetTimeline_s();
    float dt = now - last_time;
    last_time = now;
    if (dt <= 0.0f || dt > 0.01f)
    {
        dt = 0.001f;
    }

    GetChassisCommand();
    ControlSwitch();

    if (ShouldStop())
    {
        StopAllMotor();
        return;
    }

    ParamAssemble(dt);

    Link2Leg(&l_side, &chassis, dt);
    Link2Leg(&r_side, &chassis, dt);

    TargetManager(&l_side, &r_side, &chassis, dt);

    SpeedEstimate(&l_side, &r_side, &chassis, dt);
    ContactEstimate(&l_side, &r_side, &chassis, dt);
    SlipEstimate(&l_side, &r_side, &chassis, dt);

    CalcWbrLQR(&l_side, &r_side, &chassis);

    ApplyLqrOutputSigns(&l_side, &r_side);
    ApplyLqrOutputLimit(&l_side, &r_side);
    ApplyContactProtection(&l_side, &r_side, &chassis);
    ApplySlipProtection(&l_side, &r_side, &chassis);

    LegControl(&l_side, &r_side, &chassis);

    VMCProject(&l_side);
    VMCProject(&r_side);

    LimitJointPair(&l_side);
    LimitJointPair(&r_side);

    LimitAndSetMotor();
}
```

---

## 20. 推荐最终文件结构

首版最小结构：

```text
C:\Users\qianc\Desktop\cloud-embedded-wheel-leg
  application
    chassis
      chassis.c              保留，麦轮备用
      chassis.h              保留，麦轮备用
      balance.c              新增轮腿平衡主流程
      balance.h              轮腿结构体和参数
      wbr_lqr_calc.h         离线导出的 SJTU WBR LQR 表
  modules
    algorithm
      lqr.c                  已有通用 LQR，保留
      lqr.h                  已有通用 LQR，保留
```

后续拆分结构：

```text
C:\Users\qianc\Desktop\cloud-embedded-wheel-leg
  application
    chassis
      balance.c
      balance.h
      wbr_lqr_calc.h
      linkNleg.h             五连杆和 VMC
      speed_estimation.h     速度估计
```

首版建议少拆文件，先跑通闭环；稳定后再拆分几何、速度估计和控制器模块。

---

## 21. 最终验收标准

### 基础验收

- 工程可编译。
- 所有传感器方向正确。
- 所有电机反馈方向正确。
- 所有电机输出方向正确。
- 零力矩模式可靠。
- 遥控丢失保护可靠。

### 平衡验收

- 固定腿长可站立。
- 低速前后移动可回稳。
- 小 yaw 输入不破坏平衡。
- 抬车离地不会轮子飞转。
- 限幅状态不会长期满输出。

### 工程验收

- 每个功能可宏关闭。
- 每个阶段有回退版本。
- LQR 表可重新生成。
- 参数来源明确。
- 调试变量可观测。
