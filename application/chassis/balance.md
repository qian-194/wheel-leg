# balance

本文档记录当前平衡轮腿底盘 application 层的实现状态。旧的麦轮 chassis 流程已经不再作为本文档主体，当前重点是 `balance.c`、`balance_state.c`、`balance_control.c`、`balance_nmpc.c/.h` 和 `wbr_lqr_calc.h`。

## 入口流程

`BalanceInit()` 完成底盘 IMU、消息订阅/双板 CANComm、四个 HT04 关节电机、两个 LK9025 驱动轮电机和 `BalanceState` 的初始化。

`BalanceTask()` 是平衡底盘周期任务，主要顺序为：

1. 读取 `Chassis_Ctrl_Cmd_s` 控制命令；单板走消息中心，双板/调试走 CANComm。
2. 使用 `DWT_GetDeltaT()` 得到本周期 `del_t`。
3. 首次上电时执行关节零位校准 `JointCalibEncoder()`，校准期间停止驱动轮。
4. 根据 `chassis_mode` 选择控制分支：
   - `CHASSIS_ZERO_FORCE`：停止全部关节和驱动轮电机。
   - `CHASSIS_JOINT_ZERO_FORCE`：关节零力，双轮按命令开环。
   - `CHASSIS_STAND`/默认：使能全部电机，更新状态并计算平衡控制输出。
5. 回传 IMU 等底盘反馈数据。

站立分支当前调用：

```c
BalanceStateUpdate(&balance_state, Chassis_IMU_data, &chassis_cmd_recv, &motor_feedback, del_t);
BalanceNmpcApplyTarget(&balance_state);
BalanceControlUpdate(&balance_state, del_t);
```

注意：`BalanceNmpcApplyTarget()` 只覆盖底层控制参考量，`BalanceControlUpdate()` 当前只更新 `BalanceState` 中的力/力矩字段，不直接写电机输出。

独立 NMPC 任务入口为：

```c
StartNMPCTASK() -> BalanceNmpcTask() -> BalanceNmpcTaskUpdate(&balance_state, &chassis_cmd_recv)
```

`StartNMPCTASK()` 当前 `osDelay(5)`，按 200Hz 运行；主平衡任务侧在每个站立周期调用 `BalanceNmpcApplyTarget()` 消费最新参考。

## 状态更新

`BalanceStateUpdate()` 只做状态估计和状态机更新，不写电机。当前顺序为：

1. `AssembleImuState()`：装配 `pitch/yaw/roll`、角速度和 IMU 加速度，并统一符号。
2. `AssembleTargetState()`：装配目标速度、目标角速度、目标腿长和姿态目标。
3. `AssembleMotorState()`：装配四个关节电机角度/速度和左右轮端编码器状态。
4. `Link2Leg()`：由五连杆几何计算 `phi0`、腿长、腿角 `theta`、腿长速度等。
5. `EstimateSpeed()`：估计轮速、腿端速度、底盘速度和站立里程。
6. `UpdateAttitudeLesoState()`：按开关更新或旁路 pitch/roll 二阶 LESO。
7. `UpdateWheelSpeedLesoState()`：按开关更新或旁路单侧轮速 LESO。
8. `UpdateSlipState()`：基于左右轮差速模型估计打滑分数和打滑标志。
9. `UpdateContactState()`：基于上一周期腿向力估计接触置信度、离地和落地状态。
10. `FillDebugState()`：填充调试数组。

## 目标姿态

因为机械装配原因，实际平衡时机体 pitch/roll 不一定为 0。当前提供两个静态目标宏：

```c
#define BALANCE_TARGET_PITCH 0.0f
#define BALANCE_TARGET_ROLL 0.0f
```

`BalanceStateReset()` 和 `AssembleTargetState()` 都会写入：

```c
chassis->target_pitch = BALANCE_TARGET_PITCH;
chassis->target_roll = BALANCE_TARGET_ROLL;
```

当前 WBR LQR 的 10 维状态包含 pitch/pitch_dot，但不包含 roll/roll_dot，因此 LQR 内只使用 pitch 目标误差：

```c
error[8] = chassis->pitch - chassis->target_pitch;
error[9] = chassis->pitch_w;
```

`target_roll` 已保存在 `ChassisParam` 中，但当前不会进入 `wbr_lqr_calc.h` 的 10 维 K 表。
若要做 roll 闭环，需要重算包含 roll 的 K 表，或在外层做左右腿/髋差动控制。

## LQR

`wbr_lqr_calc.h` 由 `wbr_lqr_offline.py` 生成，模型为 SJTU-style WBR 10-state / 4-input discrete LQR。

状态顺序：

```text
[s, s_dot, yaw, yaw_dot,
 left_theta, left_theta_dot,
 right_theta, right_theta_dot,
 pitch, pitch_dot]
```

输出顺序：

```text
[left_wheel_torque, right_wheel_torque, left_T_hip, right_T_hip]
```

误差定义：

```text
dist_error    = chassis->dist - chassis->target_dist
vel_error     = chassis->vel - chassis->target_v
yaw_error     = wrap_pi(chassis->yaw - chassis->target_yaw)
yaw_dot_error = chassis->wz - chassis->target_wz
pitch_error   = chassis->pitch - chassis->target_pitch
```

`left_theta/right_theta` 和各角速度目标当前默认为 0。

K 表使用左右腿长作为二维调度量：

```c
WBR_LQR_K_TABLE[left_leg_grid][right_leg_grid][u][x]
```

运行时通过 `WbrLqrInterpolateK()` 对周围四个腿长网格点做双线性插值。

当前重要边界：

- `CalcWbrLQR()` 只负责查表、组装误差和计算原始 `u = -K * error`。
- 输出符号映射、离地保护和力矩限幅仍应在后续输出保护层统一处理。
- `balance_nmpc.c` 已复用 `WbrLqrInterpolateK()` 做预测评估，但 `CalcWbrLQR()` 本体输出还没有接入 `BalanceTask()` 的主控制链路。

## NMPC 上层规划

`balance_nmpc.c/.h` 新增的是上层参考规划器，不是直接电机控制器。它使用当前 `BalanceState`、遥控/上层 `Chassis_Ctrl_Cmd_s` 和 WBR LQR 线性化模型，给底层平衡控制输出下一周期参考：

```c
target_v
target_wz
target_pitch
target_roll
target_leg_len_l
target_leg_len_r
```

NMPC 的职责边界：

- 200Hz 任务侧 `BalanceNmpcTaskUpdate()` 计算并缓存 `BalanceNmpcTarget`。
- 主平衡任务侧 `BalanceNmpcApplyTarget()` 在 `BalanceStateUpdate()` 之后、`BalanceControlUpdate()` 之前应用参考。
- NMPC 不写 `F_leg`、`T_hip`、`T_front/T_back` 或 `T_wheel`。
- 底层 LQR/VMC、实际力矩限幅、接触/打滑保护仍属于后续控制输出层。

当前 NMPC 求解方式为有限候选搜索：

1. 先将命令裁剪到 `BALANCE_NMPC_*_LIMIT` 范围内。
2. 以 `BALANCE_NMPC_DT = 0.005f`、`BALANCE_NMPC_HORIZON = 8u` 做约 40ms 预测。
3. 对 `v/wz/pitch` 生成保持、按变化率前进一步、直接看向期望值的候选。
4. 使用 `WbrLqrInterpolateK()` 取得当前腿长处的 K，并用候选轨迹代价选择最优参考。
5. 最终输出再次经过 `BALANCE_NMPC_*_RATE_LIMIT` 限制，避免参考阶跃把底层推到饱和。

NMPC 状态机：

- `BALANCE_NMPC_DISABLED`：未使能、校准中、非站立模式或输入为空。
- `BALANCE_NMPC_WARMUP`：本周期正在尝试求解。
- `BALANCE_NMPC_ACTIVE`：目标安全有效，并已可供主控制任务消费。
- `BALANCE_NMPC_FALLBACK`：求解超时、连续无效或主任务侧发现目标过期。

保护机制：

- `BALANCE_NMPC_TIMEOUT_MS`：单次求解超过该耗时直接 fallback。
- `BALANCE_NMPC_TARGET_STALE_MS`：主控制侧发现目标太旧时不再使用。
- `BALANCE_NMPC_MAX_FAIL_COUNT`：连续无效或超时达到阈值后进入 fallback。
- `NmpcTargetIsSafe()` 会检查有限值、速度/角速度/pitch/roll 范围和左右腿长范围。

fallback 只清除 NMPC target 并重置规划器状态，底层仍按原有 `BalanceState` 目标和 `BalanceControlUpdate()` 继续工作。

## 控制输出

`BalanceControlUpdate()` 当前实现两部分：

1. 腿长控制：`CalcLegLengthForce()` 用腿长 PD + 重力前馈 + 可选 ADRC 扰动补偿计算 `F_leg`。
2. VMC 投影：`VMCProject()` 将虚拟腿向力 `F_leg` 和虚拟髋力矩 `T_hip` 投影为实际关节电机力矩 `T_front/T_back`。

VMC 投影中会对关键分母做最小绝对值保护，避免接近奇异位形时除数过小：

```c
phi32 = KeepAwayFromZero(phi32, LEG_VMC_SIN_MIN);
```

当前 `T_hip` 主要由 LQR 预留输出承载；在 LQR 接入前，如果没有其他控制器写入 `T_hip`，VMC 中的髋力矩项实际为 0。

## 限幅和符号

`wbr_lqr_calc.h` 中有三类 LQR 相关宏：

```c
WBR_LQR_WHEEL_TORQUE_LIMIT        2.5f
WBR_LQR_VIRTUAL_HIP_TORQUE_LIMIT 18.0f
WBR_LQR_JOINT_TORQUE_LIMIT       12.0f
```

含义上应分两层：

- 虚拟髋/虚拟杆限幅限制 `T_hip`，可以大于单个电机力矩。
- 实际关节限幅限制 VMC 投影后的 `T_front/T_back`，用于保护真实电机。

当前这些宏仍是输出保护层预留项，尚未在主控制链路里统一 clamp。

符号宏：

```c
WBR_LQR_LEFT_WHEEL_SIGN
WBR_LQR_RIGHT_WHEEL_SIGN
WBR_LQR_LEFT_HIP_SIGN
WBR_LQR_RIGHT_HIP_SIGN
```

用于后续根据电机安装方向翻转 LQR 输出符号。当前 `CalcWbrLQR()` 还没有读取这些宏。

## 接触和打滑

打滑检测使用左右轮差速模型预测理论轮速，再与实测轮速计算归一化残差：

```text
slip_score = abs(measured_wheel_w - predicted_wheel_w) / max(abs(measured), abs(predicted), BALANCE_SLIP_W_MIN)
```

接触检测当前使用上一周期 `F_leg` 作为法向力估计，并映射为接触置信度。接触状态包含：

- `LEG_CONTACT`
- `LEG_LIGHT_UNLOAD`
- `LEG_AIRBORNE`
- `LEG_LANDING`

落地状态会保持 `BALANCE_LANDING_PROTECT_TIME` 的保护窗口。

## 调参入口

常用调参宏集中在 `balance.h`：

- `BALANCE_TARGET_PITCH` / `BALANCE_TARGET_ROLL`：机械装配导致的平衡姿态零偏。
- `L0_MIN` / `L0_MAX` / `L0_INIT`：腿长范围和初始腿长。
- `LEG_LEN_KP` / `LEG_LEN_KD` / `LEG_FORCE_MAX`：腿长 PD 和腿向力限幅。
- `LEG_LEN_ADRC_ENABLE` 及相关 ADRC 参数：腿长方向扰动补偿。
- `BALANCE_ATTITUDE_LESO_ENABLE`：pitch/roll LESO 开关。
- `BALANCE_WHEEL_SPEED_LESO_ENABLE`：轮速 LESO 开关。
- 打滑和接触状态机阈值：`BALANCE_SLIP_*`、`BALANCE_CONTACT_*`、`BALANCE_LANDING_PROTECT_TIME`。

## 开关接口路径

当前不同补偿/估计工具的开关和主要接口如下：

| 功能 | 开关/入口 | 路径 | 说明 |
| --- | --- | --- | --- |
| NMPC 上层参考规划 | `BALANCE_NMPC_ENABLE` | `application/chassis/balance.h` | 1 使能 200Hz NMPC 参考规划，0 时主控制只使用原始目标。 |
| NMPC 周期任务 | `BalanceNmpcTaskUpdate()` | `application/chassis/balance_nmpc.c` | 由 `BalanceNmpcTask()` 调用，只生成参考目标。 |
| NMPC 目标应用 | `BalanceNmpcApplyTarget()` | `application/chassis/balance_nmpc.c`、`application/chassis/balance.c` | 在站立分支中覆盖 `target_v/wz/pitch/roll/leg_len`。 |
| 腿长 ADRC 扰动补偿 | `LEG_LEN_ADRC_ENABLE` | `application/chassis/balance.h` | 0 为腿长 PD + 重力前馈，1 时叠加 LESO 扰动补偿力。 |
| pitch/roll 姿态 LESO | `BALANCE_ATTITUDE_LESO_ENABLE` | `application/chassis/balance.h`、`application/chassis/balance_state.c` | 0 使用 IMU 原始姿态/角速度，1 时更新二阶 LESO 估计量和扰动量。 |
| 左右轮速 LESO | `BALANCE_WHEEL_SPEED_LESO_ENABLE` | `application/chassis/balance.h`、`application/chassis/balance_state.c` | 0 使用原始轮速，1 时更新轮速估计和轮速扰动。 |
| 打滑检测 | `BALANCE_SLIP_*` | `application/chassis/balance.h`、`application/chassis/balance_state.c` | 目前只输出 `slip_score/conf/flag`，暂不直接改控制输出。 |
| 接触/离地/落地检测 | `BALANCE_CONTACT_*`、`BALANCE_LANDING_PROTECT_TIME` | `application/chassis/balance.h`、`application/chassis/balance_state.c` | 目前只输出接触状态，后续可接入力矩保护层。 |

## 调参指南

建议按“先底层稳定，再打开补偿，最后打开 NMPC”的顺序调：

1. 先关闭 NMPC 和补偿工具：`BALANCE_NMPC_ENABLE = 0`、`LEG_LEN_ADRC_ENABLE = 0`、`BALANCE_ATTITUDE_LESO_ENABLE = 0`、`BALANCE_WHEEL_SPEED_LESO_ENABLE = 0`。
2. 标定机械静态零偏：调整 `BALANCE_TARGET_PITCH` / `BALANCE_TARGET_ROLL`，让机器人在期望站姿下无需明显持续前后推力。
3. 调腿长基础支撑：确认 `L0_MIN/L0_MAX/L0_INIT` 正确，再调 `LEG_LEN_KP`、`LEG_LEN_KD` 和 `LEG_GRAVITY_FF_GAIN`，目标是腿长跟踪不过冲、落地不硬顶。
4. 确认 VMC 和电机符号：检查 `F_leg` 正方向、`T_front/T_back` 投影方向和关节限幅。若符号未核对，不要打开更激进的外层参考。
5. 观察打滑/接触状态：先只看 `slip_score/conf/flag` 和 `contact_conf/state`，根据实车地面和腿向力反馈调整 `BALANCE_SLIP_*`、`BALANCE_CONTACT_*` 阈值。
6. 可选打开腿长 ADRC：先保持 `LEG_LEN_ADRC_DISTURBANCE_GAIN` 较小，逐步增大；若腿长振荡，优先降低 `LEG_LEN_ADRC_WO` 或补偿增益。
7. 可选打开姿态/轮速 LESO：先只用于观测，确认估计量无明显相位滞后或高频噪声后，再考虑让控制链路使用估计值。
8. 最后打开 NMPC：先限制 `BALANCE_NMPC_V_LIMIT`、`BALANCE_NMPC_WZ_LIMIT` 和 `BALANCE_NMPC_PITCH_FROM_V_GAIN`，确认 `active` 稳定、`timeout_count/invalid_count` 不持续增加，再逐步放宽速度和角速度上限。

NMPC 调参优先级：

- 跑不满 200Hz 或频繁 fallback：先减小候选数量/预测域，或放宽 `BALANCE_NMPC_TIMEOUT_MS` 前先确认主循环负载。
- 目标变化太猛：降低 `BALANCE_NMPC_V_RATE_LIMIT`、`BALANCE_NMPC_WZ_RATE_LIMIT`、`BALANCE_NMPC_PITCH_RATE_LIMIT`。
- 加速时 pitch 前倾/后仰不足：小幅调整 `BALANCE_NMPC_PITCH_FROM_V_GAIN`。
- 腿长动作跟不上：降低 `BALANCE_NMPC_LEG_RATE_LIMIT` 或先回到底层腿长 PD/ADRC 调参。

## 待完成

- 将 `CalcWbrLQR()` 接入站立控制主链路，并确认 NMPC 输出参考进入 LQR 误差定义后的闭环效果。
- 在 LQR 输出后统一做符号映射、虚拟髋限幅、VMC 后实际关节限幅、驱动轮限幅和离地/落地保护。
- 将 `T_front/T_back/T_wheel` 写入实际 HT04/LK9025 电机输出。
- 将 NMPC 状态 `BalanceNmpcStatus` 接入调试/遥测，重点观察 `mode/active/last_elapsed_ms/update_count/timeout_count/invalid_count/fallback_count`。
- 如果需要 roll 闭环，选择 12 维 LQR 重算 K 表，或实现外层 roll 差动腿力/髋力矩控制。
- 明确 `BALANCE_TARGET_PITCH/ROLL` 的实车标定流程。
