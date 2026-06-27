# Balance Contact/Slip TODO

本文档记录平衡轮腿底盘接触、离地、打滑检测与输出保护的后续工单。范围优先限制在 `application/chassis`，涉及 `modules` 层的数据结构或算法接口时需要单独确认。

## 背景

当前 `BalanceStateUpdate()` 已经包含打滑与接触状态更新：

1. `UpdateSlipState()` 基于左右轮差速模型计算轮速残差，并输出 `slip_score/slip_conf/slip_flag`。
2. `UpdateContactState()` 基于上一周期 `F_leg` 映射 `contact_conf/contact_flag/fly_flag/landing_flag`。
3. `LegContactState_e` 已覆盖 `LEG_CONTACT / LEG_LIGHT_UNLOAD / LEG_AIRBORNE / LEG_LANDING`。

当前主要短板：

1. 打滑预测速度来自轮速平均后的 `chassis->vel`，单侧打滑时预测源会被污染。
2. 离地判断主要依赖 `F_leg`，而 `F_leg` 更接近控制器输出的腿向支撑力，不等价于真实地面反力。
3. `fly_flag/slip_flag/landing_flag` 当前还没有稳定接入最终输出保护层。
4. `application` 层当前拿到的 `attitude_t` 没有 `MotionAccel_n`，`ChassisParam.MotionAccel_b` 也暂时填的是原始 `imu->Accel`，不是去重力后的运动加速度。

## 推荐优先级

1. 先做输出保护与调试观测。
2. 再做 chassis speed EKF，提升速度估计和打滑残差可信度。
3. 最后做多源接触置信度融合，替换单一 `F_leg` 输入。

## 阶段 1：调试观测与输出保护

目标：不引入新算法，先确认现有状态稳定性，并让离地/落地/打滑状态真正限制危险输出。

TODO：

1. 扩展调试量，至少能观测左右腿：
   - `contact_conf`
   - `slip_conf`
   - `contact_state`
   - `fly_flag`
   - `landing_flag`
   - `slip_flag`
2. 梳理当前输出链路，明确 LQR/VMC 输出到电机参考之间的统一保护点。
3. 新增保护函数，建议命名为 `ApplyBalanceOutputProtection()`，放在 application 层输出写电机前。
4. 离地侧轮毂力矩采用清零或软削减：
   - `LEG_AIRBORNE`：该侧 `T_wheel` 逐步削减到 0。
   - `LEG_LANDING`：落地保护窗口内限制轮毂力矩恢复斜率。
   - `LEG_LIGHT_UNLOAD`：按 `contact_conf` 缩放轮毂力矩上限。
5. 保留现有力矩限幅作为最终安全边界。

验收标准：

1. 上位机或串口能连续观察接触/打滑状态。
2. 单侧 `fly_flag` 置位时，该侧轮毂输出不会继续按 LQR 原始力矩全量输出。
3. 落地窗口内轮毂输出没有突变尖峰。
4. 不改 `modules` 层。

## 阶段 2：速度 EKF 与打滑残差升级

目标：用通用 `modules/algorithm/kalman_filter` 在 application 层封装 chassis speed EKF，输出更可信的 `v_est/acc_est`，并用左右轮相对 `v_est` 的残差判断打滑。

建议状态：

```text
x = [v, a]
```

建议量测：

```text
z0 = v_l = left_wheel_speed_body
z1 = v_r = right_wheel_speed_body
z2 = imu_forward_motion_acc
```

计算方向：

1. 左右轮先换算成推车速度，再叠加 yaw 差速补偿。
2. IMU 前向加速度应使用去重力后的运动加速度；没有数据通路前，不建议直接用原始 `Accel` 长时间积分。
3. EKF 输出：
   - `chassis->vel`
   - `chassis->acc_m`
   - 左右轮 residual
   - 可选 `chassis->vel_cov`
4. 打滑分数改为：

```text
slip_score_l = abs(v_l - v_est) / max(abs(v_est), v_min)
slip_score_r = abs(v_r - v_est) / max(abs(v_est), v_min)
```

5. 单侧 `slip_conf` 上升后，增大该侧轮速量测噪声 `R`，让 EKF 少信打滑轮。

注意事项：

1. `kalman_filter` 的自动量测调整会把 `MeasuredVector[i] == 0` 视为无效量测；底盘速度和加速度为 0 是合法状态，因此需要关闭自动调整或封装显式有效位逻辑。
2. 第一版不建议把 EKF 和保护强耦合；先并行输出 `vel_est/residual` 调参，确认稳定后再替换现有 `chassis->vel`。
3. 低速时需要较大的 `v_min` 或专门的低速门控，避免原地小抖动被放大成打滑。

验收标准：

1. 直线匀速时 `v_est` 与左右轮速度一致，残差低。
2. 急加速时不会仅因为速度变化快而长期误报打滑。
3. 单侧轮空转时，该侧 residual 明显升高，另一侧 residual 相对较低。
4. EKF 关闭或失效时可以退回当前阈值判定。

## 阶段 3：多源接触置信度融合

目标：保留当前接触状态机，将 `contact_conf` 输入从单一 `F_leg` 升级为多证据融合。

建议融合：

```text
contact_score =
  w1 * force_score
+ w2 * vertical_acc_score
+ w3 * leg_compression_score
+ w4 * wheel_spin_score
```

证据定义：

1. `force_score`：沿用当前 `F_leg / nominal_force` 映射。
2. `vertical_acc_score`：使用世界系竖直运动加速度；接近自由下落或快速下落时降低接触置信度。
3. `leg_compression_score`：用腿长目标误差、腿长速度、腿长是否被地面顶住来判断接触。
4. `wheel_spin_score`：离地时轮速容易被力矩快速拉高，但车体速度或 IMU 加速度不跟随。

状态机保留：

1. `LEG_CONTACT`
2. `LEG_LIGHT_UNLOAD`
3. `LEG_AIRBORNE`
4. `LEG_LANDING`

验收标准：

1. 正常小跳动或轻载不会频繁在接触/离地之间抖动。
2. 真离地时 `contact_conf` 能比单一 `F_leg` 更快下降。
3. 落地时能进入 `LEG_LANDING` 并保持保护窗口。
4. 打滑状态不会被误认为必然离地，只作为辅助证据。

## 需要确认的跨层事项

以下事项不属于纯 application 层修改，需要修改前单独确认：

1. 是否扩展 `modules/imu/ins_task.h` 中的 `attitude_t`，加入 `MotionAccel_b/MotionAccel_n`。
2. 是否允许 application 层直接读取 `INS` 全局对象。
3. 是否修改 `modules/algorithm/kalman_filter` 的量测有效性机制。

推荐原则：

1. 第一阶段完全不动 `modules`。
2. 第二阶段优先在 application 层封装 `kalman_filter`，不修改通用 KF 源码。
3. 若需要 `MotionAccel_n`，优先通过清晰的数据结构传递，而不是散落读取全局变量。

