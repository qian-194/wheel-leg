#include "balance.h"

#include <math.h>
#include <string.h>

/**
 * @brief 将浮点数限制在指定闭区间内。
 *
 * @param value 待限幅的输入值。
 * @param min   输出下限。
 * @param max   输出上限。
 * @return float 若 value 超出边界则返回对应边界值，否则返回 value。
 */
static float ClampFloat(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 计算非负安全平方根。
 *
 * 运动学计算中可能因浮点误差出现很小的负数，本函数将负输入钳到 0，
 * 避免 sqrtf() 产生 NaN 并污染后续状态。
 *
 * @param x 待开方数。
 * @return float sqrt(max(x, 0))。
 */
static float SafeSqrt(float x)
{
    return sqrtf(x > 0.0f ? x : 0.0f);
}

/**
 * @brief 返回三个浮点数中的最大值。
 *
 * @param a 第一个输入值。
 * @param b 第二个输入值。
 * @param c 第三个输入值。
 * @return float a、b、c 中的最大值。
 */
static float Max3Float(float a, float b, float c)
{
    float max = (a > b) ? a : b;
    return (max > c) ? max : c;
}

/**
 * @brief 将浮点数饱和到 [0, 1] 区间。
 *
 * @param value 待饱和值。
 * @return float 限幅后的归一化值。
 */
static float SaturateFloat(float value)
{
    return ClampFloat(value, 0.0f, 1.0f);
}

/**
 * @brief 一阶低通滤波更新。
 *
 * @param last  上一周期滤波输出。
 * @param input 当前输入值。
 * @param alpha 滤波系数，通常位于 [0, 1]，越大响应越快。
 * @return float 新的滤波输出。
 */
static float LowPassFloat(float last, float input, float alpha)
{
    return last + alpha * (input - last);
}

/**
 * @brief 将线性区间内的数值映射为 [0, 1] 置信度。
 *
 * value <= low 时输出 0，value >= high 时输出 1，中间线性插值。
 * 若 high <= low，返回 0 以避免除零。
 *
 * @param value 待映射值。
 * @param low   置信度为 0 的下边界。
 * @param high  置信度为 1 的上边界。
 * @return float 归一化置信度。
 */
static float MapToConfidence(float value, float low, float high)
{
    if (high <= low) return 0.0f;
    return SaturateFloat((value - low) / (high - low));
}

/**
 * @brief 初始化单腿接触与打滑状态。
 *
 * 默认认为腿已接触地面，接触置信度为 1，打滑相关计时与标志清零。
 *
 * @param state 待初始化的接触/打滑状态对象。
 */
static void InitContactSlipState(LegContactSlipState *state)
{
    if (state == 0) return;

    memset(state, 0, sizeof(*state));
    state->contact_conf = 1.0f;
    state->contact_flag = 1u;
    state->contact_state = LEG_CONTACT;
}

/**
 * @brief 重置整车平衡状态与算法状态。
 *
 * 清零 BalanceState 后设置初始腿长、基础支撑力、法向力、接触状态，
 * 并初始化腿长 ADRC、姿态 LESO、轮速 LESO 等算法实例。
 *
 * @param state 待重置的平衡状态对象。
 */
void BalanceStateReset(BalanceState *state)
{
    if (state == 0) return;

    memset(state, 0, sizeof(*state));

    const float gravity_ff = 0.5f * BODY_MASS * BALANCE_GRAVITY * LEG_GRAVITY_FF_GAIN;

    state->left.target_len = L0_INIT;
    state->right.target_len = L0_INIT;
    state->left.leg_len = L0_INIT;
    state->right.leg_len = L0_INIT;
    state->left.F_leg = gravity_ff;
    state->right.F_leg = gravity_ff;
    state->left.normal_force = 0.5f * BODY_MASS * BALANCE_GRAVITY;
    state->right.normal_force = 0.5f * BODY_MASS * BALANCE_GRAVITY;
    state->left.fly_flag = 0u;
    state->right.fly_flag = 0u;
    InitContactSlipState(&state->left.contact_slip);
    InitContactSlipState(&state->right.contact_slip);

    LADRC2_Init_Config_s leg_len_adrc_config = {
        .b0 = LEG_LEN_ADRC_B0,
        .wo = LEG_LEN_ADRC_WO,
        .freq = BALANCE_CONTROL_FREQ,
        .kp = LEG_LEN_KP * LEG_LEN_ADRC_B0,
        .kd = LEG_LEN_KD * LEG_LEN_ADRC_B0,
        .max_out = LEG_FORCE_MAX,
        .z1_init = L0_INIT,
        .z2_init = 0.0f,
        .z3_init = 0.0f,
    };

    LADRC2Init(&state->left.leg_len_adrc, &leg_len_adrc_config);
    LADRC2Init(&state->right.leg_len_adrc, &leg_len_adrc_config);

    LESO2_Init_Config_s pitch_leso_config = {
        .b0 = PITCH_LESO_B0,
        .wo = PITCH_LESO_WO,
        .freq = BALANCE_CONTROL_FREQ,
        .z1_init = 0.0f,
        .z2_init = 0.0f,
        .z3_init = 0.0f,
    };
    LESO2_Init_Config_s roll_leso_config = {
        .b0 = ROLL_LESO_B0,
        .wo = ROLL_LESO_WO,
        .freq = BALANCE_CONTROL_FREQ,
        .z1_init = 0.0f,
        .z2_init = 0.0f,
        .z3_init = 0.0f,
    };
    LESO1_Init_Config_s wheel_speed_leso_config = {
        .b0 = WHEEL_SPEED_LESO_B0,
        .wo = WHEEL_SPEED_LESO_WO,
        .freq = BALANCE_CONTROL_FREQ,
        .z1_init = 0.0f,
        .z2_init = 0.0f,
    };

    LESO2Init(&state->chassis.pitch_leso, &pitch_leso_config);
    LESO2Init(&state->chassis.roll_leso, &roll_leso_config);
    LESO1Init(&state->left.wheel_speed_leso, &wheel_speed_leso_config);
    LESO1Init(&state->right.wheel_speed_leso, &wheel_speed_leso_config);
    state->chassis.pitch_wheel_torque_ratio = ClampFloat(PITCH_WHEEL_TORQUE_RATIO, 0.0f, 1.0f);
    state->chassis.pitch_joint_torque_ratio = 1.0f - state->chassis.pitch_wheel_torque_ratio;

    state->chassis.vel_cov = 100.0f;
    state->chassis.cali_flag = 1u;
}

/**
 * @brief 从 IMU 数据装配底盘姿态、角速度和加速度状态。
 *
 * 这里统一完成坐标符号转换，供后续腿部运动学、速度估计和控制使用。
 *
 * @param chassis 待写入的底盘状态。
 * @param imu     当前 IMU 姿态数据。
 */
static void AssembleImuState(ChassisParam *chassis, const attitude_t *imu)
{
    if (imu == 0) return;

    chassis->pitch = -imu->Pitch * DEGREE_2_RAD;
    chassis->pitch_w = -imu->Gyro[X];

    chassis->yaw = imu->YawTotalAngle * DEGREE_2_RAD;
    chassis->wz = imu->Gyro[Z];

    chassis->roll = -imu->Roll * DEGREE_2_RAD;
    chassis->roll_w = -imu->Gyro[Y];

    chassis->accx = imu->Accel[X];
    chassis->accy = imu->Accel[Y];
    chassis->accz = imu->Accel[Z];

    chassis->MotionAccel_b[X] = imu->Accel[X];
    chassis->MotionAccel_b[Y] = imu->Accel[Y];
    chassis->MotionAccel_b[Z] = imu->Accel[Z];
}

/**
 * @brief 从遥控/上层命令装配目标速度、角速度、偏置角与目标腿长。
 *
 * 腿长目标会被限制在机械允许范围内；目标角速度有效时刷新目标 yaw，
 * 保持转向状态与当前 yaw 对齐。
 *
 * @param left    左腿状态。
 * @param right   右腿状态。
 * @param chassis 底盘状态。
 * @param cmd     当前底盘控制命令。
 */
static void AssembleTargetState(LinkNPodParam *left,
                                LinkNPodParam *right,
                                ChassisParam *chassis,
                                const Chassis_Ctrl_Cmd_s *cmd)
{
    if (cmd == 0) return;

    chassis->target_v = cmd->vx;
    chassis->target_wz = cmd->wz;
    chassis->offset_angle = cmd->offset_angle * DEGREE_2_RAD;

    left->target_len = ClampFloat((cmd->leg_height_l > 0.001f) ? cmd->leg_height_l : L0_INIT,
                                  L0_MIN,
                                  L0_MAX);
    right->target_len = ClampFloat((cmd->leg_height_r > 0.001f) ? cmd->leg_height_r : L0_INIT,
                                   L0_MIN,
                                   L0_MAX);

    if (fabsf(chassis->target_wz) > 0.001f)
    {
        chassis->target_yaw = chassis->yaw;
    }
}

/**
 * @brief 从各电机反馈装配左右腿关节角、关节角速度、轮速和轮端里程。
 *
 * 函数内统一左右腿电机安装方向和编码器符号，输出后续运动学使用的
 * phi1/phi4、phi1_w/phi4_w、w_ecd 与 wheel_dist。
 *
 * @param left  左腿状态。
 * @param right 右腿状态。
 * @param motor 平衡底盘相关电机反馈指针集合。
 */
static void AssembleMotorState(LinkNPodParam *left,
                               LinkNPodParam *right,
                               const BalanceMotorFeedback *motor)
{
    if (motor == 0) return;
    if (motor->lf == 0 || motor->lb == 0 ||
        motor->rf == 0 || motor->rb == 0 ||
        motor->l_driven == 0 || motor->r_driven == 0)
    {
        return;
    }

    left->phi1 = BALANCE_PI + LIMIT_LINK_RAD - motor->lb->measure.total_angle;
    left->phi1_w = -motor->lb->measure.speed_rads;
    left->phi4 = -motor->lf->measure.total_angle - LIMIT_LINK_RAD;
    left->phi4_w = -motor->lf->measure.speed_rads;
    left->w_ecd = motor->l_driven->measure.speed_rads;
    left->wheel_dist = motor->l_driven->measure.total_angle * WHEEL_RADIUS;

    right->phi1 = BALANCE_PI + LIMIT_LINK_RAD + motor->rb->measure.total_angle;
    right->phi1_w = motor->rb->measure.speed_rads;
    right->phi4 = motor->rf->measure.total_angle - LIMIT_LINK_RAD;
    right->phi4_w = motor->rf->measure.speed_rads;
    right->w_ecd = -motor->r_driven->measure.speed_rads;
    right->wheel_dist = -motor->r_driven->measure.total_angle * WHEEL_RADIUS;
}

/**
 * @brief 根据五连杆几何关系计算单腿末端、腿长和相关速度状态。
 *
 * 输入为已装配的关节角与底盘 pitch，输出腿部关键角度、端点坐标、
 * 腿长、腿长速度、摆杆角 theta、theta_w、高度与高度速度。
 *
 * @param p       待更新的单腿状态。
 * @param chassis 当前底盘状态，用于扣除机体 pitch 与 pitch_w。
 */
static void Link2Leg(LinkNPodParam *p, const ChassisParam *chassis)
{
    float xD, yD, xB, yB, BD, A0, B0, C0, xC, yC;

    p->coord[4] = xD = JOINT_DISTANCE + THIGH_LEN * cosf(p->phi4);
    p->coord[5] = yD = THIGH_LEN * sinf(p->phi4);
    p->coord[0] = xB = THIGH_LEN * cosf(p->phi1);
    p->coord[1] = yB = THIGH_LEN * sinf(p->phi1);

    BD = SafeSqrt((xD - xB) * (xD - xB) + (yD - yB) * (yD - yB));
    A0 = 2.0f * CALF_LEN * (xD - xB);
    B0 = 2.0f * CALF_LEN * (yD - yB);
    C0 = BD * BD;

    p->phi2 = 2.0f * atan2f(B0 + SafeSqrt(A0 * A0 + B0 * B0 - C0 * C0), A0 + C0);
    p->coord[2] = xC = xB + CALF_LEN * cosf(p->phi2);
    p->coord[3] = yC = yB + CALF_LEN * sinf(p->phi2);
    p->phi3 = atan2f(yC - yD, xC - xD);

    p->phi0 = atan2f(yC, xC - JOINT_DISTANCE * 0.5f);
    p->leg_len = SafeSqrt((xC - JOINT_DISTANCE * 0.5f) * (xC - JOINT_DISTANCE * 0.5f) + yC * yC);
    p->theta = p->phi0 - 0.5f * BALANCE_PI - chassis->pitch;
    p->height = p->leg_len * cosf(p->theta);

    const float predict_dt = 0.002f;
    float phi1_pred = p->phi1 + p->phi1_w * predict_dt;
    float phi4_pred = p->phi4 + p->phi4_w * predict_dt;

    xD = JOINT_DISTANCE + THIGH_LEN * cosf(phi4_pred);
    yD = THIGH_LEN * sinf(phi4_pred);
    xB = THIGH_LEN * cosf(phi1_pred);
    yB = THIGH_LEN * sinf(phi1_pred);

    BD = SafeSqrt((xD - xB) * (xD - xB) + (yD - yB) * (yD - yB));
    A0 = 2.0f * CALF_LEN * (xD - xB);
    B0 = 2.0f * CALF_LEN * (yD - yB);
    C0 = BD * BD;

    float phi2_pred = 2.0f * atan2f(B0 + SafeSqrt(A0 * A0 + B0 * B0 - C0 * C0), A0 + C0);
    xC = xB + CALF_LEN * cosf(phi2_pred);
    yC = yB + CALF_LEN * sinf(phi2_pred);

    float phi3_pred = atan2f(yC - yD, xC - xD);
    float phi0_pred = atan2f(yC, xC - JOINT_DISTANCE * 0.5f);
    float leg_len_pred = SafeSqrt((xC - JOINT_DISTANCE * 0.5f) * (xC - JOINT_DISTANCE * 0.5f) + yC * yC);

    p->phi2_w = (phi2_pred - p->phi2) / predict_dt;
    p->phi3_w = (phi3_pred - p->phi3) / predict_dt;
    p->phi0_w = (phi0_pred - p->phi0) / predict_dt;
    p->legd = (leg_len_pred - p->leg_len) / predict_dt;

    p->theta_w = (phi0_pred - 0.5f * BALANCE_PI -
                  (chassis->pitch + chassis->pitch_w * predict_dt) -
                  p->theta) / predict_dt;

    p->height_v = p->legd * cosf(p->theta) -
                  p->leg_len * sinf(p->theta) * p->theta_w;
}

/**
 * @brief 估计左右腿轮速、机体速度、底盘线速度和里程积分。
 *
 * 使用轮端编码器速度、腿部角速度和 pitch_w 估计轮地相对速度；
 * 当目标速度接近 0 时累计 dist，用作站立位置误差。
 *
 * @param left    左腿状态。
 * @param right   右腿状态。
 * @param chassis 底盘状态。
 * @param dt      本次状态更新时间，单位 s。
 */
static void EstimateSpeed(LinkNPodParam *left,
                          LinkNPodParam *right,
                          ChassisParam *chassis,
                          float dt)
{
    if (dt <= 0.0f) return;

    left->wheel_w = left->w_ecd + left->phi2_w - chassis->pitch_w;
    right->wheel_w = right->w_ecd + right->phi2_w - chassis->pitch_w;

    left->body_v = left->wheel_w * WHEEL_RADIUS +
                   left->leg_len * left->theta_w +
                   left->legd * sinf(left->theta);
    right->body_v = right->wheel_w * WHEEL_RADIUS +
                    right->leg_len * right->theta_w +
                    right->legd * sinf(right->theta);

    chassis->vel_m = 0.5f * (left->body_v + right->body_v);
    chassis->vel_predict = chassis->vel;
    chassis->vel = chassis->vel_m;
    
    if (fabsf(chassis->target_v) < 0.005f)
    {
        chassis->target_dist = 0.0f;
        chassis->dist += chassis->vel * dt;
    }
    else
    {
        chassis->target_dist = 0.0f;
        chassis->dist = 0.0f;
    }
}

/**
 * @brief 根据打滑置信度和确认时间更新打滑标志。
 *
 * 打滑置位和清除均带时间确认，避免瞬时轮速残差导致标志抖动。
 *
 * @param state 单腿接触/打滑状态。
 * @param dt    本次状态更新时间，单位 s。
 */
static void UpdateSlipFlag(LegContactSlipState *state, float dt)
{
    if (state == 0 || dt <= 0.0f) return;

    if (state->slip_flag == 0u)
    {
        state->slip_off_time = 0.0f;
        if (state->slip_conf > BALANCE_SLIP_ON_CONF)
        {
            state->slip_on_time += dt;
            if (state->slip_on_time >= BALANCE_SLIP_ON_TIME)
            {
                state->slip_flag = 1u;
                state->slip_on_time = 0.0f;
            }
        }
        else
        {
            state->slip_on_time = 0.0f;
        }
    }
    else
    {
        state->slip_on_time = 0.0f;
        if (state->slip_conf < BALANCE_SLIP_OFF_CONF)
        {
            state->slip_off_time += dt;
            if (state->slip_off_time >= BALANCE_SLIP_OFF_TIME)
            {
                state->slip_flag = 0u;
                state->slip_off_time = 0.0f;
            }
        }
        else
        {
            state->slip_off_time = 0.0f;
        }
    }
}

/**
 * @brief 更新单腿打滑分数、打滑置信度和打滑标志。
 *
 * 通过实测轮速与差速模型预测轮速的相对残差评估打滑程度，
 * 并对打滑分数做一阶低通滤波。
 *
 * @param leg               待更新的单腿状态。
 * @param predicted_wheel_w 根据底盘速度与 yaw 角速度预测的轮速。
 * @param dt                本次状态更新时间，单位 s。
 */
static void UpdateLegSlipState(LinkNPodParam *leg,
                               float predicted_wheel_w,
                               float dt)
{
    if (leg == 0 || dt <= 0.0f) return;

    const float residual = leg->wheel_w - predicted_wheel_w;
    const float denom = Max3Float(fabsf(leg->wheel_w),
                                  fabsf(predicted_wheel_w),
                                  BALANCE_SLIP_W_MIN);
    const float raw_score = fabsf(residual) / denom;

    leg->contact_slip.slip_score = LowPassFloat(leg->contact_slip.slip_score,
                                                raw_score,
                                                BALANCE_SLIP_ALPHA);
    leg->contact_slip.slip_conf = MapToConfidence(leg->contact_slip.slip_score,
                                                  BALANCE_SLIP_LOW,
                                                  BALANCE_SLIP_HIGH);
    UpdateSlipFlag(&leg->contact_slip, dt);
}

/**
 * @brief 更新左右腿打滑状态。
 *
 * 根据两轮差速模型从底盘线速度和 yaw 角速度推算左右轮理论轮速，
 * 再分别更新左右腿打滑检测状态。
 *
 * @param left    左腿状态。
 * @param right   右腿状态。
 * @param chassis 底盘状态。
 * @param dt      本次状态更新时间，单位 s。
 */
static void UpdateSlipState(LinkNPodParam *left,
                            LinkNPodParam *right,
                            const ChassisParam *chassis,
                            float dt)
{
    if (left == 0 || right == 0 || chassis == 0 || dt <= 0.0f) return;

    const float left_w_pred = (chassis->vel - TWO_WHEEL_HALF_TRACK_M * chassis->wz) / WHEEL_RADIUS;
    const float right_w_pred = (chassis->vel + TWO_WHEEL_HALF_TRACK_M * chassis->wz) / WHEEL_RADIUS;

    UpdateLegSlipState(left, left_w_pred, dt);
    UpdateLegSlipState(right, right_w_pred, dt);
}

/**
 * @brief 根据接触置信度和确认时间更新二值接触标志。
 *
 * 接触丢失和恢复均带时间确认，避免支撑力估计短时波动造成离地/落地误判。
 *
 * @param state 单腿接触/打滑状态。
 * @param dt    本次状态更新时间，单位 s。
 */
static void UpdateContactFlag(LegContactSlipState *state, float dt)
{
    if (state == 0 || dt <= 0.0f) return;

    if (state->contact_flag != 0u)
    {
        state->contact_on_time = 0.0f;
        if (state->contact_conf < BALANCE_CONTACT_OFF_CONF)
        {
            state->contact_off_time += dt;
            if (state->contact_off_time >= BALANCE_CONTACT_OFF_TIME)
            {
                state->contact_flag = 0u;
                state->contact_off_time = 0.0f;
            }
        }
        else
        {
            state->contact_off_time = 0.0f;
        }
    }
    else
    {
        state->contact_off_time = 0.0f;
        if (state->contact_conf > BALANCE_CONTACT_ON_CONF)
        {
            state->contact_on_time += dt;
            if (state->contact_on_time >= BALANCE_CONTACT_ON_TIME)
            {
                state->contact_flag = 1u;
                state->contact_on_time = 0.0f;
            }
        }
        else
        {
            state->contact_on_time = 0.0f;
        }
    }
}

/**
 * @brief 更新单腿接触状态机。
 *
 * 状态机覆盖正常接触、轻载、离地和落地保护阶段；落地阶段会保持
 * landing_flag 一段保护时间，供控制层在落地瞬间做输出保护。
 *
 * @param state 单腿接触/打滑状态。
 * @param dt    本次状态更新时间，单位 s。
 */
static void UpdateContactStateMachine(LegContactSlipState *state, float dt)
{
    if (state == 0 || dt <= 0.0f) return;

    switch (state->contact_state)
    {
    case LEG_CONTACT:
        state->landing_flag = 0u;
        state->landing_time = 0.0f;
        if (state->contact_conf < BALANCE_CONTACT_LIGHT_UNLOAD_CONF)
        {
            state->contact_state = LEG_LIGHT_UNLOAD;
        }
        break;

    case LEG_LIGHT_UNLOAD:
        if (state->contact_conf >= BALANCE_CONTACT_ON_CONF)
        {
            state->contact_state = LEG_CONTACT;
        }
        else if (state->contact_conf < BALANCE_CONTACT_AIRBORNE_CONF &&
                 state->contact_flag == 0u)
        {
            state->contact_state = LEG_AIRBORNE;
        }
        break;

    case LEG_AIRBORNE:
        if (state->contact_conf > BALANCE_CONTACT_LANDING_CONF &&
            state->contact_flag != 0u)
        {
            state->contact_state = LEG_LANDING;
            state->landing_flag = 1u;
            state->landing_time = 0.0f;
        }
        break;

    case LEG_LANDING:
        state->landing_flag = 1u;
        state->landing_time += dt;
        if (state->landing_time >= BALANCE_LANDING_PROTECT_TIME)
        {
            state->landing_flag = 0u;
            state->landing_time = 0.0f;
            state->contact_state = (state->contact_conf >= BALANCE_CONTACT_LIGHT_UNLOAD_CONF) ?
                                   LEG_CONTACT :
                                   LEG_LIGHT_UNLOAD;
        }
        break;

    default:
        state->contact_state = LEG_CONTACT;
        state->landing_flag = 0u;
        state->landing_time = 0.0f;
        break;
    }
}

/**
 * @brief 更新单腿法向力、接触置信度、接触标志和离地标志。
 *
 * 当前法向力估计直接使用上一控制周期写入的 F_leg，并映射为接触置信度。
 *
 * @param leg           待更新的单腿状态。
 * @param nominal_force 标称单腿支撑力，用于归一化接触置信度。
 * @param dt            本次状态更新时间，单位 s。
 */
static void UpdateLegContactState(LinkNPodParam *leg, float nominal_force, float dt)
{
    if (leg == 0 || dt <= 0.0f) return;

    // F_leg 在 BalanceControlUpdate() 中更新，因此这里使用的是上一控制周期的腿向力。
    leg->normal_force = leg->F_leg;

    const float support_score = (nominal_force > 0.0f) ?
                                ClampFloat(leg->normal_force / nominal_force, 0.0f, 2.0f) :
                                0.0f;
    const float contact_raw = MapToConfidence(support_score,
                                              BALANCE_CONTACT_OFF_RATIO,
                                              BALANCE_CONTACT_ON_RATIO);

    leg->contact_slip.contact_conf = LowPassFloat(leg->contact_slip.contact_conf,
                                                  contact_raw,
                                                  BALANCE_CONTACT_ALPHA);
    UpdateContactFlag(&leg->contact_slip, dt);
    UpdateContactStateMachine(&leg->contact_slip, dt);

    leg->fly_flag = (leg->contact_slip.contact_flag == 0u) ? 1u : 0u;
}

/**
 * @brief 更新左右腿接触状态。
 *
 * 使用半车重作为标称单腿支撑力，分别更新左右腿接触置信度、状态机和离地标志。
 *
 * @param left  左腿状态。
 * @param right 右腿状态。
 * @param dt    本次状态更新时间，单位 s。
 */
static void UpdateContactState(LinkNPodParam *left,
                               LinkNPodParam *right,
                               float dt)
{
    if (left == 0 || right == 0 || dt <= 0.0f) return;

    const float nominal_force = 0.5f * BODY_MASS * BALANCE_GRAVITY;

    UpdateLegContactState(left, nominal_force, dt);
    UpdateLegContactState(right, nominal_force, dt);
}

/**
 * @brief 更新 pitch/roll 的 LESO 估计状态或旁路原始 IMU 状态。
 *
 * LESO 使能时使用上一周期轮端/髋关节力矩作为弱输入更新 pitch/roll 二阶观测器；
 * 未使能时直接输出原始姿态和角速度，并清零扰动估计。
 *
 * @param state 平衡状态对象。
 * @param dt    本次状态更新时间，单位 s；此处主要作为回路有效性门控。
 */
static void UpdateAttitudeLesoState(BalanceState *state, float dt)
{
    if (state == 0 || dt <= 0.0f) return;

    state->chassis.pitch_wheel_torque_ratio = ClampFloat(PITCH_WHEEL_TORQUE_RATIO, 0.0f, 1.0f);
    state->chassis.pitch_joint_torque_ratio = 1.0f - state->chassis.pitch_wheel_torque_ratio;

#if BALANCE_ATTITUDE_LESO_ENABLE
    // 当前 pitch/roll 控制力矩尚未形成闭环，这里用上一周期轮端/髋关节力矩作为观测器弱输入。
    const float pitch_input = state->chassis.pitch_wheel_torque_ratio *
                              0.5f * (state->left.T_wheel + state->right.T_wheel) +
                              state->chassis.pitch_joint_torque_ratio *
                              0.5f * (state->left.T_hip + state->right.T_hip);
    const float roll_input = 0.5f * (state->right.T_hip - state->left.T_hip);

    // LESO 步长由初始化频率派生，这里不再传 dt；外层 dt 仍用作"回路是否在跑"的门控。
    LESO2Update(&state->chassis.pitch_leso, state->chassis.pitch, pitch_input);
    LESO2Update(&state->chassis.roll_leso, state->chassis.roll, roll_input);

    state->chassis.pitch_leso_angle = state->chassis.pitch_leso.z1;
    state->chassis.pitch_leso_rate = state->chassis.pitch_leso.z2;
    state->chassis.pitch_disturbance = state->chassis.pitch_leso.z3;
    state->chassis.roll_leso_angle = state->chassis.roll_leso.z1;
    state->chassis.roll_leso_rate = state->chassis.roll_leso.z2;
    state->chassis.roll_disturbance = state->chassis.roll_leso.z3;
#else
    state->chassis.pitch_leso_angle = state->chassis.pitch;
    state->chassis.pitch_leso_rate = state->chassis.pitch_w;
    state->chassis.pitch_disturbance = 0.0f;
    state->chassis.roll_leso_angle = state->chassis.roll;
    state->chassis.roll_leso_rate = state->chassis.roll_w;
    state->chassis.roll_disturbance = 0.0f;
#endif
}

/**
 * @brief 更新左右轮速 LESO 估计状态或旁路原始轮速。
 *
 * LESO 使能时使用上一周期轮端力矩更新一阶轮速观测器；
 * 未使能时直接使用原始轮速，并清零轮速扰动估计。
 *
 * @param left  左腿状态。
 * @param right 右腿状态。
 * @param dt    本次状态更新时间，单位 s；此处主要作为回路有效性门控。
 */
static void UpdateWheelSpeedLesoState(LinkNPodParam *left,
                                      LinkNPodParam *right,
                                      float dt)
{
    if (left == 0 || right == 0 || dt <= 0.0f) return;

#if BALANCE_WHEEL_SPEED_LESO_ENABLE
    // LESO 步长由初始化频率派生，这里不再传 dt；外层 dt 仍用作"回路是否在跑"的门控。
    LESO1Update(&left->wheel_speed_leso, left->wheel_w, left->T_wheel);
    LESO1Update(&right->wheel_speed_leso, right->wheel_w, right->T_wheel);

    left->wheel_w_leso = left->wheel_speed_leso.z1;
    left->wheel_speed_disturbance = left->wheel_speed_leso.z2;
    right->wheel_w_leso = right->wheel_speed_leso.z1;
    right->wheel_speed_disturbance = right->wheel_speed_leso.z2;
#else
    left->wheel_w_leso = left->wheel_w;
    left->wheel_speed_disturbance = 0.0f;
    right->wheel_w_leso = right->wheel_w;
    right->wheel_speed_disturbance = 0.0f;
#endif
}

/**
 * @brief 填充底盘调试数组。
 *
 * 将当前关键状态按固定顺序写入 chassis->debug，便于在线观测和上位机调试。
 *
 * @param left    左腿状态。
 * @param right   右腿状态。
 * @param chassis 底盘状态。
 */
static void FillDebugState(const LinkNPodParam *left,
                           const LinkNPodParam *right,
                           ChassisParam *chassis)
{
    chassis->debug[0] = chassis->dist;
    chassis->debug[1] = chassis->vel;
    chassis->debug[2] = left->theta;
    chassis->debug[3] = left->theta_w;
    chassis->debug[4] = right->theta;
    chassis->debug[5] = right->theta_w;
    chassis->debug[6] = left->leg_len;
    chassis->debug[7] = right->leg_len;
    chassis->debug[8] = chassis->pitch;
    chassis->debug[9] = chassis->pitch_w;
}

/**
 * @brief 平衡状态主更新入口。
 *
 * 按固定顺序装配 IMU、目标命令和电机反馈，完成腿部运动学、速度估计、
 * LESO 状态、打滑检测、接触检测和调试量填充。该函数只更新状态，
 * 不直接写电机输出。
 *
 * @param state 待更新的平衡状态对象。
 * @param imu   当前 IMU 姿态数据。
 * @param cmd   当前底盘控制命令。
 * @param motor 当前电机反馈集合。
 * @param dt    本次状态更新时间，单位 s。
 */
void BalanceStateUpdate(BalanceState *state,
                        const attitude_t *imu,
                        const Chassis_Ctrl_Cmd_s *cmd,
                        const BalanceMotorFeedback *motor,
                        float dt)
{
    if (state == 0) return;

    AssembleImuState(&state->chassis, imu);
    AssembleTargetState(&state->left, &state->right, &state->chassis, cmd);
    AssembleMotorState(&state->left, &state->right, motor);

    Link2Leg(&state->left, &state->chassis);
    Link2Leg(&state->right, &state->chassis);

    EstimateSpeed(&state->left, &state->right, &state->chassis, dt);
    UpdateAttitudeLesoState(state, dt);
    UpdateWheelSpeedLesoState(&state->left, &state->right, dt);
    UpdateSlipState(&state->left, &state->right, &state->chassis, dt);
    UpdateContactState(&state->left, &state->right, dt);
    FillDebugState(&state->left, &state->right, &state->chassis);
}
