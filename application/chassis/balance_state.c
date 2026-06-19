#include "balance.h"

#include <math.h>
#include <string.h>

static float ClampFloat(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static float SafeSqrt(float x)
{
    return sqrtf(x > 0.0f ? x : 0.0f);
}

static float Max3Float(float a, float b, float c)
{
    float max = (a > b) ? a : b;
    return (max > c) ? max : c;
}

static float SaturateFloat(float value)
{
    return ClampFloat(value, 0.0f, 1.0f);
}

static float LowPassFloat(float last, float input, float alpha)
{
    return last + alpha * (input - last);
}

static float MapToConfidence(float value, float low, float high)
{
    if (high <= low) return 0.0f;
    return SaturateFloat((value - low) / (high - low));
}

static void InitContactSlipState(LegContactSlipState *state)
{
    if (state == 0) return;

    memset(state, 0, sizeof(*state));
    state->contact_conf = 1.0f;
    state->contact_flag = 1u;
    state->contact_state = LEG_CONTACT;
}

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
        .z1_init = 0.0f,
        .z2_init = 0.0f,
        .z3_init = 0.0f,
    };
    LESO2_Init_Config_s roll_leso_config = {
        .b0 = ROLL_LESO_B0,
        .wo = ROLL_LESO_WO,
        .z1_init = 0.0f,
        .z2_init = 0.0f,
        .z3_init = 0.0f,
    };
    LESO1_Init_Config_s wheel_speed_leso_config = {
        .b0 = WHEEL_SPEED_LESO_B0,
        .wo = WHEEL_SPEED_LESO_WO,
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

static void UpdateContactState(LinkNPodParam *left,
                               LinkNPodParam *right,
                               float dt)
{
    if (left == 0 || right == 0 || dt <= 0.0f) return;

    const float nominal_force = 0.5f * BODY_MASS * BALANCE_GRAVITY;

    UpdateLegContactState(left, nominal_force, dt);
    UpdateLegContactState(right, nominal_force, dt);
}

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

    LESO2Update(&state->chassis.pitch_leso, state->chassis.pitch, pitch_input, dt);
    LESO2Update(&state->chassis.roll_leso, state->chassis.roll, roll_input, dt);

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

static void UpdateWheelSpeedLesoState(LinkNPodParam *left,
                                      LinkNPodParam *right,
                                      float dt)
{
    if (left == 0 || right == 0 || dt <= 0.0f) return;

#if BALANCE_WHEEL_SPEED_LESO_ENABLE
    LESO1Update(&left->wheel_speed_leso, left->wheel_w, left->T_wheel, dt);
    LESO1Update(&right->wheel_speed_leso, right->wheel_w, right->T_wheel, dt);

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
