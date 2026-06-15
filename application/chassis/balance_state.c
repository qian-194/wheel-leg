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

void BalanceStateReset(BalanceState *state)
{
    if (state == 0) return;

    memset(state, 0, sizeof(*state));

    state->left.target_len = L0_INIT;
    state->right.target_len = L0_INIT;
    state->left.leg_len = L0_INIT;
    state->right.leg_len = L0_INIT;

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
    FillDebugState(&state->left, &state->right, &state->chassis);
}
