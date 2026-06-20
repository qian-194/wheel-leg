#include "chassis_estimator.hpp"

#include "state_math.hpp"

#include <math.h>

namespace balance::state
{

void AssembleImuState(ChassisParam *chassis, const attitude_t *imu) noexcept
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

void EstimateSpeed(LinkNPodParam *left,
                   LinkNPodParam *right,
                   ChassisParam *chassis,
                   float dt) noexcept
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

void UpdateAttitudeLesoState(BalanceState *state, float dt) noexcept
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

void UpdateWheelSpeedLesoState(LinkNPodParam *left,
                               LinkNPodParam *right,
                               float dt) noexcept
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

} // namespace balance::state
