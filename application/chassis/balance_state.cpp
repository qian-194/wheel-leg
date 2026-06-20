#include "balance.h"

#include "state_math.hpp"
#include "leg_kinematics.hpp"
#include "chassis_estimator.hpp"
#include "contact_slip_estimator.hpp"

#include <math.h>
#include <string.h>

// 本文件负责平衡状态的装配与编排：初始化算法实例、装配命令/电机反馈、
// 按固定顺序调用各估计子模块（运动学、速度/姿态/轮速估计、打滑/接触检测）。
// 子模块实现见 leg_kinematics / chassis_estimator / contact_slip_estimator。
// 对外入口 BalanceStateReset/BalanceStateUpdate 以 extern "C" 全局符号导出，
// 供 C 侧 balance.c / balance_control.c 调用。
namespace balance::state
{

/**
 * @brief 重置整车平衡状态与算法状态。
 *
 * 清零 BalanceState 后设置初始腿长、基础支撑力、法向力、接触状态，
 * 并初始化腿长 ADRC、姿态 LESO、轮速 LESO 等算法实例。
 *
 * @param state 待重置的平衡状态对象。
 */
extern "C" void BalanceStateReset(BalanceState *state)
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
extern "C" void BalanceStateUpdate(BalanceState *state,
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

} // namespace balance::state
