/**
 * @file balance.h
 * @brief 轮腿平衡机器人控制头文件
 * @note 基于SJTU WBR LQR整车10状态模型
 * @date 2026-06-09
 */

#ifndef BALANCE_H
#define BALANCE_H

#include "stdint.h"
#include "controller.h"

/* ==================== 机械参数定义 ==================== */
// 五连杆参数
#define CALF_LEN 0.15f          // 小腿长度 m
#define THIGH_LEN 0.15f         // 大腿长度 m
#define JOINT_DISTANCE 0.08f    // 髋部前后关节距离 m

// 底盘参数
#define WHEEL_RADIUS 0.06f      // 轮半径 m
#define BODY_MASS 10.0f         // 机体质量 kg

// 腿长限制
#define L0_MIN 0.14f            // 最短腿长 m
#define L0_MAX 0.37f            // 最长腿长 m
#define L0_INIT 0.20f           // 上电默认腿长 m

// 速度估计参数
#define VEL_PROCESS_NOISE 0.01f // 速度过程噪声
#define VEL_MEASURE_NOISE 0.05f // 速度测量噪声

// 关节和轮毂索引
#define JOINT_CNT 4u
#define LF 0u  // Left Front
#define LB 1u  // Left Back
#define RF 2u  // Right Front
#define RB 3u  // Right Back

#define DRIVEN_CNT 2u
#define LD 0u  // Left Driven
#define RD 1u  // Right Driven

// 单位转换
#define DEGREE_2_RAD 0.017453292519943295f  // PI/180
#define RAD_2_DEGREE 57.295779513082320876f // 180/PI

/* ==================== 结构体定义 ==================== */

/**
 * @brief 单侧腿部状态参数
 */
typedef struct
{
    /* 关节角度和角速度，单位 rad / rad/s */
    float phi1, phi2, phi3, phi4, phi0;          // 关节角度
    float phi1_w, phi2_w, phi3_w, phi4_w, phi0_w; // 关节角速度

    /* 轮毂状态 */
    float w_ecd;      // 轮毂编码器速度 rad/s
    float wheel_dist; // 轮子位移 m
    float wheel_w;    // 轮速（补偿后）rad/s
    float body_v;     // 机体速度测量 m/s
    float T_wheel;    // 轮毂力矩输出 N·m

    /* 腿部状态 */
    float theta;      // 腿角（相对竖直方向）rad
    float theta_w;    // 腿角速度 rad/s
    float leg_len;    // 当前腿长 m
    float legd;       // 腿长变化率 m/s
    float height;     // 髋部高度 m
    float height_v;   // 髋部高度变化率 m/s
    float target_len; // 目标腿长 m

    /* 腿长控制和虚拟髋力矩 */
    float F_leg; // 腿部支撑力 N
    float T_hip; // 虚拟髋力矩（来自LQR）N·m

    /* VMC 映射后的前后关节电机力矩 */
    float T_front; // 前关节力矩 N·m
    float T_back;  // 后关节力矩 N·m

    /* 离地保护 */
    float normal_force; // 支撑力 N
    uint8_t fly_flag;   // 离地标志

    /* 五连杆关键点坐标，调试用 */
    float coord[6]; // [x1,y1,x2,y2,x3,y3]
} LinkNPodParam;

/**
 * @brief 整车状态参数
 */
typedef struct
{
    /* 位移和速度，单位 m / m/s */
    float dist;        // 实际位移 m
    float target_dist; // 目标位移 m
    float vel;         // 估计速度 m/s
    float target_v;    // 目标速度 m/s
    float vel_m;       // 速度测量值 m/s
    float vel_predict; // 速度预测值 m/s
    float vel_cov;     // 速度协方差

    /* yaw，单位 rad / rad/s */
    float yaw;        // 当前yaw角 rad
    float target_yaw; // 目标yaw角 rad
    float wz;         // 当前yaw角速度 rad/s
    float target_wz;  // 目标yaw角速度 rad/s
    float offset_angle; // 底盘云台夹角 rad

    /* pitch/roll，单位 rad / rad/s */
    float pitch;    // 当前pitch角 rad
    float pitch_w;  // pitch角速度 rad/s
    float roll;     // 当前roll角 rad
    float roll_w;   // roll角速度 rad/s
    float target_roll; // 目标roll角 rad

    /* IMU 加速度和角加速度 */
    float MotionAccel_b[3]; // 机体坐标系加速度（去重力）m/s^2
    float dgyro[3];         // 角加速度 rad/s^2
    float accx, accy, accz; // 原始加速度 m/s^2
    float acc_m;            // 加速度测量 m/s^2
    float acc_last;         // 上次加速度 m/s^2

    /* 状态和调试 */
    uint8_t cali_flag; // 标定标志
    float debug[10];   // 调试数据
} ChassisParam;

/* ==================== 函数声明 ==================== */

/**
 * @brief 轮腿平衡控制初始化
 */
void BalanceInit(void);

/**
 * @brief 轮腿平衡控制任务
 * @note 建议运行频率 1kHz
 */
void BalanceTask(void);

#endif // BALANCE_H
