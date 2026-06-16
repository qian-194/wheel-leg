#pragma once

#include "stdint.h"
#include "ins_task.h"
#include "robot_def.h"
#include "HT04.h"
#include "LK9025.h"

// 底盘参数
#define CALF_LEN 0.288f              // 小腿
#define THIGH_LEN 0.15f             // 大腿
#define JOINT_DISTANCE 0.15f        // 关节间距
#define LIMIT_LINK_RAD 0.3457      // 初始限位角度,见ParamAssemble
// 机体质量
#define BODY_MASS (10.022f) //不带头
//#define BODY_MASS (11.976f) //带头

#define L0_MAX    (0.30f) //0.3f
#define L0_MIN    (0.14f)
#define L0_INIT   (0.2f)

/**关节限位角,单位：rad */
// 腿最短时
#define LF_MIN (-0.00145f)
#define LB_MIN ( 0.00728f)
#define RF_MIN ( 0.00145f)
#define RB_MIN (-0.00145f)

// 腿最长时,去限位后
#define LF_MAX (-1.68f)
#define LB_MAX ( 1.68f)
#define RF_MAX ( 1.68f)
#define RB_MAX (-1.68f)

#define BALANCE_GRAVITY_BIAS 0
#define ROLL_GRAVITY_BIAS 0
#define BALANCE_GRAVITY 9.81f
#define LEG_GRAVITY_FF_GAIN 1.0f
#define LEG_LEN_KP 500.0f
#define LEG_LEN_KD 20.0f
#define LEG_FORCE_MAX 200.0f
#define LEG_VMC_SIN_MIN 0.05f
#define MAX_ACC_REF 1.2f    // m/s²,底盘最大加速度
#define MAX_WZ_ACC_REF 3.5f    // rad/s²,底盘最大角加速度

// 驱动轮质量
#define WHEEL_RADIUS 0.06775f         // 轮子半径
#define WHEEL_MASS (0.718f)

// IMU距离机体质心的距离
#define CENTER_IMU_X 0.0f      // IMU到机体质心的前后方向距离,正值表示IMU在前侧
#define CENTER_IMU_Z 0.0f    // IMU到机体质心的宽度方向距离,正值表示IMU在右侧
#define CENTER_IMU_Y -0.0834f      // IMU到机体质心的竖直方向距离,正值表示IMU在上方

#define VEL_PROCESS_NOISE 10    // 速度过程噪声,电机置信度
#define VEL_MEASURE_NOISE 1800  // 速度测量噪声,IMU置信度
// 同时估计加速度和速度时对加速度的噪声
// 更好的方法是设置为动态,当有冲击时/加加速度大时更相信轮速
#define ACC_PROCESS_NOISE 2000 // 加速度过程噪声
#define ACC_MEASURE_NOISE 0.01 // 加速度测量噪声

// 用于循环枚举的宏,方便访问关节电机和驱动轮电机
#define JOINT_CNT 4u
#define LF 0u
#define LB 1u
#define RF 2u
#define RB 3u

#define DRIVEN_CNT 2u
#define LD 0u
#define RD 1u

//物理参数
#define DEGREE_2_RAD 0.017453292519943295f
#define RAD_2_DEGREE 57.295779513082320876f
#define BALANCE_PI 3.14159265358979323846f
typedef struct
{
    // joint
    float phi1_w, phi4_w, phi2_w, phi3_w, phi0_w; // phi2_w or phi3_w used for calc real wheel speed
    float T_back, T_front; // 髋关节扭矩,(N·m)

    // link angle, phi1-ph5, phi0 is pod angle
    float phi1, phi2, phi3, phi4, phi0; // (rad)

    // wheel
    float w_ecd;            // 电机编码器速度,(rad/s)
    float wheel_dist;       // 单侧轮子的位移,(m)
    float wheel_w;          // 单侧轮子的速度,(rad/s)
    float body_v;           // 髋关节速度,(m/s)
    float T_wheel;          // 驱动轮扭矩,(N·m)
    float zw_ddot;          // 世界坐标系下驱动轮竖直方向加速度,上为+
    float normal_force;     // 支持力
    uint8_t fly_flag;       // 离地标志位

    // pod
    float theta, theta_w;   // 杆和垂直方向的夹角,为控制状态之一
    float leg_len, legd;    // leg_len为腿长，legd为腿长变化率
    float height, height_v; // height为腿高，height_v为腿高变化率
    float F_leg, T_hip;     // 支持力和髋关节扭矩,(N·m)
    float target_len;       // 目标腿高

    float coord[6]; // xb yb xc yc xd yd

} LinkNPodParam;

typedef struct
{
    uint8_t cali_flag;         // 关节校准标志位
    // 速度
    float vel, target_v;        // 底盘速度,单位：m/s,卡尔曼滤波后
    float vel_m;                // 底盘速度测量值
    float vel_predict;          // 底盘速度预测值
    float vel_cov;              // 速度方差
    float acc_m, acc_last;      // 世界坐标系下水平方向加速度,用于计算速度预测值

    // 位移
    float dist, target_dist;    // 底盘位移距离
    // 转向
    float target_yaw;       // 底盘目标航向角，逆时针为+，单位：rad
    float target_roll;              // 底盘目标横滚角，右倾为+，单位：rad
    float target_wz;                // 底盘目标角速度，逆时针为+，单位：rad/s
    float offset_angle;          //底盘和归中位置的夹角，底盘相对云台逆时针为+，单位：rad，范围：(-Π,Π]

    // IMU
    float yaw, wz;              // 底盘偏航角度和角速度,逆时针自旋为+，单位：rad,rad/s
    float pitch, pitch_w;       // 底盘俯仰角度和角速度,上翘为+，单位：rad,rad/s
    float roll, roll_w;         // 底盘横滚角度和角速度,右倾为+，单位：rad,rad/s
    
    float dgyro[3];             // 底盘的角加速度
    float MotionAccel_b[3];     // 底盘在机体坐标系下的IMU加速度，前x,右z,上y

    float accx;         // 机体前进加速度，前+
    float accy;         // 机体上下加速度，上+
    float accz;         // 机体左右加速度，右+

    float debug[10];         // 串口用

} ChassisParam;

typedef struct
{
    HTMotorInstance *lf;
    HTMotorInstance *lb;
    HTMotorInstance *rf;
    HTMotorInstance *rb;
    LKMotorInstance *l_driven;
    LKMotorInstance *r_driven;
} BalanceMotorFeedback;

typedef struct
{
    LinkNPodParam left;
    LinkNPodParam right;
    ChassisParam chassis;
} BalanceState;

void BalanceStateReset(BalanceState *state);

void BalanceStateUpdate(BalanceState *state,
                        const attitude_t *imu,
                        const Chassis_Ctrl_Cmd_s *cmd,
                        const BalanceMotorFeedback *motor,
                        float dt);

void BalanceControlUpdate(BalanceState *state, float dt);

/**
 * @brief 平衡底盘初始化
 *
 */
void BalanceInit(void);

/**
 * @brief 平衡底盘任务
 *
 */
void BalanceTask(void);
//全车急停
void BalanceMotorStopAll(void);
//关节急停
void BalanceJointStop(void);
//全车使能
void BalanceMotorEnableAll(void);
