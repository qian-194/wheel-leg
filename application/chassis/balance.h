#pragma once

#include "stdint.h"
#include "ins_task.h"
#include "robot_def.h"
#include "HT04.h"
#include "LK9025.h"
#include "adrc.h"
#include "leso.h"

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

#define BALANCE_GRAVITY_BIAS 0       // 平衡方向重力补偿偏置,预留调参项
#define ROLL_GRAVITY_BIAS 0          // 横滚方向重力补偿偏置,预留调参项
#define BALANCE_TARGET_PITCH 0.0f    // 机体平衡时的目标 pitch,rad; 机械装配不水平时在这里填入静态零偏
#define BALANCE_TARGET_ROLL 0.0f     // 机体平衡时的目标 roll,rad; 当前 WBR LQR 未包含 roll 状态,供外层 roll 控制预留
#define BALANCE_GRAVITY 9.81f        // 重力加速度,m/s^2
#define LEG_GRAVITY_FF_GAIN 1.0f     // 腿长支撑力重力前馈系数,1.0 表示按半车重补偿
#define LEG_LEN_KP 500.0f            // 腿长 PD 位置增益,N/m
#define LEG_LEN_KD 20.0f             // 腿长 PD 速度阻尼,N/(m/s)
#define LEG_FORCE_MAX 200.0f         // 单腿腿向支撑力输出上限,N
#define LEG_VMC_SIN_MIN 0.05f        // VMC 雅可比分母 sin 最小绝对值,防止奇异点除数过小
#define BALANCE_CONTROL_FREQ 1000.0f  // 平衡控制回路标称频率,Hz; LESO/ADRC 和运动学预测据此派生内部步长 dt=1/freq
#define BALANCE_NMPC_ENABLE 0        // 上层 NMPC 参考规划使能; 0 时只使用原底层 LQR/VMC 目标
// NMPC 上层参考规划参数，集中放在 balance.h 便于统一调参。
#define BALANCE_NMPC_TIMEOUT_MS 2.0f       // NMPC 单周期求解预算,ms; 不等于 200Hz 任务周期
#define BALANCE_NMPC_TARGET_STALE_MS 15.0f // BalanceTask 使用目标前检查时间戳,防止继续吃旧参考,ms
#define BALANCE_NMPC_MAX_FAIL_COUNT 3u     // 连续超时/无效解达到该次数后进入 fallback
#define BALANCE_NMPC_V_LIMIT 3.0f          // NMPC 上层目标线速度限幅,m/s
#define BALANCE_NMPC_WZ_LIMIT 6.0f         // NMPC 上层目标角速度限幅,rad/s
#define BALANCE_NMPC_PITCH_LIMIT 0.25f     // NMPC 目标 pitch 限幅,rad
#define BALANCE_NMPC_ROLL_LIMIT 0.20f      // NMPC 目标 roll 限幅,rad
#define BALANCE_NMPC_DT 0.005f             // 200Hz NMPC 步长,s
#define BALANCE_NMPC_HORIZON 8u            // NMPC 预测步数; 8 步约 40ms
#define BALANCE_NMPC_V_RATE_LIMIT 1.2f     // NMPC 参考线速度变化率限幅,m/s^2
#define BALANCE_NMPC_WZ_RATE_LIMIT 3.5f    // NMPC 参考角速度变化率限幅,rad/s^2
#define BALANCE_NMPC_LEG_RATE_LIMIT 0.45f  // NMPC 目标腿长变化率限幅,m/s
#define BALANCE_NMPC_PITCH_FROM_V_GAIN 0.035f // 根据目标线速度生成 pitch 前馈的增益,rad/(m/s)
#define BALANCE_NMPC_PITCH_RATE_LIMIT 0.60f // NMPC 目标 pitch 变化率限幅,rad/s
#define BALANCE_NMPC_LEG_MASS 2.292f       // NMPC 模型单侧腿等效质量,kg
#define BALANCE_NMPC_WHEEL_INERTIA 0.0025f // NMPC 模型单侧驱动轮转动惯量,kg*m^2
#define BALANCE_NMPC_BODY_INERTIA_PITCH 0.207300f // NMPC 模型机体 pitch 转动惯量,kg*m^2
#define BALANCE_NMPC_BODY_INERTIA_YAW 0.4931f // NMPC 模型机体 yaw 转动惯量,kg*m^2
#define BALANCE_NMPC_BODY_COM_OFFSET_X 0.00497f // NMPC 模型机体质心相对轮轴前后偏置,m
#define BALANCE_NMPC_LEG_INERTIA_WIDTH 0.05f // NMPC 腿部等效惯量宽度项,m
#define IMU_TEMP_CTRL_ENABLE 0       // IMU 恒温控制使能; 0 为 debug 关温控,1 为 download/run 开温控
#define LEG_LEN_ADRC_ENABLE 0        // 腿长 ADRC 扰动补偿使能; 0 为纯 PD+重力前馈
#define LEG_LEN_ADRC_B0 (2.0f / BODY_MASS) // 腿长对象输入增益估计,(m/s^2)/N; 近似按半车质量估计
#define LEG_LEN_ADRC_WO 35.0f        // 腿长 LESO 观测器带宽,rad/s
#define LEG_LEN_ADRC_DISTURBANCE_GAIN 0.25f // 扰动补偿注入系数; 先小比例接入,上车后逐步调
#define LEG_LEN_ADRC_DISTURBANCE_MAX 60.0f  // 单腿 ADRC 扰动补偿力限幅,N
#define BALANCE_ATTITUDE_LESO_ENABLE 0 // pitch/roll 二阶 LESO 使能; 0 时只保留原始 IMU 状态
#define BALANCE_WHEEL_SPEED_LESO_ENABLE 0 // 左右轮速一阶 LESO 使能; 0 时只使用原始轮速
#define PITCH_LESO_B0 1.0f             // pitch 输入增益估计; 当前 pitch 力矩模型未闭环,先按归一化输入处理
#define PITCH_LESO_WO 35.0f            // pitch 二阶 LESO 观测器带宽,rad/s
#define ROLL_LESO_B0 1.0f              // roll 输入增益估计; 当前 roll 力矩模型未闭环,先按归一化输入处理
#define ROLL_LESO_WO 30.0f             // roll 二阶 LESO 观测器带宽,rad/s
#define WHEEL_SPEED_LESO_B0 1.0f       // 轮速一阶 LESO 输入增益估计; 输入为上一周期轮端力矩
#define WHEEL_SPEED_LESO_WO 35.0f      // 轮速一阶 LESO 观测器带宽,rad/s
#define PITCH_WHEEL_TORQUE_RATIO 0.60f // pitch 控制量分配到轮力矩的比例,剩余部分预留给髋/关节力矩
#define MAX_ACC_REF BALANCE_NMPC_V_RATE_LIMIT     // 兼容旧命名: 底盘最大前向加速度,m/s^2
#define MAX_WZ_ACC_REF BALANCE_NMPC_WZ_RATE_LIMIT // 兼容旧命名: 底盘最大角加速度,rad/s^2

// 腿力分层控制与 RollTask。默认全部关闭，保证回归时 F_leg 仍等于基础腿长控制输出。
#define BALANCE_TURN_LOAD_FF_ENABLE 0      // 转向压腿惯量前馈使能
#define BALANCE_ROLL_TASK_ENABLE 0         // 台阶/不等腿长 roll 反馈补偿使能
#define BALANCE_ROLL_TASK_LESO_ENABLE 0    // RollTask 专用二阶 LESO 扰动补偿使能
#define BALANCE_CONTACT_PROTECT_ENABLE 0   // 接触/落地腿力保护使能
#define BALANCE_TURN_LOAD_K_VWZ_FF 0.0f    // v * target_wz 转向压腿前馈增益,N/(m/s*rad/s)
#define BALANCE_TURN_LOAD_K_WZ2_FF 0.0f    // target_wz * abs(target_wz) 转向压腿前馈增益,N/(rad/s)^2
#define BALANCE_TURN_LOAD_K_WZDOT_FF 0.0f  // target_wz 差分前馈增益,N/(rad/s^2)
#define BALANCE_TURN_LOAD_FF_MAX_RATIO 0.15f // 转向压腿前馈限幅,相对单腿标称支撑力
#define BALANCE_TURN_LOAD_ALPHA 0.10f      // 转向压腿前馈一阶低通系数
#define BALANCE_ROLL_REF_K_WZ 0.0f         // 转向 roll 参考 target_wz 增益,rad/(rad/s)
#define BALANCE_ROLL_REF_K_VWZ 0.0f        // 转向 roll 参考 v*target_wz 增益,rad/(m/s*rad/s)
#define BALANCE_ROLL_KP 0.0f               // roll 反馈比例增益,N/rad
#define BALANCE_ROLL_KD 0.0f               // roll 反馈阻尼增益,N/(rad/s)
#define BALANCE_ROLL_FB_MAX_RATIO 0.15f    // roll 反馈限幅,相对单腿标称支撑力
#define BALANCE_ROLL_TASK_LESO_B0 1.0f     // RollTask LESO 输入增益估计,(rad/s^2)/N
#define BALANCE_ROLL_TASK_LESO_WO 25.0f    // RollTask LESO 观测器带宽,rad/s
#define BALANCE_ROLL_DISTURBANCE_GAIN 0.0f // RollTask LESO 扰动补偿注入系数
#define BALANCE_ROLL_DISTURBANCE_MAX_RATIO 0.10f // RollTask 扰动补偿限幅,相对单腿标称支撑力
#define BALANCE_ROLL_WEIGHT_BASE 1.0f      // roll 反馈基础权重
#define BALANCE_ROLL_WEIGHT_MIN 0.0f       // roll 反馈最小权重
#define BALANCE_ROLL_WEIGHT_MAX 1.0f       // roll 反馈最大权重
#define BALANCE_ROLL_TERRAIN_LEG_DIFF_LOW 0.01f // 腿长/目标腿长差低阈值,m
#define BALANCE_ROLL_TERRAIN_LEG_DIFF_HIGH 0.05f // 腿长/目标腿长差高阈值,m
#define BALANCE_ROLL_TERRAIN_ROLL_LOW 0.03f // roll 地形权重低阈值,rad
#define BALANCE_ROLL_TERRAIN_ROLL_HIGH 0.15f // roll 地形权重高阈值,rad
#define BALANCE_ROLL_TERRAIN_GAIN 0.50f    // 地形权重增益
#define BALANCE_ROLL_TURN_RELEASE_WZ 1.5f  // target_wz 达到该值附近时降低 roll 硬反馈,rad/s
#define BALANCE_ROLL_TURN_RELEASE_MIN 0.45f // 快速转向时 roll 反馈最低释放权重
#define BALANCE_FINAL_DELTA_F_MAX_RATIO 0.30f // 左右腿力总差动限幅,相对单腿标称支撑力
#define BALANCE_AIRBORNE_FORCE_SCALE 0.25f // 离地侧腿力保护缩放
#define BALANCE_LANDING_ROLL_FB_SCALE 0.30f // 落地保护期间 roll 反馈缩放

// 打滑与接触/离地检测参数，首版只输出状态，不直接参与控制
#define TWO_WHEEL_TRACK_WIDTH_M ((float)TRACK_WIDTH * 0.001f) // 左右轮距,m; TRACK_WIDTH 原单位为 mm
#define TWO_WHEEL_HALF_TRACK_M (0.5f * TWO_WHEEL_TRACK_WIDTH_M) // 左右轮半距,m; 差速轮速预测使用
#define BALANCE_SLIP_W_MIN 2.0f             // 打滑分数归一化最小轮速分母,rad/s; 防低速误判
#define BALANCE_SLIP_LOW 0.15f              // slip_score 低阈值; 低于该值映射为 0 打滑置信度
#define BALANCE_SLIP_HIGH 0.35f             // slip_score 高阈值; 高于该值映射为 1 打滑置信度
#define BALANCE_SLIP_ALPHA 0.10f            // 打滑分数一阶低通 系数; 越大响应越快但更抖
#define BALANCE_SLIP_ON_CONF 0.70f          // 打滑置位置信度阈值,配合确认时间使用
#define BALANCE_SLIP_OFF_CONF 0.30f         // 打滑清除置信度阈值,低于该值持续一段时间后解除
#define BALANCE_SLIP_ON_TIME 0.020f         // 打滑确认时间,s
#define BALANCE_SLIP_OFF_TIME 0.050f        // 打滑解除时间,s; 通常比确认时间长以防抖
#define BALANCE_CONTACT_OFF_RATIO 0.20f     // 支撑力/标称单腿重力低阈值; 低于该值接触置信度为 0
#define BALANCE_CONTACT_ON_RATIO 0.45f      // 支撑力/标称单腿重力高阈值; 高于该值接触置信度为 1
#define BALANCE_CONTACT_ALPHA 0.10f         // 接触置信度一阶低通系数; 越大落地响应越快
#define BALANCE_CONTACT_ON_CONF 0.60f       // 接触置位置信度阈值,配合接触确认时间使用
#define BALANCE_CONTACT_OFF_CONF 0.20f      // 接触清除置信度阈值,低于该值持续一段时间后认为离地
#define BALANCE_CONTACT_ON_TIME 0.010f      // 接触确认时间,s
#define BALANCE_CONTACT_OFF_TIME 0.020f     // 离地确认时间,s
#define BALANCE_CONTACT_LIGHT_UNLOAD_CONF 0.50f // 低于该接触置信度进入轻微卸载状态
#define BALANCE_CONTACT_AIRBORNE_CONF 0.20f // 低于该接触置信度并确认无接触后进入离地状态
#define BALANCE_CONTACT_LANDING_CONF 0.50f  // 离地后高于该接触置信度并确认接触时进入落地状态
#define BALANCE_LANDING_PROTECT_TIME 0.050f // 落地保护窗口时间,s

// 驱动轮质量
#define WHEEL_RADIUS 0.06775f         // 轮子半径
#define WHEEL_MASS (0.718f)

// NMPC 模型中复用的底盘物理参数别名，保证模型和底层配置从 balance.h 同源。
#define BALANCE_NMPC_WHEEL_MASS WHEEL_MASS
#define BALANCE_NMPC_BODY_MASS BODY_MASS
#define BALANCE_NMPC_WHEEL_RADIUS WHEEL_RADIUS
#define BALANCE_NMPC_HALF_TRACK TWO_WHEEL_HALF_TRACK_M

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
#ifndef DEGREE_2_RAD
#define DEGREE_2_RAD 0.017453292519943295f
#endif
#ifndef RAD_2_DEGREE
#define RAD_2_DEGREE 57.295779513082320876f
#endif
#define BALANCE_PI 3.14159265358979323846f

typedef enum
{
    LEG_CONTACT = 0,
    LEG_LIGHT_UNLOAD,
    LEG_AIRBORNE,
    LEG_LANDING,
} LegContactState_e;

typedef struct
{
    float slip_score;       // 轮速残差归一化后的打滑分数
    float slip_conf;        // 打滑置信度,0~1
    uint8_t slip_flag;      // 打滑标志位,带滞回

    float contact_conf;     // 接触置信度,0~1
    uint8_t contact_flag;   // 接触标志位,带滞回
    uint8_t landing_flag;   // 落地瞬间及保护窗口标志位
    LegContactState_e contact_state;

    float slip_on_time;     // 打滑确认计时
    float slip_off_time;    // 打滑解除计时
    float contact_on_time;  // 接触确认计时
    float contact_off_time; // 离地确认计时
    float landing_time;     // 落地保护计时
} LegContactSlipState;

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
    LegContactSlipState contact_slip; // 打滑与接触/离地检测状态

    // pod
    float theta, theta_w;   // 杆和垂直方向的夹角,为控制状态之一
    float leg_len, legd;    // leg_len为腿长，legd为腿长变化率
    float height, height_v; // height为腿高，height_v为腿高变化率
    float F_leg, T_hip;     // 支持力和髋关节扭矩,(N·m)
    float target_len;       // 目标腿高
    LADRC2Instance leg_len_adrc; // 腿长二阶 ADRC,当前用于估计并补偿腿长方向总扰动
    float leg_len_disturbance_acc; // 腿长 LESO 估计的总扰动加速度,m/s^2
    float leg_len_disturbance_force; // 由扰动估计换算出的补偿力,N
    float leg_len_adrc_output; // 完整 LADRC 输出,当前仅用于调试观察
    LESO1Instance wheel_speed_leso; // 单侧轮速一阶 LESO
    float wheel_w_leso;             // LESO 估计轮速,rad/s
    float wheel_speed_disturbance;  // 轮速总扰动估计,rad/s^2

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
    float target_pitch;            // 底盘目标俯仰角，上翘为+，单位：rad; LQR pitch 误差使用
    float target_roll;              // 底盘目标横滚角，右倾为+，单位：rad
    float target_wz;                // 底盘目标角速度，逆时针为+，单位：rad/s
    float offset_angle;          //底盘和归中位置的夹角，底盘相对云台逆时针为+，单位：rad，范围：(-Π,Π]

    // IMU
    float yaw, wz;              // 底盘偏航角度和角速度,逆时针自旋为+，单位：rad,rad/s
    float pitch, pitch_w;       // 底盘俯仰角度和角速度,上翘为+，单位：rad,rad/s
    float roll, roll_w;         // 底盘横滚角度和角速度,右倾为+，单位：rad,rad/s
    LESO2Instance pitch_leso;   // pitch 二阶 LESO
    LESO2Instance roll_leso;    // roll 二阶 LESO
    float pitch_leso_angle;     // LESO 估计 pitch,rad
    float pitch_leso_rate;      // LESO 估计 pitch 角速度,rad/s
    float pitch_disturbance;    // LESO 估计 pitch 总扰动,rad/s^2
    float roll_leso_angle;      // LESO 估计 roll,rad
    float roll_leso_rate;       // LESO 估计 roll 角速度,rad/s
    float roll_disturbance;     // LESO 估计 roll 总扰动,rad/s^2
    float pitch_wheel_torque_ratio; // pitch 控制量分配到轮力矩的比例
    float pitch_joint_torque_ratio; // pitch 控制量分配到髋/关节力矩的比例
    
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
    float F_base_l;        // 左腿基础腿长控制力,N
    float F_base_r;        // 右腿基础腿长控制力,N
    float delta_F_turn_ff; // 转向压腿前馈差动力,N; 左加右减
    float delta_F_roll_fb; // RollTask 反馈差动力,N; 左加右减
    float delta_F_total;   // 最终差动力,N; 左加右减
    float roll_ref_turn;   // 转向允许的 roll 参考,rad
    float roll_err;        // roll_ref_turn - roll,rad
    float roll_weight;     // roll 反馈调度权重
    float target_wz_last;  // 上周期目标 yaw 角速度,rad/s
    float wz_dot;          // 目标 yaw 角加速度估计,rad/s^2
    LESO2Instance roll_task_leso; // RollTask 专用二阶 LESO，估计 roll 角速度与总扰动
    float roll_leso_angle;        // RollTask LESO 估计 roll,rad
    float roll_leso_rate;         // RollTask LESO 估计 roll 角速度,rad/s
    float roll_disturbance;       // RollTask LESO 估计总扰动,rad/s^2
    float roll_disturbance_comp;  // RollTask LESO 换算后的腿力差动补偿,N
    float delta_F_roll_fb_last;   // 上周期 RollTask 实际反馈差动力,N
} LegRollControlState;

typedef struct
{
    LinkNPodParam left;
    LinkNPodParam right;
    ChassisParam chassis;
    LegRollControlState roll_ctrl;
} BalanceState;

/**
 * @brief 重置平衡状态对象和内部算法状态。
 *
 * 初始化左右腿目标、基础支撑力、接触状态、ADRC/LESO 实例和校准标志。
 *
 * @param state 待重置的平衡状态对象。
 */
void BalanceStateReset(BalanceState *state);

/**
 * @brief 更新平衡状态估计。
 *
 * 装配 IMU、控制命令和电机反馈，计算腿部运动学、速度、打滑/接触状态和调试量。
 *
 * @param state 待更新的平衡状态对象。
 * @param imu   当前 IMU 姿态数据。
 * @param cmd   当前底盘控制命令。
 * @param motor 当前平衡底盘相关电机反馈。
 * @param dt    本次状态更新时间，单位 s。
 */
void BalanceStateUpdate(BalanceState *state,
                        const attitude_t *imu,
                        const Chassis_Ctrl_Cmd_s *cmd,
                        const BalanceMotorFeedback *motor,
                        float dt);

/**
 * @brief 更新平衡控制输出。
 *
 * 根据状态估计计算腿长支撑力，并将虚拟腿向力/髋关节力矩投影到关节输出字段。
 *
 * @param state 平衡状态对象。
 * @param dt    本次控制周期，单位 s。
 */
void BalanceControlUpdate(BalanceState *state, float dt);

/**
 * @brief 初始化平衡底盘应用。
 */
void BalanceInit(void);

/**
 * @brief 平衡底盘周期任务。
 */
void BalanceTask(void);

void BalanceNmpcTask(void);

/**
 * @brief 停止四个关节电机和左右驱动轮电机。
 */
void BalanceMotorStopAll(void);

/**
 * @brief 停止四个关节电机，不处理驱动轮。
 */
void BalanceJointStop(void);

/**
 * @brief 使能四个关节电机和左右驱动轮电机。
 */
void BalanceMotorEnableAll(void);
