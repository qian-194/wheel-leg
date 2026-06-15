/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "HT04.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
#include "ins_task.h"
//平衡用函数
#include "balance.h"
#include "wbr_lqr_calc.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"


#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static referee_info_t* referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap;                                       // 超级电容

static HTMotorInstance *lf, *lb, *rf, *rb, *joint[4]; 
static LKMotorInstance *l_driven, *r_driven, *driven[2];

attitude_t *Chassis_IMU_data; //轮腿底盘加入IMU数据
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ZF_ENCODER_ECD_TO_DEGREE) // 对齐时的角度,0-360
static float joint_pos_min[JOINT_CNT] = {LF_MIN, LB_MIN, RF_MIN, RB_MIN}; // 四个关节电机的最小角度数组,方便传参和调试
static float joint_pos_max[JOINT_CNT] = {LF_MAX, LB_MAX, RF_MAX, RB_MAX}; // 四个关节电机的最大角度数组,方便传参和调试
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘

void ChassisInit()
{
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    //关节电机初始化
    Motor_Init_Config_s joint_conf = {
        // 写一个,剩下的修改方向和id即可
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 2.8f,
                .Kd = 0.0f,
                .Ki = 0.01,
                .DeadBand = 0.0001,
                .Improve = PID_DerivativeFilter | PID_Derivative_On_Measurement,
                .MaxOut = 2,
                .Derivative_LPF_RC = 0.05,
            }, // 仅用于复位腿
            .speed_PID = {
                .Kp = 2.5,
                .Kd = 0,
                .Ki = 0.03,
                .IntegralLimit = 1,
                .DeadBand = 0.0001,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 2,
                .Derivative_LPF_RC = 0.05,
            }, // 仅用于复位腿
        },
        .controller_setting_init_config = {
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .outer_loop_type = OPEN_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
        },
        .motor_type = HT04,
        .motor_mode = TORQUE_MODE,
    };
    // 左关节
    joint_conf.can_init_config.can_handle = &hcan2;
    joint_conf.controller_setting_init_config.motor_reverse_flag = FEEDBACK_DIRECTION_NORMAL; // 左关节不反向

    joint_conf.can_init_config.tx_id = 2;
    joint_conf.can_init_config.rx_id = 2;
    joint[LF] = lf = HTMotorInit(&joint_conf);
    joint_conf.can_init_config.tx_id = 3;
    joint_conf.can_init_config.rx_id = 3;
    joint[LB] = lb = HTMotorInit(&joint_conf);

    // 右关节
    joint_conf.can_init_config.can_handle = &hcan1;
    joint_conf.controller_setting_init_config.motor_reverse_flag = FEEDBACK_DIRECTION_REVERSE; // 右关节反向

    joint_conf.can_init_config.tx_id = 1;
    joint_conf.can_init_config.rx_id = 1;
    joint[RF] = rf = HTMotorInit(&joint_conf);
    joint_conf.can_init_config.tx_id = 4;
    joint_conf.can_init_config.rx_id = 4;
    joint[RB] = rb = HTMotorInit(&joint_conf);

    // 驱动轮电机
    Motor_Init_Config_s driven_conf = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 300,
                .Kd = 15,
                .Ki = 0,
                .DeadBand = 0.01,
                .Improve = PID_DerivativeFilter | PID_Derivative_On_Measurement,
                .MaxOut = 1500,
                .Derivative_LPF_RC = 0.05,
            }, // 仅用于静止
            .speed_PID = {
                .Kp = 500,
                .Kd = 15,
                .Ki = 10,
                .IntegralLimit = 100,
                .DeadBand = 0.0001,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 1500,
                .Derivative_LPF_RC = 0.05,
            }, // 仅用于静止
        },
        .controller_setting_init_config = {
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .outer_loop_type = OPEN_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
        },
        .motor_type = LK9025,
        .motor_mode = TORQUE_MODE,
    };
    // driven_conf.can_init_config.can_handle = &hcan2;
    driven_conf.can_init_config.tx_id = 1;
    driven_conf.can_init_config.rx_id = 1;
    driven_conf.controller_setting_init_config.motor_reverse_flag = FEEDBACK_DIRECTION_REVERSE; // 左轮反向
    driven[LD] = l_driven = LKMotorInit(&driven_conf);
    // driven_conf.can_init_config.can_handle = &hcan2;
    driven_conf.can_init_config.tx_id = 2;
    driven_conf.can_init_config.rx_id = 2;
    driven_conf.controller_setting_init_config.motor_reverse_flag = FEEDBACK_DIRECTION_NORMAL;
    driven[RD] = r_driven = LKMotorInit(&driven_conf);

}


/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
    #ifdef ONE_BOARD
        SubGetMessage(chassis_sub, &chassis_cmd_recv);
    #endif
    #if defined(CHASSIS_BOARD) || defined(CHASSIS_DEBUG)
        chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
    #endif // CHASSIS_BOARD || CHASSIS_DEBUG
    







    chassis_feedback_data.chassis_imu_data = *Chassis_IMU_data;
    // 推送反馈消息
    #ifdef ONE_BOARD
        PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
    #endif
    #if defined(CHASSIS_BOARD) || defined(CHASSIS_DEBUG)
        CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
    #endif // CHASSIS_BOARD || CHASSIS_DEBUG
}