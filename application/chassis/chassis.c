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
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

attitude_t *Chassis_IMU_data; //轮腿底盘加入IMU数据


/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘

void ChassisInit()
{
    #ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
        Chassis_IMU_data = INS_Init(); // 底盘IMU初始化
        chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
        chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    #endif // ONE_BOARD

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