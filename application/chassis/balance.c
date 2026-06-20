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

#include <math.h>
#include <string.h>

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

static attitude_t *Chassis_IMU_data; //轮腿底盘加入IMU数据
static BalanceState balance_state;
static uint32_t balance_dwt_cnt;
static float del_t;
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ZF_ENCODER_ECD_TO_DEGREE) // 对齐时的角度,0-360
static float joint_pos_min[JOINT_CNT] = {LF_MIN, LB_MIN, RF_MIN, RB_MIN}; // 四个关节电机的最小角度数组,方便传参和调试
static float joint_pos_max[JOINT_CNT] = {LF_MAX, LB_MAX, RF_MAX, RB_MAX}; // 四个关节电机的最大角度数组,方便传参和调试
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘

#define TWO_WHEEL_OPEN_LOOP_GAIN 200.0f
#define TWO_WHEEL_OPEN_LOOP_MAX 600.0f
#define JOINT_CALI_SPEED_TH 0.05f
#define JOINT_CALI_TORQUE_TH 1.5f
#define JOINT_CALI_CONFIRM_CNT 100u
#define JOINT_CALI_CNT_MAX 250u

static uint8_t JointCalibEncoder(void);
static void BalanceTwoWheelOpenLoopControl(const Chassis_Ctrl_Cmd_s *cmd);

/**
 * @brief 初始化平衡底盘应用。
 *
 * 完成底盘 IMU、消息发布订阅、四个关节电机、左右驱动轮电机和
 * BalanceState 的初始化，并启动 DWT 周期计时基准。
 */
void BalanceInit()
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
        .motor_mode = MIT_MODE,
    };
    // 左关节
    joint_conf.can_init_config.can_handle = &hcan1;
    joint_conf.controller_setting_init_config.motor_reverse_flag = FEEDBACK_DIRECTION_NORMAL; // 左关节不反向

    joint_conf.can_init_config.tx_id = 2;
    joint_conf.can_init_config.rx_id = 2;
    joint[LF] = lf = HTMotorInit(&joint_conf);
    joint_conf.can_init_config.tx_id = 3;
    joint_conf.can_init_config.rx_id = 3;
    joint[LB] = lb = HTMotorInit(&joint_conf);

    // 右关节
    joint_conf.can_init_config.can_handle = &hcan2;
    joint_conf.controller_setting_init_config.motor_reverse_flag = FEEDBACK_DIRECTION_REVERSE; // 右关节反向

    joint_conf.can_init_config.tx_id = 1;
    joint_conf.can_init_config.rx_id = 1;
    joint[RF] = rf = HTMotorInit(&joint_conf);
    joint_conf.can_init_config.tx_id = 4;
    joint_conf.can_init_config.rx_id = 4;
    joint[RB] = rb = HTMotorInit(&joint_conf);

    // 驱动轮电机
    Motor_Init_Config_s driven_conf = {
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
    // driven_l_conf.can_init_config.can_handle = &hcan1;
    driven_conf.can_init_config.can_handle = &hcan1,
    driven_conf.can_init_config.tx_id = 1;
    driven_conf.can_init_config.rx_id = 1;
    driven_conf.controller_setting_init_config.motor_reverse_flag = FEEDBACK_DIRECTION_REVERSE; // 左轮反向
    driven[LD] = l_driven = LKMotorInit(&driven_conf);
    // driven_r_conf.can_init_config.can_handle = &hcan2;
    driven_conf.can_init_config.can_handle = &hcan2,
    driven_conf.can_init_config.tx_id = 2;
    driven_conf.can_init_config.rx_id = 2;
    driven_conf.controller_setting_init_config.motor_reverse_flag = FEEDBACK_DIRECTION_NORMAL;
    driven[RD] = r_driven = LKMotorInit(&driven_conf);

    BalanceStateReset(&balance_state);
    DWT_GetDeltaT(&balance_dwt_cnt);
}


/**
 * @brief 平衡底盘控制核心周期任务。
 *
 * 每周期读取底盘命令、计算真实周期、执行上电关节校准，并根据底盘模式选择
 * 急停、双轮开环或站立平衡控制。任务末尾会回传底盘反馈数据。
 */
void BalanceTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
    #ifdef ONE_BOARD
        SubGetMessage(chassis_sub, &chassis_cmd_recv);
    #endif
    #if defined(CHASSIS_BOARD) || defined(CHASSIS_DEBUG)
        chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
    #endif // CHASSIS_BOARD || CHASSIS_DEBUG

    del_t = DWT_GetDeltaT(&balance_dwt_cnt);
    BalanceMotorFeedback motor_feedback = {
        .lf = lf,
        .lb = lb,
        .rf = rf,
        .rb = rb,
        .l_driven = l_driven,
        .r_driven = r_driven,
    };
    //上电初始化确定位置
    if (balance_state.chassis.cali_flag)
    {
        LKMotorSetRef(l_driven, 0.0f);
        LKMotorSetRef(r_driven, 0.0f);
        LKMotorStop(l_driven);
        LKMotorStop(r_driven);

        if (JointCalibEncoder() == 0u)
            return;

        balance_state.chassis.cali_flag = 0u;
    }
    //chassismode的处理,根据不同模式进行不同的控制
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_ZERO_FORCE:
        BalanceMotorStopAll();
        break;

    case CHASSIS_JOINT_ZERO_FORCE:
        BalanceTwoWheelOpenLoopControl(&chassis_cmd_recv);
        break;

    case CHASSIS_STAND:
    default:
        BalanceMotorEnableAll();

        BalanceStateUpdate(&balance_state,
                           Chassis_IMU_data,
                           &chassis_cmd_recv,
                           &motor_feedback,
                           del_t);
        BalanceControlUpdate(&balance_state, del_t);
        break;
    }

    chassis_feedback_data.chassis_imu_data = *Chassis_IMU_data;
    // 推送反馈消息
    #ifdef ONE_BOARD
        PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
    #endif
    #if defined(CHASSIS_BOARD) || defined(CHASSIS_DEBUG)
        CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
    #endif // CHASSIS_BOARD || CHASSIS_DEBUG
}


/**
 * @brief 停止所有平衡底盘电机。
 *
 * 先将四个关节 MIT 参考和左右驱动轮参考清零，再分别停止关节电机与驱动轮电机。
 * 用于零力模式和安全急停。
 */
void BalanceMotorStopAll(void)
{
    HTMotorSetMITRef(lf, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    HTMotorSetMITRef(lb, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    HTMotorSetMITRef(rf, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    HTMotorSetMITRef(rb, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    HTMotorStop(lf);
    HTMotorStop(lb);
    HTMotorStop(rf);
    HTMotorStop(rb);

    LKMotorSetRef(l_driven, 0.0f);
    LKMotorSetRef(r_driven, 0.0f);

    LKMotorStop(l_driven);
    LKMotorStop(r_driven);
}

/**
 * @brief 停止四个关节电机。
 *
 * 清零四个 HT 关节电机的 MIT 参考并停止关节电机，不影响左右驱动轮状态。
 * 双轮开环调试模式下用于释放腿部关节输出。
 */
void BalanceJointStop(void)
{
    HTMotorSetMITRef(lf, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    HTMotorSetMITRef(lb, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    HTMotorSetMITRef(rf, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    HTMotorSetMITRef(rb, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    HTMotorStop(lf);
    HTMotorStop(lb);
    HTMotorStop(rf);
    HTMotorStop(rb);
}

/**
 * @brief 使能所有平衡底盘电机。
 *
 * 使能四个 HT 关节电机和左右 LK 驱动轮电机，通常在进入站立控制前调用。
 */
void BalanceMotorEnableAll(void)
{
    HTMotorEnable(lf);
    HTMotorEnable(lb);
    HTMotorEnable(rf);
    HTMotorEnable(rb);

    LKMotorEnable(l_driven);
    LKMotorEnable(r_driven);
}

/**
 * @brief 执行关节电机上电编码器校准。
 *
 * HT 关节电机掉电后会丢失绝对位置。校准流程将各关节缓慢推向短腿机械限位，
 * 当检测到速度足够低且堵转电流足够大并持续确认后，将当前位置标定为编码器零点。
 *
 * @return uint8_t 1 表示四个关节均完成校准；0 表示仍在校准过程中。
 */
static uint8_t JointCalibEncoder(void)
{
    static uint8_t cali_flag[JOINT_CNT] = {0};
    static uint8_t cali_cnt[JOINT_CNT] = {0};

    if (cali_flag[LF] == 0u && cali_flag[LB] == 0u &&
        cali_flag[RF] == 0u && cali_flag[RB] == 0u)
    {
        for (uint8_t i = 0; i < JOINT_CNT; i++)
        {
            HTMotorEnable(joint[i]);
            HTMotorOuterLoop(joint[i], OPEN_LOOP);
        }
    }

    for (uint8_t i = 0; i < JOINT_CNT; i++)
    {
        if (!cali_flag[i])
        {
            if (fabsf(joint[i]->measure.speed_rads) < JOINT_CALI_SPEED_TH &&
                fabsf(joint[i]->measure.real_current) > JOINT_CALI_TORQUE_TH)
            {
                if (cali_cnt[i] < JOINT_CALI_CNT_MAX)
                    cali_cnt[i]++;

                if (cali_cnt[i] >= JOINT_CALI_CONFIRM_CNT)
                {
                    HTMotorCalibEncoder(joint[i]);
                    HTMotorOuterLoop(joint[i], OPEN_LOOP);
                    HTMotorSetRef(joint[i], 0.0f);
                    HTMotorSetMITRef(joint[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    HTMotorStop(joint[i]);
                    cali_flag[i] = 1u;
                    cali_cnt[i] = 0u;
                }
            }
            else
            {
                cali_cnt[i] = 0u;
            }
        }
    }

    if (!cali_flag[LF]) HTMotorSetRef(lf,  1.2f);
    if (!cali_flag[LB]) HTMotorSetRef(lb, -1.2f);
    if (!cali_flag[RF]) HTMotorSetRef(rf,  1.2f);
    if (!cali_flag[RB]) HTMotorSetRef(rb, -1.2f);

    for (uint8_t i = 0; i < JOINT_CNT; i++)
    {
        if (!cali_flag[i])
            return 0u;
    }

    memset(cali_flag, 0, sizeof(cali_flag));
    memset(cali_cnt, 0, sizeof(cali_cnt));
    return 1u;
}

/**
 * @brief 双轮开环调试控制。
 *
 * 根据底盘期望线速度和角速度计算左右轮开环参考，停止关节电机后将左右驱动轮
 * 切到开环输出。该模式用于关节零力状态下验证驱动轮方向和基础运动。
 *
 * @param cmd 当前底盘控制命令。
 */
static void BalanceTwoWheelOpenLoopControl(const Chassis_Ctrl_Cmd_s *cmd)
{
    float left_v = cmd->vx - cmd->wz * TWO_WHEEL_TRACK_WIDTH_M * 0.5f;
    float right_v = cmd->vx + cmd->wz * TWO_WHEEL_TRACK_WIDTH_M * 0.5f;

    float left_ref = left_v * TWO_WHEEL_OPEN_LOOP_GAIN;
    float right_ref = right_v * TWO_WHEEL_OPEN_LOOP_GAIN;

    LIMIT_MIN_MAX(left_ref, -TWO_WHEEL_OPEN_LOOP_MAX, TWO_WHEEL_OPEN_LOOP_MAX);
    LIMIT_MIN_MAX(right_ref, -TWO_WHEEL_OPEN_LOOP_MAX, TWO_WHEEL_OPEN_LOOP_MAX);

    BalanceJointStop();

    LKMotorEnable(l_driven);
    LKMotorEnable(r_driven);

    l_driven->motor_settings.outer_loop_type = OPEN_LOOP;
    r_driven->motor_settings.outer_loop_type = OPEN_LOOP;

    LKMotorSetRef(l_driven, left_ref);
    LKMotorSetRef(r_driven, right_ref);
}
