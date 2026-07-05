#include "LK9025.h"
#include "stdlib.h"
#include "general_def.h"
#include "daemon.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

static uint8_t idx;
static LKMotorInstance *lkmotor_instance[LK_MOTOR_MX_CNT] = {NULL};
static CANInstance *sender_instances[LK_MOTOR_MX_CNT] = {NULL}; // 每条CAN总线一个LK多电机发送实例
static uint8_t sender_idx;
// 后续考虑兼容单电机和多电机指令.

#define LK_MULTI_MOTOR_TX_ID 0x280U

static CANInstance *LKMotorGetSender(CANInstance *motor_can)
{
    for (uint8_t i = 0; i < sender_idx; i++)
    {
        if (sender_instances[i]->can_handle == motor_can->can_handle)
        {
            return sender_instances[i];
        }
    }

    if (sender_idx >= LK_MOTOR_MX_CNT)
        while (1)
            ;

#ifdef FDCAN
    motor_can->txconf.Identifier = LK_MULTI_MOTOR_TX_ID;
#else
    motor_can->txconf.StdId = LK_MULTI_MOTOR_TX_ID;
#endif
    sender_instances[sender_idx++] = motor_can;
    return motor_can;
}

/**
 * @brief 电机反馈报文解析
 *
 * @param _instance 发生中断的caninstance
 */
static void LKMotorDecode(CANInstance *_instance)
{
    LKMotorInstance *motor = (LKMotorInstance *)_instance->id; // 通过caninstance保存的father id获取对应的motorinstance
    LKMotor_Measure_t *measure = &motor->measure;
    uint8_t *rx_buff = _instance->rx_buff;

    DaemonReload(motor->daemon); // 喂狗
    measure->feed_dt = DWT_GetDeltaT(&measure->feed_dwt_cnt);

    measure->last_ecd = measure->ecd;
    measure->ecd = (uint16_t)((rx_buff[7] << 8) | rx_buff[6]);

    measure->angle_single_round = ECD_ANGLE_COEF_LK * measure->ecd;

    measure->speed_rads = (1 - SPEED_SMOOTH_COEF) * measure->speed_rads +
                          DEGREE_2_RAD * SPEED_SMOOTH_COEF * (float)((int16_t)(rx_buff[5] << 8 | rx_buff[4]));

    measure->real_current = (1 - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rx_buff[3] << 8 | rx_buff[2]));

    measure->temperature = rx_buff[1];

    if (measure->ecd - measure->last_ecd > 32768)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -32768)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

static void LKMotorLostCallback(void *motor_ptr)
{
#ifdef DISABLE_LOG_SYSTEM
    (void)motor_ptr;
#else
    LKMotorInstance *motor = (LKMotorInstance *)motor_ptr;
    LOGWARNING("[LKMotor] motor lost, id: %d", motor->motor_can_ins->tx_id);
#endif
}

LKMotorInstance *LKMotorInit(Motor_Init_Config_s *config)
{
    LKMotorInstance *motor = (LKMotorInstance *)malloc(sizeof(LKMotorInstance));
    memset(motor, 0, sizeof(LKMotorInstance));

    motor->motor_type = config->motor_type;
    motor->motor_mode = config->motor_mode;
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.id = motor;
    config->can_init_config.can_module_callback = LKMotorDecode;
    config->can_init_config.rx_id = 0x140 + config->can_init_config.tx_id;
    config->can_init_config.tx_id = config->can_init_config.tx_id + 0x280 - 1; // 这样在发送写入buffer的时候更方便,因为下标从0开始,LK多电机发送id为0x280
    motor->motor_can_ins = CANRegister(&config->can_init_config);

    LKMotorGetSender(motor->motor_can_ins);

    LKMotorEnable(motor);
    DWT_GetDeltaT(&motor->measure.feed_dwt_cnt);
    lkmotor_instance[idx++] = motor;

    Daemon_Init_Config_s daemon_config = {
        .callback = LKMotorLostCallback,
        .owner_id = motor,
        .reload_count = 5, // 50ms
    };
    motor->daemon = DaemonRegister(&daemon_config);

    return motor;
}

/* 每条CAN总线使用该总线上的第一个LK电机can instance发送0x280多电机控制帧 */
void LKMotorControl()
{
    float pid_measure, pid_ref;
    int16_t set;
    LKMotorInstance *motor;
    LKMotor_Measure_t *measure;
    Motor_Control_Setting_s *setting;
    CANInstance *sender;
    uint32_t buff_offset;

    for (uint8_t i = 0; i < sender_idx; i++)
    {
        memset(sender_instances[i]->tx_buff, 0, sizeof(sender_instances[i]->tx_buff));
    }

    for (size_t i = 0; i < idx; ++i)
    {
        motor = lkmotor_instance[i];
        measure = &motor->measure;
        setting = &motor->motor_settings;
        pid_ref = motor->pid_ref;

        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
        {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->other_angle_feedback_ptr;
            else
                pid_measure = measure->real_current;
            pid_ref = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor->speed_feedforward_ptr;
        }

        if ((setting->close_loop_type & SPEED_LOOP) && setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))
        {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->other_speed_feedback_ptr;
            else
                pid_measure = measure->speed_rads;
            pid_ref = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
            if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
                pid_ref += *motor->current_feedforward_ptr;
        }

        if (setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor->current_PID, measure->real_current, pid_ref);
        }

        set = pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            set *= -1;
        sender = LKMotorGetSender(motor->motor_can_ins);
        buff_offset = (motor->motor_can_ins->tx_id - LK_MULTI_MOTOR_TX_ID) * 2U;
        if (buff_offset >= sizeof(sender->tx_buff))
        {
            continue;
        }

        if (motor->stop_flag == MOTOR_STOP)
        { // 若该电机处于停止状态,直接将发送buff置零
            memset(sender->tx_buff + buff_offset, 0, sizeof(uint16_t));
        }
        else
        {
            memcpy(sender->tx_buff + buff_offset, &set, sizeof(uint16_t));
        }
    }

    for (uint8_t i = 0; i < sender_idx; i++)
    {
        CANTransmit(sender_instances[i], 0.2);
    }
}

void LKMotorStop(LKMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void LKMotorEnable(LKMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void LKMotorSetRef(LKMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

uint8_t LKMotorIsOnline(LKMotorInstance *motor)
{
    return DaemonIsOnline(motor->daemon);
}
