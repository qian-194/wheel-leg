#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"
#include "ins_task.h"
#include "buzzer/buzzer_app.h"

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD) 
#include "balance.h"
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"
#endif

static void RobotINSOfflineCallback(void)
{
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    BalanceMotorStopAll();
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    GimbalStopAll();
    ShootStopAll();
#endif

    BuzzerAppPlay(BUZZER_APP_SOUND_HIGH_LONG);
}


void RobotInit()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    
    BSPInit();

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDInit();
    GimbalInit();
    ShootInit();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    BalanceInit();
#endif

    INS_RegisterOfflineCallback(RobotINSOfflineCallback);

    OSTaskInit(); // 创建基础任务

    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
    if (INS_IsOffline())
    {
        RobotINSOfflineCallback();
        return;
    }

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDTask();
    GimbalTask();
    ShootTask();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    BalanceTask();
#endif


}
