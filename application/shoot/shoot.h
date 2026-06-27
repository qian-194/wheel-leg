#ifndef SHOOT_H
#define SHOOT_H

/**
 * @brief 发射初始化,会被RobotInit()调用
 * 
 */
void ShootInit();

/**
 * @brief 发射任务
 * 
 */
void ShootTask();

/**
 * @brief Stop all shoot motors.
 */
void ShootStopAll(void);

#endif // SHOOT_H
