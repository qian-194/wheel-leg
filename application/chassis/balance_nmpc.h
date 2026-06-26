#pragma once

#include "balance.h"

/*
 * NMPC 只作为 200Hz 上层参考规划器使用，不直接输出电机力矩。
 * 关闭该宏后，BalanceTask 会继续使用原来的目标值和底层 LQR/VMC 控制。
 */
#ifndef BALANCE_NMPC_ENABLE
#define BALANCE_NMPC_ENABLE 1
#endif

/* 超过该耗时认为本周期 NMPC 不可信，外层会进入 fallback 并保留底层控制。 */
#define BALANCE_NMPC_TIMEOUT_MS 2.0f
/* BalanceTask 使用目标前会检查时间戳，防止 200Hz 任务卡住后继续吃旧参考。 */
#define BALANCE_NMPC_TARGET_STALE_MS 15.0f
#define BALANCE_NMPC_MAX_FAIL_COUNT 3u

/* 上层参考限幅。底层 LQR/VMC 仍会做自己的力矩/关节保护。 */
#define BALANCE_NMPC_V_LIMIT 3.0f
#define BALANCE_NMPC_WZ_LIMIT 6.0f
#define BALANCE_NMPC_PITCH_LIMIT 0.25f
#define BALANCE_NMPC_ROLL_LIMIT 0.20f

/* 200Hz NMPC 步长，预测域 8 步约等于 40ms。 */
#define BALANCE_NMPC_DT 0.005f
#define BALANCE_NMPC_HORIZON 8u

/* 参考变化率限制，避免上层目标阶跃把底层 LQR 推到饱和。 */
#define BALANCE_NMPC_V_RATE_LIMIT 1.2f
#define BALANCE_NMPC_WZ_RATE_LIMIT 3.5f
#define BALANCE_NMPC_LEG_RATE_LIMIT 0.45f
#define BALANCE_NMPC_PITCH_FROM_V_GAIN 0.035f
#define BALANCE_NMPC_PITCH_RATE_LIMIT 0.60f

typedef enum
{
    BALANCE_NMPC_DISABLED = 0,
    BALANCE_NMPC_WARMUP,
    BALANCE_NMPC_ACTIVE,
    BALANCE_NMPC_FALLBACK,
} BalanceNmpcMode_e;

typedef struct
{
    /* NMPC 输出给底层平衡控制的下一周期参考，不包含任何电机力矩。 */
    float target_v;
    float target_wz;
    float target_pitch;
    float target_roll;
    float target_leg_len_l;
    float target_leg_len_r;
    float timestamp_ms;
    uint8_t valid;
} BalanceNmpcTarget;

typedef struct
{
    /* 调试/遥测用状态，用于判断 NMPC 是否正在生效、是否超时或失效。 */
    BalanceNmpcMode_e mode;
    float last_elapsed_ms;
    float last_update_ms;
    uint32_t update_count;
    uint32_t timeout_count;
    uint32_t invalid_count;
    uint32_t fallback_count;
    uint8_t active;
} BalanceNmpcStatus;

void BalanceNmpcTaskUpdate(const BalanceState *state, const Chassis_Ctrl_Cmd_s *cmd);
void BalanceNmpcApplyTarget(BalanceState *state);
void BalanceNmpcGetStatus(BalanceNmpcStatus *status);
