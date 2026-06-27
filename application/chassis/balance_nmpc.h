#pragma once

#include "balance.h"

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
