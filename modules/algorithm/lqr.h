#ifndef __LQR_H__
#define __LQR_H__

#include "stdint.h"
#include "string.h"
#include "arm_math.h"
#include "user_lib.h"

#ifdef __cplusplus
extern "C" {
#endif

// 默认计算 u = Kx，不自动加负号，需导出已含符号的 K 矩阵；
// 若开启 LQR_NEGATE_GAIN 特性，则模块内部自动取负，计算标准负反馈 u = -Kx。
typedef enum{
    LQR_MODE_REGULATION = 0, //U = Kx
    LQR_MODE_TRACK_REFERENCE, //U = K(x - x_ref)
}LQR_Mode_e;

typedef enum
{
    LQR_FEATURE_NONE = 0x00,
    LQR_USE_FEEDFORWARD = 0x01,
    LQR_USE_OUTPUT_LIMIT = 0x02,
    LQR_NEGATE_GAIN = 0x04, // 模块内部自动取负，实现标准负反馈 u = -Kx
} LQR_Feature_e;

typedef struct
{
    float *State;
    float *Reference;
    float *StateError;
    float *Control;
    float *Feedforward;
    float *GainMatrix;
} LQR_Workspace_s;

typedef struct lqr_t
{
    uint8_t StateSize;
    uint8_t ControlSize;

    LQR_Mode_e Mode;
    uint8_t Features;

    float *State;
    float *Reference;
    float *StateError;
    float *Control;
    float *Feedforward;
    float *GainMatrix;
    const float *OutputMin;
    const float *OutputMax;

    mat K;
    mat X;
    mat U;

    int8_t MatStatus;
    uint32_t UpdateCount;

    void (*PreProcess)(struct lqr_t *lqr);
    void (*PostProcess)(struct lqr_t *lqr);
} LQRInstance;

typedef struct
{
    uint8_t StateSize;
    uint8_t ControlSize;

    LQR_Mode_e Mode;
    uint8_t Features;

    const float *InitialGain;
    const float *InitialReference;
    const float *InitialFeedforward;
    const float *OutputMin;
    const float *OutputMax;

    LQR_Workspace_s *Workspace;

    void (*PreProcess)(LQRInstance *lqr);
    void (*PostProcess)(LQRInstance *lqr);
} LQR_Init_Config_s;

typedef enum
{
    LQR_OK = 0,
    LQR_ERR_NULL = -1,
    LQR_ERR_SIZE = -2,
} LQR_Status_e;

int8_t LQRInit(LQRInstance *lqr, const LQR_Init_Config_s *config);
float *LQRCalculate(LQRInstance *lqr, const float *state);
void LQRSetGain(LQRInstance *lqr, const float *gain);
void LQRSetReference(LQRInstance *lqr, const float *reference);
void LQRSetFeedforward(LQRInstance *lqr, const float *feedforward);
void LQRReset(LQRInstance *lqr);

#ifdef __cplusplus
}
#endif

#endif // __LQR_H__
