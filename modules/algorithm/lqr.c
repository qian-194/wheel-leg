#include "lqr.h"

/**
 * @brief   绑定应用层提供的静态工作区。
 *          本模块不进行任何动态内存分配，所有缓冲必须由 app 层静态提供。
 * @return  LQR_OK 成功；LQR_ERR_NULL 工作区或其中任一缓冲为空。
 */
static int8_t LQRBindWorkspace(LQRInstance *lqr, const LQR_Init_Config_s *config)
{
    const LQR_Workspace_s *workspace = config->Workspace;

    if (workspace == NULL)
    {
        return LQR_ERR_NULL;
    }

    lqr->State = workspace->State;
    lqr->Reference = workspace->Reference;
    lqr->StateError = workspace->StateError;
    lqr->Control = workspace->Control;
    lqr->Feedforward = workspace->Feedforward;
    lqr->GainMatrix = workspace->GainMatrix;

    if (lqr->State == NULL ||
        lqr->Reference == NULL ||
        lqr->StateError == NULL ||
        lqr->Control == NULL ||
        lqr->Feedforward == NULL ||
        lqr->GainMatrix == NULL)
    {
        return LQR_ERR_NULL;
    }

    return LQR_OK;
}

static void LQRCopyFloat(float *dst, const float *src, uint16_t length)
{
    if (dst == NULL || src == NULL || length == 0)
    {
        return;
    }

    memcpy(dst, src, sizeof(float) * length);
}

int8_t LQRInit(LQRInstance *lqr, const LQR_Init_Config_s *config)
{
    int8_t status;

    if (lqr == NULL || config == NULL)
    {
        return LQR_ERR_NULL;
    }

    if (config->StateSize == 0 || config->ControlSize == 0)
    {
        return LQR_ERR_SIZE;
    }

    memset(lqr, 0, sizeof(LQRInstance));

    lqr->StateSize = config->StateSize;
    lqr->ControlSize = config->ControlSize;
    lqr->Mode = config->Mode;
    lqr->Features = config->Features;
    lqr->OutputMin = config->OutputMin;
    lqr->OutputMax = config->OutputMax;
    lqr->PreProcess = config->PreProcess;
    lqr->PostProcess = config->PostProcess;

    status = LQRBindWorkspace(lqr, config);
    if (status != LQR_OK)
    {
        return status;
    }

    LQRCopyFloat(lqr->GainMatrix, config->InitialGain,
                 (uint16_t)lqr->StateSize * lqr->ControlSize);
    LQRCopyFloat(lqr->Reference, config->InitialReference, lqr->StateSize);
    LQRCopyFloat(lqr->Feedforward, config->InitialFeedforward, lqr->ControlSize);

    arm_mat_init_f32(&lqr->K, lqr->ControlSize, lqr->StateSize, lqr->GainMatrix);
    arm_mat_init_f32(&lqr->X, lqr->StateSize, 1, lqr->StateError);
    arm_mat_init_f32(&lqr->U, lqr->ControlSize, 1, lqr->Control);

    return LQR_OK;
}

void LQRSetGain(LQRInstance *lqr, const float *gain)
{
    if (lqr == NULL || gain == NULL || lqr->GainMatrix == NULL)
    {
        return;
    }

    memcpy(lqr->GainMatrix, gain, sizeof(float) * lqr->StateSize * lqr->ControlSize);
}

void LQRSetReference(LQRInstance *lqr, const float *reference)
{
    if (lqr == NULL || reference == NULL || lqr->Reference == NULL)
    {
        return;
    }

    memcpy(lqr->Reference, reference, sizeof(float) * lqr->StateSize);
}

void LQRSetFeedforward(LQRInstance *lqr, const float *feedforward)
{
    if (lqr == NULL || feedforward == NULL || lqr->Feedforward == NULL)
    {
        return;
    }

    memcpy(lqr->Feedforward, feedforward, sizeof(float) * lqr->ControlSize);
}

float *LQRCalculate(LQRInstance *lqr, const float *state)
{
    if (lqr == NULL || state == NULL || lqr->State == NULL)
    {
        return NULL;
    }

    memcpy(lqr->State, state, sizeof(float) * lqr->StateSize);

    if (lqr->PreProcess != NULL)
    {
        lqr->PreProcess(lqr);
    }

    if (lqr->Mode == LQR_MODE_TRACK_REFERENCE)
    {
        for (uint8_t i = 0; i < lqr->StateSize; i++)
        {
            lqr->StateError[i] = lqr->State[i] - lqr->Reference[i];
        }
    }
    else
    {
        memcpy(lqr->StateError, lqr->State, sizeof(float) * lqr->StateSize);
    }

    lqr->MatStatus = arm_mat_mult_f32(&lqr->K, &lqr->X, &lqr->U);

    if (lqr->MatStatus == ARM_MATH_SUCCESS)
    {
        if (lqr->Features & LQR_NEGATE_GAIN)
        {
            for (uint8_t i = 0; i < lqr->ControlSize; i++)
            {
                lqr->Control[i] = -lqr->Control[i];
            }
        }

        if (lqr->Features & LQR_USE_FEEDFORWARD)
        {
            for (uint8_t i = 0; i < lqr->ControlSize; i++)
            {
                lqr->Control[i] += lqr->Feedforward[i];
            }
        }

        if ((lqr->Features & LQR_USE_OUTPUT_LIMIT) &&
            lqr->OutputMin != NULL &&
            lqr->OutputMax != NULL)
        {
            for (uint8_t i = 0; i < lqr->ControlSize; i++)
            {
                VAL_LIMIT(lqr->Control[i], lqr->OutputMin[i], lqr->OutputMax[i]);
            }
        }
    }
    else
    {
        memset(lqr->Control, 0, sizeof(float) * lqr->ControlSize);
    }

    if (lqr->PostProcess != NULL)
    {
        lqr->PostProcess(lqr);
    }

    lqr->UpdateCount++;

    return lqr->Control;
}

void LQRReset(LQRInstance *lqr)
{
    if (lqr == NULL)
    {
        return;
    }

    memset(lqr->State, 0, sizeof(float) * lqr->StateSize);
    memset(lqr->Reference, 0, sizeof(float) * lqr->StateSize);
    memset(lqr->StateError, 0, sizeof(float) * lqr->StateSize);
    memset(lqr->Control, 0, sizeof(float) * lqr->ControlSize);
    memset(lqr->Feedforward, 0, sizeof(float) * lqr->ControlSize);

    lqr->MatStatus = ARM_MATH_SUCCESS;
    lqr->UpdateCount = 0;
}