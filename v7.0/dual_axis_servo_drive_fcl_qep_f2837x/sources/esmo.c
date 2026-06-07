//#############################################################################
//
// FILE:    esmo.c
//
// TITLE:   Enhanced sliding mode observer for dual FCL project
//
//#############################################################################

#include "esmo.h"

#ifndef ESMO_MIN_BEMF_MAG
#define ESMO_MIN_BEMF_MAG    (1.0e-6f)
#endif

#pragma CODE_SECTION(ESMO_run, ".TI.ramfunc");

ESMO_Handle ESMO_init(void *pMemory, const size_t numBytes)
{
    ESMO_Handle handle;

    if(numBytes < sizeof(ESMO_Obj))
    {
        return((ESMO_Handle)0);
    }

    handle = (ESMO_Handle)pMemory;

    return(handle);
}

void ESMO_resetParams(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->pll_ui = 0.0f;
    obj->pll_Out = 0.0f;

    obj->speedEst = 0.0f;
    obj->speedFlt = 0.0f;
    obj->speedRef = 0.0f;

    obj->theta = 0.0f;
    obj->thetaEst = 0.0f;
    obj->thetaElec_rad = 0.0f;
    obj->thetaErr = 0.0f;

    obj->EstIalpha = 0.0f;
    obj->EstIbeta = 0.0f;
    obj->Zalpha = 0.0f;
    obj->Zbeta = 0.0f;
    obj->Ealpha = 0.0f;
    obj->Ebeta = 0.0f;

    obj->Kslide = obj->KslideMin;
    obj->pll_Kp = obj->pll_KpMin;

    return;
}

void ESMO_setPLLParams(ESMO_Handle handle, const float32_t pll_KpMax,
                       const float32_t pll_KpMin, const float32_t pll_KpSF)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->pll_KpMax = pll_KpMax;
    obj->pll_KpMin = pll_KpMin;
    obj->pll_KpSF = pll_KpSF;

    return;
}

void ESMO_setKslideParams(ESMO_Handle handle,
                          const float32_t KslideMax,
                          const float32_t KslideMin)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->KslideMax = KslideMax;
    obj->KslideMin = KslideMin;

    return;
}

void ESMO_setParams(ESMO_Handle handle, const ESMO_Params *pParams)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    float32_t motor_Rs;
    float32_t baseVI;

    obj->scaleFreq_Hz = pParams->maxFrequency_Hz;

    obj->speed_sf = 1.0f / pParams->maxFrequency_Hz;
    obj->voltage_sf = 1.0f / (3.0f * pParams->voltageBase_V);
    obj->current_sf = 1.0f / pParams->currentBase_A;

    obj->Ts = pParams->ctrlPeriod_sec;
    obj->base_wTs = obj->Ts * obj->scaleFreq_Hz;
    obj->thetaDelta = obj->scaleFreq_Hz * obj->Ts;
    obj->thetaErrSF = MATH_ONE_OVER_TWO_PI;

    obj->Kslide = obj->KslideMin;
    obj->pll_Kp = obj->pll_KpMin;

    obj->pll_ui = 0.0f;
    obj->pll_Out = 0.0f;
    obj->pll_Ki = 0.0f;
    obj->pll_Umax = 1.0f;
    obj->pll_Umin = -1.0f;

    obj->EstIalpha = 0.0f;
    obj->EstIbeta = 0.0f;

    obj->Zalpha = 0.0f;
    obj->Zbeta = 0.0f;

    obj->Ealpha = 0.0f;
    obj->Ebeta = 0.0f;

    motor_Rs = (pParams->motor_Rs_d_Ohm + pParams->motor_Rs_q_Ohm) * 0.5f;
    baseVI = (pParams->voltageBase_V / pParams->currentBase_A) * 2.0f;

    obj->Fdsmopos = expf(-(motor_Rs / pParams->motor_Ls_d_H) * obj->Ts);
    obj->Fqsmopos = expf(-(motor_Rs / pParams->motor_Ls_q_H) * obj->Ts);
    obj->Gdsmopos = (baseVI / motor_Rs) * (1.0f - obj->Fdsmopos);
    obj->Gqsmopos = (baseVI / motor_Rs) * (1.0f - obj->Fqsmopos);

    ESMO_updateFilterParams(handle);

    return;
}

void ESMO_updateFilterParams(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    float32_t tempVarLpf = obj->lpfFc_Hz * MATH_TWO_PI * obj->Ts;

    obj->lpf_a1 = 1.0f / (1.0f + tempVarLpf);
    obj->lpf_b0 = 1.0f - obj->lpf_a1;

    obj->Kslf = obj->filterFc_Hz * MATH_TWO_PI * obj->base_wTs;

    return;
}

void ESMO_updatePLLParams(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    float32_t speedRefAbs = fabsf(obj->speedRef);

    obj->pll_Kp = obj->pll_KpMin + obj->pll_KpSF * speedRefAbs;
    obj->pll_Kp = __fsat(obj->pll_Kp, obj->pll_KpMax, obj->pll_KpMin);

    return;
}

void ESMO_run(ESMO_Handle handle,
              float32_t Vdcbus,
              MATH_vec3 *pVabc_pu,
              MATH_vec2 *pIabVec)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    float32_t Vtemp;
    float32_t VphaseA;
    float32_t VphaseB;
    float32_t ValphaError;
    float32_t VbetaError;
    float32_t IalphaError;
    float32_t IbetaError;
    float32_t thetaOffset;
    float32_t pllSine;
    float32_t pllCosine;
    float32_t thetaErrSF;

    Vtemp = Vdcbus * obj->voltage_sf;

    VphaseA = Vtemp *
            (pVabc_pu->value[0] * 2.0f - pVabc_pu->value[1] -
             pVabc_pu->value[2]);

    VphaseB = Vtemp *
            (pVabc_pu->value[1] * 2.0f - pVabc_pu->value[0] -
             pVabc_pu->value[2]);

    obj->Valpha = VphaseA;
    obj->Vbeta = (VphaseA + VphaseB * 2.0f) * MATH_ONE_OVER_SQRT_THREE;

    ValphaError = obj->Valpha - obj->Ealpha - obj->Zalpha;
    VbetaError = obj->Vbeta - obj->Ebeta - obj->Zbeta;

    obj->EstIalpha = obj->Gdsmopos * ValphaError +
                     obj->Fdsmopos * obj->EstIalpha;
    obj->EstIbeta = obj->Gqsmopos * VbetaError +
                    obj->Fqsmopos * obj->EstIbeta;

    IalphaError = obj->EstIalpha - pIabVec->value[0] * obj->current_sf;
    IbetaError = obj->EstIbeta - pIabVec->value[1] * obj->current_sf;

    obj->Zalpha = __fsat(IalphaError, obj->E0, -obj->E0) * obj->Kslide;
    obj->Zbeta = __fsat(IbetaError, obj->E0, -obj->E0) * obj->Kslide;

    obj->Ealpha = obj->Ealpha + obj->Kslf * (obj->Zalpha - obj->Ealpha);
    obj->Ebeta = obj->Ebeta + obj->Kslf * (obj->Zbeta - obj->Ebeta);

    obj->thetaOffset_rad = obj->speedRef * obj->offsetSF + 0.005f;
    obj->thetaElec_rad = atan2f(obj->Ealpha, obj->Ebeta) +
                         obj->thetaOffset_rad;

    if(obj->thetaElec_rad > MATH_PI)
    {
        obj->thetaElec_rad -= MATH_TWO_PI;
    }
    else if(obj->thetaElec_rad < -MATH_PI)
    {
        obj->thetaElec_rad += MATH_TWO_PI;
    }

    thetaOffset = __atan2puf32((obj->speedRef * obj->offsetSF), obj->Kslf);
    obj->thetaPll = obj->theta - thetaOffset;

    pllSine = __sinpuf32(obj->thetaPll);
    pllCosine = __cospuf32(obj->thetaPll);

    obj->Ed = obj->Ealpha * pllCosine + obj->Ebeta * pllSine;
    obj->Eq = obj->Ebeta * pllCosine - obj->Ealpha * pllSine;
    obj->Eq_mag = sqrtf(obj->Ealpha * obj->Ealpha + obj->Ebeta * obj->Ebeta);

    thetaErrSF = obj->thetaErrSF;

    if(obj->Eq >= 0.0f)
    {
        thetaErrSF = -obj->thetaErrSF;
    }

    if(obj->Eq_mag > ESMO_MIN_BEMF_MAG)
    {
        obj->thetaErr = obj->Ed * thetaErrSF / obj->Eq_mag;
    }
    else
    {
        obj->thetaErr = 0.0f;
    }

    obj->pll_ui = (obj->pll_Ki * obj->thetaErr) + obj->pll_ui;

    obj->pll_Out = __fsat((obj->pll_Kp * obj->thetaErr + obj->pll_ui),
                          obj->pll_Umax, obj->pll_Umin);

    obj->speedEst = (obj->pll_Out + obj->speedEst) * 0.5f;
    obj->speedFlt = obj->lpf_b0 * obj->pll_Out + obj->lpf_a1 * obj->speedFlt;

    obj->theta = obj->theta + obj->speedFlt * obj->thetaDelta;

    if(obj->theta > 1.0f)
    {
        obj->theta -= 1.0f;
    }
    else if(obj->theta < -1.0f)
    {
        obj->theta += 1.0f;
    }

    obj->thetaEst = obj->theta * MATH_TWO_PI;

    if(obj->thetaEst > MATH_PI)
    {
        obj->thetaEst -= MATH_TWO_PI;
    }
    else if(obj->thetaEst < -MATH_PI)
    {
        obj->thetaEst += MATH_TWO_PI;
    }

    return;
}

void ESMO_full_run(ESMO_Handle handle,
                   float32_t Vdcbus,
                   MATH_vec3 *pVabc_pu,
                   MATH_vec2 *pIabVec)
{
    ESMO_run(handle, Vdcbus, pVabc_pu, pIabVec);

    return;
}
