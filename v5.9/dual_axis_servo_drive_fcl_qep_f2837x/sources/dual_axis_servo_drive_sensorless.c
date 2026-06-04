//#############################################################################
//
// FILE:    dual_axis_servo_drive_sensorless.c
//
// TITLE:   eSMO adapter for dual-axis FCL project
//
//#############################################################################

#include "dual_axis_servo_drive_settings.h"
#include "dual_axis_servo_drive_user.h"
#include "fcl_enum.h"
#include "fcl_cpu_cla_dm.h"
#include "dual_axis_servo_drive_sensorless.h"

ESMO_Obj esmoVars[2];

static float32_t normalizePu(float32_t anglePu)
{
    while(anglePu >= 1.0f)
    {
        anglePu -= 1.0f;
    }

    while(anglePu < 0.0f)
    {
        anglePu += 1.0f;
    }

    return(anglePu);
}

static float32_t saturatePu(float32_t value)
{
    return(__fsat(value, 1.0f, -1.0f));
}

static float32_t wrapPuHalf(float32_t value)
{
    while(value >= 0.5f)
    {
        value -= 1.0f;
    }

    while(value < -0.5f)
    {
        value += 1.0f;
    }

    return(value);
}

uint16_t isSensorlessActive(const MOTOR_Vars_t *pMotor)
{
    return(pMotor->positionFeedback != POSITION_FEEDBACK_QEP);
}

uint16_t isSensorlessControl(const MOTOR_Vars_t *pMotor)
{
    return(pMotor->positionFeedback == POSITION_FEEDBACK_ESMO);
}

uint16_t isSensorlessTakeoverActive(const MOTOR_Vars_t *pMotor)
{
    uint16_t active = 0U;

    if((isSensorlessControl(pMotor) != 0U) &&
       (pMotor->ptrFCL->lsw == ENC_CALIBRATION_DONE) &&
       (pMotor->esmoTakeoverCntr < pMotor->esmoTakeoverCntMax))
    {
        active = 1U;
    }

    return(active);
}

uint16_t isSensorlessReadyForClosedLoop(const MOTOR_Vars_t *pMotor)
{
    ESMO_Obj *obj = (ESMO_Obj *)pMotor->esmoHandle;
    float32_t forceSpeedAbs = fabs(pMotor->esmoForceSpeed);
    float32_t setpointAbs = fabs(pMotor->rc.SetpointValue);
    float32_t speedAbs = fabs(pMotor->esmoSpeedPu);
    float32_t minSetpoint;
    float32_t minSpeed;
    float32_t minBemf;
    uint16_t ready = 1U;

    if(isSensorlessControl(pMotor) == 0U)
    {
        return(ready);
    }

    ready = 0U;

    if(obj == (ESMO_Obj *)0)
    {
        return(ready);
    }

    if(pMotor == &motorVars[0])
    {
        minSetpoint = M1_ESMO_TAKEOVER_MIN_SETPOINT;
        minSpeed = M1_ESMO_TAKEOVER_MIN_SPEED;
        minBemf = M1_ESMO_TAKEOVER_MIN_BEMF;
    }
    else
    {
        minSetpoint = M2_ESMO_TAKEOVER_MIN_SETPOINT;
        minSpeed = M2_ESMO_TAKEOVER_MIN_SPEED;
        minBemf = M2_ESMO_TAKEOVER_MIN_BEMF;
    }

    if((setpointAbs >= minSetpoint) &&
       (speedAbs >= minSpeed) &&
       (setpointAbs >= (0.90f * forceSpeedAbs)) &&
       (speedAbs >= (0.50f * forceSpeedAbs)) &&
       ((pMotor->rc.SetpointValue * pMotor->esmoSpeedPu) > 0.0f) &&
       (obj->Eq_mag >= minBemf))
    {
        ready = 1U;
    }

    return(ready);
}

void initSensorlessEstimator(MOTOR_Vars_t *pMotor)
{
    ESMO_Params esmoParams;
    ESMO_Handle esmoHandle;

    if(pMotor == &motorVars[0])
    {
        esmoHandle = ESMO_init(&esmoVars[0], sizeof(ESMO_Obj));
        pMotor->esmoHandle = (void *)esmoHandle;

        ESMO_setPLLParams(esmoHandle, M1_ESMO_PLL_KP_MAX,
                          M1_ESMO_PLL_KP_MIN, M1_ESMO_PLL_KP_SF);
        ESMO_setKslideParams(esmoHandle, M1_ESMO_KSLIDE_MAX,
                             M1_ESMO_KSLIDE_MIN);
        ESMO_setBEMFThreshold(esmoHandle, M1_ESMO_BEMF_THRESHOLD);
        ESMO_setBEMFKslfFreq(esmoHandle, M1_ESMO_BEMF_KSLF_FC_HZ);
        ESMO_setOffsetCoef(esmoHandle, M1_ESMO_THETA_OFFSET_SF);
        ESMO_setSpeedFilterFreq(esmoHandle, M1_ESMO_SPEED_LPF_FC_HZ);

        esmoParams.maxFrequency_Hz = M1_BASE_FREQ;
        esmoParams.ctrlPeriod_sec = pMotor->Ts;
        esmoParams.voltageBase_V = M1_BASE_VOLTAGE;
        esmoParams.currentBase_A = M1_BASE_CURRENT;
        esmoParams.motor_Rs_d_Ohm = M1_RS;
        esmoParams.motor_Rs_q_Ohm = M1_RS;
        esmoParams.motor_Ls_d_H = M1_LD;
        esmoParams.motor_Ls_q_H = M1_LQ;
    }
    else
    {
        esmoHandle = ESMO_init(&esmoVars[1], sizeof(ESMO_Obj));
        pMotor->esmoHandle = (void *)esmoHandle;

        ESMO_setPLLParams(esmoHandle, M2_ESMO_PLL_KP_MAX,
                          M2_ESMO_PLL_KP_MIN, M2_ESMO_PLL_KP_SF);
        ESMO_setKslideParams(esmoHandle, M2_ESMO_KSLIDE_MAX,
                             M2_ESMO_KSLIDE_MIN);
        ESMO_setBEMFThreshold(esmoHandle, M2_ESMO_BEMF_THRESHOLD);
        ESMO_setBEMFKslfFreq(esmoHandle, M2_ESMO_BEMF_KSLF_FC_HZ);
        ESMO_setOffsetCoef(esmoHandle, M2_ESMO_THETA_OFFSET_SF);
        ESMO_setSpeedFilterFreq(esmoHandle, M2_ESMO_SPEED_LPF_FC_HZ);

        esmoParams.maxFrequency_Hz = M2_BASE_FREQ;
        esmoParams.ctrlPeriod_sec = pMotor->Ts;
        esmoParams.voltageBase_V = M2_BASE_VOLTAGE;
        esmoParams.currentBase_A = M2_BASE_CURRENT;
        esmoParams.motor_Rs_d_Ohm = M2_RS;
        esmoParams.motor_Rs_q_Ohm = M2_RS;
        esmoParams.motor_Ls_d_H = M2_LD;
        esmoParams.motor_Ls_q_H = M2_LQ;
    }

    ESMO_setParams(esmoHandle, &esmoParams);
    ESMO_resetParams(esmoHandle);

    return;
}

void resetSensorlessEstimator(MOTOR_Vars_t *pMotor)
{
    ESMO_Handle esmoHandle = (ESMO_Handle)pMotor->esmoHandle;

    pMotor->esmoForceRunCntr = 0;
    pMotor->esmoTakeoverCntr = 0;
    pMotor->esmoAnglePu = 0.0f;
    pMotor->esmoAngleRad = 0.0f;
    pMotor->esmoRawAnglePu = 0.0f;
    pMotor->esmoSpeedPu = 0.0f;
    pMotor->esmoSpeedHz = 0.0f;
    pMotor->esmoQepAnglePu = 0.0f;
    pMotor->esmoAngleErrPu = 0.0f;
    pMotor->esmoSpeedErrPu = 0.0f;
    pMotor->esmoIab_A[0] = 0.0f;
    pMotor->esmoIab_A[1] = 0.0f;
    pMotor->esmoVabc_pu[0] = 0.0f;
    pMotor->esmoVabc_pu[1] = 0.0f;
    pMotor->esmoVabc_pu[2] = 0.0f;

    if(esmoHandle != (ESMO_Handle)0)
    {
        ESMO_resetParams(esmoHandle);
    }

    return;
}

void forceSensorlessAngle(MOTOR_Vars_t *pMotor, float32_t anglePu)
{
    pMotor->esmoAnglePu = normalizePu(anglePu);
    pMotor->esmoAngleRad = pMotor->esmoAnglePu * MATH_TWO_PI;

    if(pMotor->esmoAngleRad > MATH_PI)
    {
        pMotor->esmoAngleRad -= MATH_TWO_PI;
    }

    return;
}

void syncSensorlessEstimatorAngle(MOTOR_Vars_t *pMotor, float32_t anglePu)
{
    ESMO_Handle esmoHandle = (ESMO_Handle)pMotor->esmoHandle;
    float32_t rawAnglePu;

    forceSensorlessAngle(pMotor, anglePu);

    if(esmoHandle != (ESMO_Handle)0)
    {
        rawAnglePu = normalizePu(pMotor->esmoAnglePu +
                                 pMotor->esmoAngleOffsetPu);
        pMotor->esmoRawAnglePu = rawAnglePu;
        ESMO_setAngleElecPu(esmoHandle, rawAnglePu);
    }

    return;
}

void blendSensorlessAngle(MOTOR_Vars_t *pMotor, float32_t openLoopAnglePu)
{
    float32_t estimatorAnglePu = pMotor->esmoAnglePu;
    float32_t alpha = 1.0f;
    float32_t angleErrPu;

    if(pMotor->esmoTakeoverCntMax > 0U)
    {
        alpha = (float32_t)pMotor->esmoTakeoverCntr /
                (float32_t)pMotor->esmoTakeoverCntMax;
    }

    alpha = __fsat(alpha, 1.0f, 0.0f);

    angleErrPu = wrapPuHalf(estimatorAnglePu - openLoopAnglePu);
    pMotor->esmoAnglePu = normalizePu(openLoopAnglePu + (alpha * angleErrPu));
    pMotor->esmoAngleRad = pMotor->esmoAnglePu * MATH_TWO_PI;

    if(pMotor->esmoAngleRad > MATH_PI)
    {
        pMotor->esmoAngleRad -= MATH_TWO_PI;
    }

    pMotor->esmoAngleErrPu = wrapPuHalf(pMotor->esmoAnglePu -
                                        pMotor->esmoQepAnglePu);

    if(pMotor->esmoTakeoverCntr < pMotor->esmoTakeoverCntMax)
    {
        pMotor->esmoTakeoverCntr++;
    }

    return;
}

void runSensorlessEstimator(MOTOR_Vars_t *pMotor)
{
    ESMO_Handle esmoHandle = (ESMO_Handle)pMotor->esmoHandle;
    MATH_vec3 Vabc_pu;
    MATH_vec2 Iab_A;
    float32_t speedRef_Hz;
    float32_t angleComp_rad;
    float32_t anglePLL_rad;
    float32_t speedPu;

    if((esmoHandle == (ESMO_Handle)0) || (isSensorlessActive(pMotor) == 0U))
    {
        return;
    }

    Vabc_pu.value[0] = pMotor->esmoVabc_pu[0];
    Vabc_pu.value[1] = pMotor->esmoVabc_pu[1];
    Vabc_pu.value[2] = pMotor->esmoVabc_pu[2];

    Iab_A.value[0] = pMotor->esmoIab_A[0];
    Iab_A.value[1] = pMotor->esmoIab_A[1];

    speedRef_Hz = pMotor->rc.SetpointValue * pMotor->baseFreq;
    ESMO_setSpeedRef(esmoHandle, speedRef_Hz);
    ESMO_updatePLLParams(esmoHandle);
    ESMO_run(esmoHandle, pMotor->FCL_params.Vdcbus, &Vabc_pu, &Iab_A);

    if(pMotor->ptrFCL->lsw == ENC_CALIBRATION_DONE)
    {
        ESMO_updateKslide(esmoHandle);
    }

    pMotor->esmoSpeedHz = ESMO_getSpeedPLL_Hz(esmoHandle);
    angleComp_rad = pMotor->esmoSpeedHz * pMotor->esmoAngleDelaySF;
    anglePLL_rad = ESMO_incrAngle(ESMO_getAnglePLL(esmoHandle),
                                  angleComp_rad);

    pMotor->esmoRawAnglePu = normalizePu(anglePLL_rad * MATH_ONE_OVER_TWO_PI);
    pMotor->esmoAnglePu = normalizePu(pMotor->esmoRawAnglePu -
                                      pMotor->esmoAngleOffsetPu);
    pMotor->esmoAngleRad = pMotor->esmoAnglePu * MATH_TWO_PI;

    if(pMotor->esmoAngleRad > MATH_PI)
    {
        pMotor->esmoAngleRad -= MATH_TWO_PI;
    }

    speedPu = pMotor->esmoSpeedHz / pMotor->baseFreq;
    pMotor->esmoSpeedPu = saturatePu(speedPu);
    pMotor->esmoQepAnglePu = pMotor->ptrFCL->qep.ElecTheta;
    pMotor->esmoAngleErrPu = wrapPuHalf(pMotor->esmoAnglePu -
                                        pMotor->esmoQepAnglePu);
    pMotor->esmoSpeedErrPu = pMotor->esmoSpeedPu - pMotor->speed.Speed;

    return;
}
