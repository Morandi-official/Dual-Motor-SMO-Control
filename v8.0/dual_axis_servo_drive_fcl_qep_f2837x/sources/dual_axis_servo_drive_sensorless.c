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

// PLL lag feedforward (DISABLED by default - see below). thetaErr is the
// P-only PLL's instantaneous phase lag. The original idea was to add it to
// the output angle to cancel the lag. The 2026-06-13 sweep showed this
// DOUBLE-compensates: the existing speed*esmoAngleDelaySF term plus the
// 0.40 offset already absorb most of the lag, so adding the full thetaErr
// (15..34 elec deg) on top OVER-corrects and inflates the residual bias to
// +24..36 deg. With the FF off the raw bias is only +7..9 deg (0.2-0.4pu)
// and -5 deg at 0.5pu, which the bias table below cancels cleanly. The FF
// is kept as a live experiment (set esmoThetaErrFFGain[motor] = 1.0 to
// re-enable, but then the bias table must be re-measured FF-on).
volatile float32_t esmoThetaErrFFGain[2] = {0.0f, 0.0f};
static float32_t esmoThetaErrFlt[2] = {0.0f, 0.0f};

// thetaErr low-pass: ~30 Hz at 10 kHz ISR (k = 2*pi*fc*Ts). Filters PLL
// ripple out of the FOC angle while still tracking load transients in ms.
#define ESMO_THETAERR_LPF_K     (0.0188f)

// Feedforward clamp: 0.12 pu = 43 elec deg, above the largest expected lag
// (34 deg at 0.5pu) but small enough to bound any startup garbage.
#define ESMO_THETAERR_FF_MAX_PU (0.12f)

// Speed-indexed bias compensation table. The residual bias against the
// index-calibrated QEP reference is reproducible to 0.1 elec deg run to
// run but NOT linear in speed (~+8 deg flat over 0.2-0.4pu, then -5 deg at
// 0.5pu where the voltage margin tightens), so a global offset/delaySF
// trim cannot flatten it. Linear interpolation between breakpoints, ramped
// from zero below the first breakpoint so the compensation is OFF during
// force-run and the takeover blend (rc setpoint is still the force speed
// there) and glides in with the speed ramp afterwards. Applied on the
// OUTPUT side only - never touches the handoff seed or the PLL.
//
// Re-ENABLED (esmoBiasCompGain = 1) now that the QEP frame is anchored
// (esmoQepIndexUseFixed). The 06-13 16:13 sweep (8 re-flash power-ups, FF
// off, gain 0) showed the true eSMO bias is now reproducible CROSS-SESSION
// to 0.05-0.22 deg: +13.4/+15.4/+9.5/-7.5 deg at 0.2/0.3/0.4/0.5pu. Because
// it no longer drifts between power-ups, a static table calibrated once
// transfers, so the values below = that measured bias and gain 1 nulls it
// (predicted residual ~0 at all four speeds).
//
// For pure QUANTIFICATION of the raw observer bias, set esmoBiasCompGain
// [motor] = 0 to read the true angleErr. With the table ON, angleErr shows
// the DEVIATION from this light-load calibration - useful for load testing
// (it reveals how far the bias moves under load). Re-calibrate (set the
// table to the gain-0 angleErr) if the motor's thermal state or load
// differs materially from the 16:13 light-load room-temperature sweep.
// Kslide must be at 0.55 (kslideHalf_q15 == 0.275) when calibrating.
#define ESMO_BIAS_TABLE_N       (4U)
#define ESMO_BIAS_RAMP_LO_PU    (0.15f)
static const float32_t esmoBiasSpeedBp[ESMO_BIAS_TABLE_N] =
        {0.20f, 0.30f, 0.40f, 0.50f};
volatile float32_t esmoBiasTablePu[ESMO_BIAS_TABLE_N] =
        {0.03726f, 0.04280f, 0.02646f, -0.02098f};
volatile float32_t esmoBiasCompGain[2] = {1.0f, 1.0f};

static float32_t esmoBiasTableLookup(float32_t setpointPu)
{
    float32_t spd = fabsf(setpointPu);
    float32_t biasPu;
    uint16_t i;

    if(spd <= ESMO_BIAS_RAMP_LO_PU)
    {
        biasPu = 0.0f;
    }
    else if(spd < esmoBiasSpeedBp[0])
    {
        biasPu = esmoBiasTablePu[0] * (spd - ESMO_BIAS_RAMP_LO_PU) /
                 (esmoBiasSpeedBp[0] - ESMO_BIAS_RAMP_LO_PU);
    }
    else if(spd >= esmoBiasSpeedBp[ESMO_BIAS_TABLE_N - 1U])
    {
        biasPu = esmoBiasTablePu[ESMO_BIAS_TABLE_N - 1U];
    }
    else
    {
        biasPu = esmoBiasTablePu[0];

        for(i = 0U; i < (ESMO_BIAS_TABLE_N - 1U); i++)
        {
            if(spd < esmoBiasSpeedBp[i + 1U])
            {
                biasPu = esmoBiasTablePu[i] +
                        ((esmoBiasTablePu[i + 1U] - esmoBiasTablePu[i]) *
                         (spd - esmoBiasSpeedBp[i]) /
                         (esmoBiasSpeedBp[i + 1U] - esmoBiasSpeedBp[i]));
                break;
            }
        }
    }

    // The table was measured at positive rotation; mirror for reverse.
    if(setpointPu < 0.0f)
    {
        biasPu = -biasPu;
    }

    return(biasPu);
}

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

static void seedESMOAngleAndSpeed(ESMO_Handle handle,
                                  float32_t anglePu,
                                  float32_t speedPu)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    anglePu = normalizePu(anglePu);
    speedPu = saturatePu(speedPu);

    obj->theta = anglePu;
    obj->thetaEst = anglePu * MATH_TWO_PI;

    if(obj->thetaEst > MATH_PI)
    {
        obj->thetaEst -= MATH_TWO_PI;
    }
    else if(obj->thetaEst < -MATH_PI)
    {
        obj->thetaEst += MATH_TWO_PI;
    }

    obj->speedEst = speedPu;
    obj->speedFlt = speedPu;
    obj->pll_Out = speedPu;
    obj->pll_ui = 0.0f;
    obj->thetaPll = anglePu;
    obj->thetaElec_rad = obj->thetaEst;
    obj->thetaOffset_rad = 0.0f;
    obj->thetaErr = 0.0f;
    obj->Ed = 0.0f;
    obj->Eq = 0.0f;
    obj->Eq_mag = 0.0f;

    return;
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

    esmoThetaErrFlt[(pMotor == &motorVars[0]) ? 0U : 1U] = 0.0f;

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
    float32_t seedSpeedPu;

    forceSensorlessAngle(pMotor, anglePu);

    if(esmoHandle != (ESMO_Handle)0)
    {
        rawAnglePu = normalizePu(pMotor->esmoAnglePu +
                                 pMotor->esmoAngleOffsetPu);
        pMotor->esmoRawAnglePu = rawAnglePu;

        seedSpeedPu = pMotor->rc.SetpointValue;

        if(fabsf(seedSpeedPu) < fabsf(pMotor->esmoForceSpeed))
        {
            if(seedSpeedPu < 0.0f)
            {
                seedSpeedPu = -fabsf(pMotor->esmoForceSpeed);
            }
            else
            {
                seedSpeedPu = fabsf(pMotor->esmoForceSpeed);
            }
        }

        seedESMOAngleAndSpeed(esmoHandle, rawAnglePu, seedSpeedPu);
        pMotor->esmoSpeedPu = seedSpeedPu;
        pMotor->esmoSpeedHz = seedSpeedPu * pMotor->baseFreq;
    }

    return;
}

void blendSensorlessAngle(MOTOR_Vars_t *pMotor, float32_t openLoopAnglePu)
{
    float32_t estimatorAnglePu = normalizePu(pMotor->esmoRawAnglePu -
                                             pMotor->esmoAngleOffsetPu);
    float32_t alpha = 1.0f;
    float32_t angleErrPu;

    if(pMotor->esmoTakeoverCntMax > 0U)
    {
        alpha = (float32_t)pMotor->esmoTakeoverCntr /
                (float32_t)pMotor->esmoTakeoverCntMax;
    }

    alpha = __fsat(alpha, 1.0f, 0.0f);
    alpha = alpha * alpha * (3.0f - (2.0f * alpha));

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
    uint16_t motorIdx;
    float32_t speedRef_Hz;
    float32_t lagCompPu;
    float32_t angleComp_rad;
    float32_t anglePLL_rad;
    float32_t estimatorAnglePu;
    float32_t speedPu;

    if((esmoHandle == (ESMO_Handle)0) || (isSensorlessActive(pMotor) == 0U))
    {
        return;
    }

    motorIdx = (pMotor == &motorVars[0]) ? 0U : 1U;

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

    // PLL lag feedforward: thetaErr is the PLL's instantaneous phase lag
    // (steady state: speed/Kp). Low-pass it and add it to the output angle.
    // This is an open-loop output correction - the PLL dynamics stay the
    // validated pure-P design - yet it tracks speed, load and acceleration
    // changes automatically with the filter's ~30Hz bandwidth.
    esmoThetaErrFlt[motorIdx] += ESMO_THETAERR_LPF_K *
            (((ESMO_Obj *)esmoHandle)->thetaErr - esmoThetaErrFlt[motorIdx]);

    lagCompPu = __fsat(esmoThetaErrFlt[motorIdx] *
                       esmoThetaErrFFGain[motorIdx],
                       ESMO_THETAERR_FF_MAX_PU, -ESMO_THETAERR_FF_MAX_PU);

    // Subtract the speed-indexed bias table (zero through force-run and
    // blend since the rc setpoint is still at the force speed there).
    lagCompPu -= (esmoBiasTableLookup(pMotor->rc.SetpointValue) *
                  esmoBiasCompGain[motorIdx]);

    pMotor->esmoSpeedHz = ESMO_getSpeedPLL_Hz(esmoHandle);
    angleComp_rad = (pMotor->esmoSpeedHz * pMotor->esmoAngleDelaySF) +
                    (lagCompPu * MATH_TWO_PI);
    anglePLL_rad = ESMO_incrAngle(ESMO_getAnglePLL(esmoHandle),
                                  angleComp_rad);

    pMotor->esmoRawAnglePu = normalizePu(anglePLL_rad * MATH_ONE_OVER_TWO_PI);
    estimatorAnglePu = normalizePu(pMotor->esmoRawAnglePu -
                                   pMotor->esmoAngleOffsetPu);

    if(isSensorlessTakeoverActive(pMotor) == 0U)
    {
        pMotor->esmoAnglePu = estimatorAnglePu;
        pMotor->esmoAngleRad = pMotor->esmoAnglePu * MATH_TWO_PI;

        if(pMotor->esmoAngleRad > MATH_PI)
        {
            pMotor->esmoAngleRad -= MATH_TWO_PI;
        }
    }

    speedPu = pMotor->esmoSpeedHz / pMotor->baseFreq;
    pMotor->esmoSpeedPu = saturatePu(speedPu);
    pMotor->esmoQepAnglePu = pMotor->ptrFCL->qep.ElecTheta;
    pMotor->esmoAngleErrPu = wrapPuHalf(pMotor->esmoAnglePu -
                                        pMotor->esmoQepAnglePu);
    pMotor->esmoSpeedErrPu = pMotor->esmoSpeedPu - pMotor->speed.Speed;

    return;
}
