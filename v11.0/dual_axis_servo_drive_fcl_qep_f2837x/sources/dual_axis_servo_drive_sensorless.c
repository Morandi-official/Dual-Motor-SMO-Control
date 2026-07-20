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

#ifndef MATH_PI_OVER_TWO
#define MATH_PI_OVER_TWO   ((float32_t)(1.5707963267948966))
#endif

// ---- Field-weakening control (FWC), one voltage-magnitude PI per motor --
// Same algorithm as the SDK fwc library (voltage PI -> current-vector angle
// = pi/2 - PI_out -> cos/sin split into id/iq), implemented inline here to
// avoid pulling the fwc/pi SDK libraries (and their include-path / extra
// source-file dependencies) into the build. When Vs = sqrt(vd^2+vq^2)
// exceeds VsRef (a fraction of the modulation limit) the PI output goes
// negative, tilting the current vector past 90 deg to inject NEGATIVE id and
// weaken the back-emf so the drive runs past base speed (~0.55-0.6pu here).
//
// DISABLED by default: with flagEnableFWC[x] = 0 runFieldWeakening() returns
// without touching pi_id.ref / pi_iq.ref, so the build is bit-identical to
// the pre-FW behavior. Enable per motor from CCS and raise the gains
// gradually on the bench. Gains are starting guesses for the project voltage
// pu and 10kHz ISR rate - TUNE on hardware.
volatile uint16_t flagEnableFWC[2] = {0U, 0U};
volatile float32_t fwcKp[2]          = {M1_FWC_KP, M2_FWC_KP};
volatile float32_t fwcKi[2]          = {M1_FWC_KI, M2_FWC_KI};
volatile float32_t fwcVsRefSF[2]     = {M1_FWC_VSREF_SF, M2_FWC_VSREF_SF};
volatile float32_t fwcAngleMaxRad[2] =
        {(M1_FWC_MAX_ANGLE_DEG / 180.0f) * MATH_PI,
         (M2_FWC_MAX_ANGLE_DEG / 180.0f) * MATH_PI};
volatile float32_t fwcIdMaxNegPu[2]  = {M1_FWC_ID_MAX_NEG_PU, M2_FWC_ID_MAX_NEG_PU};
static float32_t fwcUi[2] = {0.0f, 0.0f};       // PI integrator state
// Diagnostics (watch in CCS).
volatile float32_t fwcVsPu[2]    = {0.0f, 0.0f};
volatile float32_t fwcVsRefPu[2] = {0.0f, 0.0f};
volatile float32_t fwcIdRefPu[2] = {0.0f, 0.0f};
volatile float32_t fwcAngleRad[2] = {0.0f, 0.0f};

// Keep the quiet 2026-06-22 angle path over the full speed range. A 2026-06-23
// full-electrical-cycle capture showed that the 0.55 thetaErr feedforward
// over-corrected the 0.9pu angle by about 27 electrical degrees. Leave the
// gain watch-tunable for controlled experiments, but keep it off by default.
volatile float32_t esmoThetaErrFFGain[2] = {0.0f, 0.0f};
volatile float32_t esmoThetaErrFFBlend[2] = {0.0f, 0.0f};
volatile float32_t esmoThetaErrFFPu[2] = {0.0f, 0.0f};
// The only high-speed stability aid retained after the 2026-06-22 baseline:
// the PLL Kp lift previously verified through 0.9pu. Acoustic isolation is
// handled by keeping thetaErr feedforward disabled and PI limits static.
volatile float32_t esmoPllKpHighSpeedGain[2] = {1.25f, 1.25f};
volatile float32_t esmoPllKpApplied[2] = {0.0f, 0.0f};
static float32_t esmoThetaErrFlt[2] = {0.0f, 0.0f};

#define ESMO_PLL_KP_BOOST_START_PU (0.70f)
#define ESMO_PLL_KP_BOOST_FULL_PU  (0.80f)
#define ESMO_PLL_KP_HARD_MAX       (7.00f)
#define ESMO_THETAERR_FF_START_PU  (0.82f)
#define ESMO_THETAERR_FF_FULL_PU   (0.88f)

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
// transfers, so the values below equal that measured bias and gain 1 nulls it
// (predicted residual approximately zero at all four measured speeds).
//
// For pure QUANTIFICATION of the raw observer bias, set esmoBiasCompGain
// [motor] = 0 to read the true angleErr. With the table ON, angleErr shows
// the DEVIATION from this light-load calibration - useful for load testing
// (it reveals how far the bias moves under load). Re-calibrate (set the
// table to the gain-0 angleErr) if the motor's thermal state or load
// differs materially from the 16:13 light-load room-temperature sweep.
// Kslide must be at 0.55 (kslideHalf_q15 == 0.275) when calibrating.
#define ESMO_BIAS_TABLE_N       (8U)
#define ESMO_BIAS_RAMP_LO_PU    (0.15f)
static const float32_t esmoBiasSpeedBp[ESMO_BIAS_TABLE_N] =
        {0.20f, 0.30f, 0.40f, 0.50f, 0.60f, 0.70f, 0.80f, 0.90f};
volatile float32_t esmoBiasTablePu[ESMO_BIAS_TABLE_N] =
        {0.02890f, 0.04280f, 0.02182f, -0.02098f,
         -0.07081f, -0.12820f, -0.12312f, -0.14984f};
volatile float32_t esmoBiasCompGain[2] = {1.0f, 1.0f};

// ---- Loss-of-lock watchdog state (parameter-variation robustness experiment).
// Defined here (above resetSensorlessEstimator, which clears the latch). See the
// updateSensorlessLockWatch() definition lower in this file for the full notes.
// READ esmoLostLock[x] once per test point: 1 = lost lock = cell OUTSIDE safe
// region. Latched for the whole run, cleared on STOP. No new datalog channel.
volatile float32_t esmoLockAngErrThrDeg[2] = {30.0f, 30.0f};  // |angErr| limit (elec deg)
volatile float32_t esmoLockSpdErrThrPu[2]  = {0.05f, 0.05f};  // |speedErr| limit (pu)
volatile uint16_t  esmoLockDebounceMs[2]   = {100U, 100U};    // must persist this long (ms)
volatile uint16_t  esmoLostLock[2]         = {0U, 0U};        // <-- READ per run (latched)
volatile float32_t esmoLockAngErrDeg[2]    = {0.0f, 0.0f};    // live |angErr| (elec deg)
static   uint16_t  esmoLockBadCntr[2]      = {0U, 0U};        // debounce counter (ISR ticks)

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

// ---- Runtime observer detune (Rs/Ls mismatch for the robustness experiment).
// Scale the eSMO's ASSUMED Rs/Ls. Observer-only: the real motor, the FCL current
// loop and the plant are untouched. Mismatch ratio = assumed / true (1.0 =
// nominal). Change esmoRsScale/esmoLsScale[x] in CCS while the motor is STOPPED;
// the background reset path detects the change and re-applies it (recomputes the
// observer's discrete gains Fd/Gd via ESMO_setParams), so the next MOTOR_RUN uses
// the detuned observer - no rebuild. esmoRsAssumedOhm/esmoLsAssumedH read back
// the value actually in use (to confirm the change took).
volatile float32_t esmoRsScale[2]      = {1.0f, 1.0f};
volatile float32_t esmoLsScale[2]      = {1.0f, 1.0f};
volatile float32_t esmoRsAssumedOhm[2] = {0.0f, 0.0f};
volatile float32_t esmoLsAssumedH[2]   = {0.0f, 0.0f};
static   float32_t esmoRsApplied[2]    = {1.0f, 1.0f};
static   float32_t esmoLsApplied[2]    = {1.0f, 1.0f};

// (Re)configure an already-initialized eSMO with the assumed Rs/Ls scaled by the
// current esmoRsScale/esmoLsScale. Mirrors the original init parameter sequence
// exactly when both scales are 1.0.
static void configureSensorlessParams(MOTOR_Vars_t *pMotor, uint16_t motorIdx)
{
    ESMO_Handle esmoHandle = (ESMO_Handle)pMotor->esmoHandle;
    ESMO_Params esmoParams;
    float32_t rsScale;
    float32_t lsScale;

    if((esmoHandle == (ESMO_Handle)0) || (motorIdx > 1U))
    {
        return;
    }

    rsScale = esmoRsScale[motorIdx];
    lsScale = esmoLsScale[motorIdx];

    if(motorIdx == 0U)
    {
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
        esmoParams.motor_Rs_d_Ohm = M1_RS * rsScale;
        esmoParams.motor_Rs_q_Ohm = M1_RS * rsScale;
        esmoParams.motor_Ls_d_H = M1_LD * lsScale;
        esmoParams.motor_Ls_q_H = M1_LQ * lsScale;
    }
    else
    {
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
        esmoParams.motor_Rs_d_Ohm = M2_RS * rsScale;
        esmoParams.motor_Rs_q_Ohm = M2_RS * rsScale;
        esmoParams.motor_Ls_d_H = M2_LD * lsScale;
        esmoParams.motor_Ls_q_H = M2_LQ * lsScale;
    }

    ESMO_setParams(esmoHandle, &esmoParams);
    ESMO_resetParams(esmoHandle);

    esmoRsAssumedOhm[motorIdx] = esmoParams.motor_Rs_d_Ohm;
    esmoLsAssumedH[motorIdx]   = esmoParams.motor_Ls_d_H;
    esmoRsApplied[motorIdx]    = rsScale;
    esmoLsApplied[motorIdx]    = lsScale;

    return;
}

void initSensorlessEstimator(MOTOR_Vars_t *pMotor)
{
    ESMO_Handle esmoHandle;
    uint16_t motorIdx = (pMotor == &motorVars[0]) ? 0U : 1U;

    esmoHandle = ESMO_iniæÚ$z{-®éÜj×÷F÷$–G…ÒÂ6—¦Vöb„U4Ôõôö&¢’“°¢Ö÷F÷"ÓæW6Öô†æFÆRÒ‡fö–B¢–W6Öô†æFÆS° ¢6öæf–wW&U6Vç6÷&ÆW75&×2‡Ö÷F÷"ÂÖ÷F÷$–G‚“° ¢&WGW&ã°§Ğ §fö–B&W6WE6Vç6÷&ÆW74W7F–ÖF÷"„ÔõDõ%õf'5÷B§Ö÷F÷"§°¢U4Ôõô†æFÆRW6Öô†æFÆRÒ„U4Ôõô†æFÆR—Ö÷F÷"ÓæW6Öô†æFÆS° ¢Ö÷F÷"ÓæW6Öôf÷&6U'Vä6çG"Ò°¢Ö÷F÷"ÓæW6ÖõF¶V÷fW$6çG"Ò°¢Ö÷F÷"ÓæW6ÖôævÆURÒãc°¢Ö÷F÷"ÓæW6ÖôævÆU&BÒãc°¢Ö÷F÷"ÓæW6Öõ&tævÆURÒãc°¢Ö÷F÷"ÓæW6Öõ7VVERÒãc°¢Ö÷F÷"ÓæW6Öõ7VVD‡¢Òãc°¢Ö÷F÷"ÓæW6ÖõWævÆURÒãc°¢Ö÷F÷"ÓæW6ÖôævÆTW'%RÒãc°¢Ö÷F÷"ÓæW6Öõ7VVDW'%RÒãc° ¢òò6ÆV"F†RÆ÷72ÖöbÖÆö6²ÆF6‚6òV6‚'Vâ7F'G2g&W6€¢W6ÖôÆ÷7DÆö6µ²‡Ö÷F÷"ÓÒfÖ÷F÷%f'5³Ò’òR¢UÒÒS°¢W6ÖôÆö6´&D6çG%²‡Ö÷F÷"ÓÒfÖ÷F÷%f'5³Ò’òR¢UÒÒS°¢Ö÷F÷"ÓæW6Öô–%ô³ÒÒãc°¢Ö÷F÷"ÓæW6Öô–%ô³ÒÒãc°¢Ö÷F÷"ÓæW6Öõf&5÷U³ÒÒãc°¢Ö÷F÷"ÓæW6Öõf&5÷U³ÒÒãc°¢Ö÷F÷"ÓæW6Öõf&5÷U³%ÒÒãc° ¢W6ÖõF†WFW'$fÇE²‡Ö÷F÷"ÓÒfÖ÷F÷%f'5³Ò’òR¢UÒÒãc°¢W6ÖõF†WFW'$dd&ÆVæE²‡Ö÷F÷"ÓÒfÖ÷F÷%f'5³Ò’òR¢UÒÒãc°¢W6ÖõF†WFW'$deU²‡Ö÷F÷"ÓÒfÖ÷F÷%f'5³Ò’òR¢UÒÒãc°¢W6ÖõÆÄ·Æ–VE²‡Ö÷F÷"ÓÒfÖ÷F÷%f'5³Ò’òR¢UÒÒãc° ¢òò6ÆV"F†Rf–VÆB×vV¶Væ–ær–çFVw&F÷"öâWfW'’5Dõ6òV6‚'Vâ7F'G0¢òòv—F‚–BÒ†æò&W6–GVÂvV¶Væ–ær’ÒF†R&VfW&Væ6W2&R&W7F÷&VB'¢òòF†Ræ÷&ÖÂ7F'GWF‚à¢gv5V•²‡Ö÷F÷"ÓÒfÖ÷F÷%f'5³Ò’òR¢UÒÒãc°¢gv4–E&VeU²‡Ö÷F÷"ÓÒfÖ÷F÷%f'5³Ò’òR¢UÒÒãc° ¢–b†W6Öô†æFÆRÒ„U4Ôõô†æFÆR“¢°¢U4Ôõ÷&W6WE&×2†W6Öô†æFÆR“° ¢òò'VçF–ÖRö'6W'fW"FWGVæS¢–bF†R452'2ôÇ266ÆR6†ævVB‡G—–6ÆÇ¢òò6WBv†–ÆR7F÷VB’Â&RÖÇ’—Bæ÷r6òF†RæW‡BÔõDõ%õ%TâW6W2F†P¢òòFWGVæVBö'6W'fW"â&V6ö×WFVBöæÇ’öâ6†ævR†æòW"Ô•5"W‡b‚’’à¢°¢V–çCe÷BÖ÷F÷$–G‚Ò‡Ö÷F÷"ÓÒfÖ÷F÷%f'5³Ò’òR¢S° ¢–b‚†W6Öõ'566ÆU¶Ö÷F÷$–G…ÒÒW6Öõ'4Æ–VE¶Ö÷F÷$–G…Ò’ÇÀ¢†W6ÖôÇ566ÆU¶Ö÷F÷$–G…ÒÒW6ÖôÇ4Æ–VE¶Ö÷F÷$–G…Ò’¢°¢6öæf–wW&U6Vç6÷&ÆW75&×2‡Ö÷F÷"ÂÖ÷F÷$–G‚“°¢Ğ¢Ğ¢Ğ ¢&WGW&ã°§Ğ §fö–Bf÷&6U6Vç6÷&ÆW74ævÆR„ÔõDõ%õf'5÷B§Ö÷F÷"ÂfÆöC3%÷BævÆUR§°¢Ö÷F÷"ÓæW6ÖôævÆURÒæ÷&ÖÆ—¦UR†ævÆUR“°¢Ö÷F÷"ÓæW6ÖôævÆU&BÒÖ÷F÷"ÓæW6ÖôævÆUR¢ÔD…õEtõõ“° ¢–b‡Ö÷F÷"ÓæW6ÖôævÆU&BâÔD…õ’¢°¢Ö÷F÷"ÓæW6ÖôævÆU&BÓÒÔD…õEtõõ“°¢Ğ ¢&WGW&ã°§Ğ §fö–B7–æ56Vç6÷&ÆW74W7F–ÖF÷$ævÆR„ÔõDõ%õf'5÷B§Ö÷F÷"ÂfÆöC3%÷BævÆUR§°¢U4Ôõô†æFÆRW6Öô†æFÆRÒ„U4Ôõô†æFÆR—Ö÷F÷"ÓæW6Öô†æFÆS°¢fÆöC3%÷B&tævÆUS°¢fÆöC3%÷B6VVE7VVES° ¢f÷&6U6Vç6÷&ÆW74ævÆR‡Ö÷F÷"ÂævÆUR“° ¢–b†W6Öô†æFÆRÒ„U4Ôõô†æFÆR“¢°¢&tævÆURÒæ÷&ÖÆ—¦UR‡Ö÷F÷"ÓæW6ÖôævÆUR°¢Ö÷F÷"ÓæW6ÖôævÆTöfg6WER“°¢Ö÷F÷"ÓæW6Öõ&tævÆURÒ&tævÆUS° ¢6VVE7VVERÒÖ÷F÷"Óç&2å6WGö–çEfÇVS° ¢–b†f'6b‡6VVE7VVER’Âf'6b‡Ö÷F÷"ÓæW6Öôf÷&6U7VVB’¢°¢–b‡6VVE7VVERÂãb¢°¢6VVE7VVERÒÖf'6b‡Ö÷F÷"ÓæW6Öôf÷&6U7VVB“°¢Ğ¢VÇ6P¢°¢6VVE7VVERÒf'6b‡Ö÷F÷"ÓæW6Öôf÷&6U7VVB“°¢Ğ¢Ğ ¢6VVDU4ÔôævÆTæE7VVB†W6Öô†æFÆRÂ&tævÆURÂ6VVE7VVER“°¢Ö÷F÷"ÓæW6Öõ7VVERÒ6VVE7VVES°¢Ö÷F÷"ÓæW6Öõ7VVD‡¢Ò6VVE7VVER¢Ö÷F÷"Óæ&6Tg&W°¢Ğ ¢&WGW&ã°§Ğ §fö–B&ÆVæE6Vç6÷&ÆW74ævÆR„ÔõDõ%õf'5÷B§Ö÷F÷"ÂfÆöC3%÷B÷VäÆö÷ævÆUR§°¢fÆöC3%÷BW7F–ÖF÷$ævÆURÒæ÷&ÖÆ—¦UR‡Ö÷F÷"ÓæW6Öõ&tævÆURĞ¢Ö÷F÷"ÓæW6ÖôævÆTöfg6WER“°¢fÆöC3%÷BÇ†Òãc°¢fÆöC3%÷BævÆTW'%S° ¢–b‡Ö÷F÷"ÓæW6ÖõF¶V÷fW$6çDÖ‚âR¢°¢Ç†Ò†fÆöC3%÷B—Ö÷F÷"ÓæW6ÖõF¶V÷fW$6çG"ğ¢†fÆöC3%÷B—Ö÷F÷"ÓæW6ÖõF¶V÷fW$6çDÖƒ°¢Ğ ¢Ç†Òõög6B†Ç†ÂãbÂãb“°¢Ç†ÒÇ†¢Ç†¢ƒ2ãbÒƒ"ãb¢Ç†’“° ¢ævÆTW'%RÒw&T†Æb†W7F–ÖF÷$ævÆURÒ÷VäÆö÷ævÆUR“°¢Ö÷F÷"ÓæW6ÖôævÆURÒæ÷&ÖÆ—¦UR†÷VäÆö÷ævÆUR²†Ç†¢ævÆTW'%R’“°¢Ö÷F÷"ÓæW6ÖôævÆU&BÒÖ÷F÷"ÓæW6ÖôævÆUR¢ÔD…õEtõõ“° ¢–b‡Ö÷F÷"ÓæW6ÖôævÆU&BâÔD…õ’¢°¢Ö÷F÷"ÓæW6ÖôævÆU&BÓÒÔD…õEtõõ“°¢Ğ ¢Ö÷F÷"ÓæW6ÖôævÆTW'%RÒw&T†Æb‡Ö÷F÷"ÓæW6ÖôævÆURĞ¢Ö÷F÷"ÓæW6ÖõWævÆUR“° ¢–b‡Ö÷F÷"ÓæW6ÖõF¶V÷fW$6çG"ÂÖ÷F÷"ÓæW6ÖõF¶V÷fW$6çDÖ‚¢°¢Ö÷F÷"ÓæW6ÖõF¶V÷fW$6çG"²³°¢Ğ ¢&WGW&ã°§Ğ §fö–B'Vå6Vç6÷&ÆW74W7F–ÖF÷"„ÔõDõ%õf'5÷B§Ö÷F÷"§°¢U4Ôõô†æFÆRW6Öô†æFÆRÒ„U4Ôõô†æFÆR—Ö÷F÷"ÓæW6Öô†æFÆS°¢ÔD…÷fV32f&5÷S°¢ÔD…÷fV3"–%ô°¢V–çCe÷BÖ÷F÷$–Gƒ°¢fÆöC3%÷B7VVE&Veô‡£°¢fÆöC3%÷B7VVE&Vd'5S°¢fÆöC3%÷BÆÄ·&ÆVæC°¢fÆöC3%÷BÆÄ·v–ã°¢fÆöC3%÷BÆÄ·Æ–VC°¢fÆöC3%÷BÆt6ö×S°¢fÆöC3%÷Bfd&ÆVæC°¢fÆöC3%÷B7VVD'5S°¢fÆöC3%÷BævÆT6ö×÷&C°¢fÆöC3%÷BævÆUÄÅ÷&C°¢fÆöC3%÷BW7F–ÖF÷$ævÆUS°¢fÆöC3%÷B7VVES° ¢–b‚†W6Öô†æFÆRÓÒ„U4Ôõô†æFÆR“’ÇÂ†—56Vç6÷&ÆW747F—fR‡Ö÷F÷"’ÓÒR’¢°¢&WGW&ã°¢Ğ ¢Ö÷F÷$–G‚Ò‡Ö÷F÷"ÓÒfÖ÷F÷%f'5³Ò’òR¢S° ¢f&5÷RçfÇVU³ÒÒÖ÷F÷"ÓæW6Öõf&5÷U³Ó°¢f&5÷RçfÇVU³ÒÒÖ÷F÷"ÓæW6Öõf&5÷U³Ó°¢f&5÷RçfÇVU³%ÒÒÖ÷F÷"ÓæW6Öõf&5÷U³%Ó° ¢–%ôçfÇVU³ÒÒÖ÷F÷"ÓæW6Öô–%ô³Ó°¢–%ôçfÇVU³ÒÒÖ÷F÷"ÓæW6Öô–%ô³Ó° ¢7VVE&Veô‡¢ÒÖ÷F÷"Óç&2å6WGö–çEfÇVR¢Ö÷F÷"Óæ&6Tg&W°¢U4Ôõ÷6WE7VVE&Vb†W6Öô†æFÆRÂ7VVE&Veô‡¢“°¢U4Ôõ÷WFFUÄÅ&×2†W6Öô†æFÆR“° ¢7VVE&Vd'5RÒf'6b‡Ö÷F÷"Óç&2å6WGö–çEfÇVR“°¢ÆÄ·Æ–VBÒ‚„U4Ôõôö&¢¢–W6Öô†æFÆR’ÓçÆÅô·° ¢–b‡7VVE&Vd'5RâU4ÔõõÄÅôµô$ôõ5Eõ5D%EõR¢°¢–b‡7VVE&Vd'5RãÒU4ÔõõÄÅôµô$ôõ5EôeTÄÅõR¢°¢ÆÄ·&ÆVæBÒãc°¢Ğ¢VÇ6P¢°¢ÆÄ·&ÆVæBÒ‡7VVE&Vd'5RÒU4ÔõõÄÅôµô$ôõ5Eõ5D%EõR’ğ¢„U4ÔõõÄÅôµô$ôõ5EôeTÄÅõRĞ¢U4ÔõõÄÅôµô$ôõ5Eõ5D%EõR“°¢Ğ ¢ÆÄ·v–âÒãb²‡ÆÄ·&ÆVæB ¢…õög6B†W6ÖõÆÄ·†–v…7VVDv–å¶Ö÷F÷$–G…ÒÀ¢ã3VbÂãb’Òãb’“°¢ÆÄ·Æ–VB£ÒÆÄ·v–ã°¢ÆÄ·Æ–VBÒõög6B‡ÆÄ·Æ–VBÂU4ÔõõÄÅôµô„$EôÔ‚À¢‚„U4Ôõôö&¢¢–W6Öô†æFÆR’ÓçÆÅô·Ö–â“°¢U4Ôõ÷6WEÄÄ·†W6Öô†æFÆRÂÆÄ·Æ–VB“°¢Ğ ¢W6ÖõÆÄ·Æ–VE¶Ö÷F÷$–G…ÒÒÆÄ·Æ–VC° ¢U4Ôõ÷'Vâ†W6Öô†æFÆRÂÖ÷F÷"Óäd4Å÷&×2åfF6'W2Âef&5÷RÂd–%ô“° ¢–b‡Ö÷F÷"ÓçG$d4ÂÓæÇ7rÓÒTä5ô4Ä”%$D”ôåôDôäR¢°¢U4Ôõ÷WFFT·6Æ–FR†W6Öô†æFÆR“°¢Ğ ¢òòÄÂÆrfVVFf÷'v&C¢F†WFW'"—2F†RÄÂw2–ç7FçFæV÷W2†6RÆp¢òò‡7FVG’7FFS¢7VVBô·’âÆ÷r×72—BæBFB—BFòF†R÷WGWBævÆRà¢òòF†—2—2â÷VâÖÆö÷÷WGWB6÷'&V7F–öã²F†RÄÂ&VÖ–ç2F†RW7F&Æ—6†V@¢òòÖöæÇ’FW6–vâv†–ÆRF†Rf–ÇFW&VBFW&Ò6÷fW'2—G2G–æÖ–2†6RÆrà¢W6ÖõF†WFW'$fÇE¶Ö÷F÷$–G…Ò³ÒU4ÔõõD„UDU%%ôÅeô² ¢‚‚„U4Ôõôö&¢¢–W6Öô†æFÆR’ÓçF†WFW'"ÒW6ÖõF†WFW'$fÇE¶Ö÷F÷$–G…Ò“° ¢Ö÷F÷"ÓæW6Öõ7VVD‡¢ÒU4ÔõövWE7VVEÄÅô‡¢†W6Öô†æFÆR“°¢7VVD'5RÒf'6b‡Ö÷F÷"ÓæW6Öõ7VVD‡¢òÖ÷F÷"Óæ&6Tg&W“° ¢–b‡7VVD'5RÃÒU4ÔõõD„UDU%%ôdeõ5D%EõR¢°¢fd&ÆVæBÒãc°¢Ğ¢VÇ6R–b‡7VVD'5RãÒU4ÔõõD„UDU%%ôdeôeTÄÅõR¢°¢fd&ÆVæBÒãc°¢Ğ¢VÇ6P¢°¢fd&ÆVæBÒ‡7VVD'5RÒU4ÔõõD„UDU%%ôdeõ5D%EõR’ğ¢„U4ÔõõD„UDU%%ôdeôeTÄÅõRÒU4ÔõõD„UDU%%ôdeõ5D%EõR“°¢Ğ ¢Æt6ö×RÒõög6B†W6ÖõF†WFW'$fÇE¶Ö÷F÷$–G…Ò ¢W6ÖõF†WFW'$ddv–å¶Ö÷F÷$–G…Ò¢fd&ÆVæBÀ¢U4ÔõõD„UDU%%ôdeôÔ…õRÂÔU4ÔõõD„UDU%%ôdeôÔ…õR“°¢W6ÖõF†WFW'$dd&ÆVæE¶Ö÷F÷$–G…ÒÒfd&ÆVæC°¢W6ÖõF†WFW'$deU¶Ö÷F÷$–G…ÒÒÆt6ö×S° ¢òò7V'G&7BF†R7VVBÖ–æFW†VB&–2F&ÆR‡¦W&òF‡&÷Vv‚f÷&6R×'Vâæ@¢òò&ÆVæB6–æ6RF†R&26WGö–çB—27F–ÆÂBF†Rf÷&6R7VVBF†W&R’à¢Æt6ö×RÓÒ†W6Öô&–5F&ÆTÆöö·W‡Ö÷F÷"Óç&2å6WGö–çEfÇVR’ ¢W6Öô&–46ö×v–å¶Ö÷F÷$–G…Ò“° ¢ævÆT6ö×÷&BÒ‡Ö÷F÷"ÓæW6Öõ7VVD‡¢¢Ö÷F÷"ÓæW6ÖôævÆTFVÆ•4b’°¢†Æt6ö×R¢ÔD…õEtõõ’“°¢ævÆUÄÅ÷&BÒU4Ôõö–æ7$ævÆR„U4ÔõövWDævÆUÄÂ†W6Öô†æFÆR’À¢ævÆT6ö×÷&B“° ¢Ö÷F÷"ÓæW6Öõ&tævÆURÒæ÷&ÖÆ—¦UR†ævÆUÄÅ÷&B¢ÔD…ôôäUôõdU%õEtõõ’“°¢W7F–ÖF÷$ævÆURÒæ÷&ÖÆ—¦UR‡Ö÷F÷"ÓæW6Öõ&tævÆURĞ¢Ö÷F÷"ÓæW6ÖôævÆTöfg6WER“° ¢–b†—56Vç6÷&ÆW75F¶V÷fW$7F—fR‡Ö÷F÷"’ÓÒR¢°¢Ö÷F÷"ÓæW6ÖôævÆURÒW7F–ÖF÷$ævÆUS°¢Ö÷F÷"ÓæW6ÖôævÆU&BÒÖ÷F÷"ÓæW6ÖôævÆUR¢ÔD…õEtõõ“° ¢–b‡Ö÷F÷"ÓæW6ÖôævÆU&BâÔD…õ’¢°¢Ö÷F÷"ÓæW6ÖôævÆU&BÓÒÔD…õEtõõ“°¢Ğ¢Ğ ¢7VVERÒÖ÷F÷"ÓæW6Öõ7VVD‡¢òÖ÷F÷"Óæ&6Tg&W°¢Ö÷F÷"ÓæW6Öõ7VVERÒ6GW&FUR‡7VVER“°¢Ö÷F÷"ÓæW6ÖõWævÆURÒÖ÷F÷"ÓçG$d4ÂÓçWäVÆV5F†WF°¢Ö÷F÷"ÓæW6ÖôævÆTW'%RÒw&T†Æb‡Ö÷F÷"ÓæW6ÖôævÆURĞ¢Ö÷F÷"ÓæW6ÖõWævÆUR“°¢Ö÷F÷"ÓæW6Öõ7VVDW'%RÒÖ÷F÷"ÓæW6Öõ7VVERÒÖ÷F÷"Óç7VVBå7VVC° ¢&WGW&ã°§Ğ ¢òòÒÒÒÒÆ÷72ÖöbÖÆö6²vF6†För‡&ÖWFW"×f&–F–öâ&ö'W7FæW72W‡W&–ÖVçB’ÒÒÒĞ¢òòÆF6†W2W6ÖôÆ÷7DÆö6µ·…ÓÓv†VâÂ–â6Vç6÷&ÆW724Äõ4TBÆö÷†gFW"F†RF¶V÷fW ¢òò&ÆVæB’ÂF†RU4ÔòævÆRW'&÷"g2F†R–æFW‚Öæ6†÷&VBU&VfW&Væ6RW†6VVG0¢òòW6ÖôÆö6´ætW'%F‡$FVr†VÆV2FVr’õ"F†R7VVBW'&÷"W†6VVG2W6ÖôÆö6µ7DW'%F‡%P¢òò‡R’6öçF–çV÷W6Ç’f÷"ÆöævW"F†âW6ÖôÆö6´FV&÷Væ6T×2âF†—2—2F†RW"×FW7BĞ¢òòö–çBÄô4²FV6—6–öâf÷"F†R…'2ÆÆÖ&FÇ7VVB’&ö'W7FæW72Ö ¢òòW6ÖôÆ÷7DÆö6µ·…ÒÓÒÓâ7F–VBÆö6¶VBÓâw&–B6VÆÂ”å4”DRF†R6fR&Vv–öà¢òòW6ÖôÆ÷7DÆö6µ·…ÒÓÒÓâÆ÷7BÆö6²Óâw&–B6VÆÂõUE4”DRF†R6fR&Vv–öà¢òò$TBW6ÖôÆ÷7DÆö6µ·…Òöæ6RW"'Vâ–â452†—B—2ÆF6†VBf÷"F†Rv†öÆR'Vâ“°¢òò—B—26ÆV&VBWFöÖF–6ÆÇ’öâ5DõòV6‚æWr'Vâ‡&W6WE6Vç6÷&ÆW74W7F–ÖF÷"’à¢òòF†R“gƒrFFÆör'VffW"FöW2äõBæVVBæWr6†ææVÂÒF†—2—2W"×'VâfÆrà¢òòW6ÖôÆö6´ætW'$FVu·…Ò—2F†RÆ—fRÆætW''Â–âFVr††æG’FòvF6‚÷Æ÷B’à¢òòF‡&W6†öÆG2&RvF6‚×GVæ&ÆRg&öÒ452âF†RfÆr—2Gf—6÷'’æBFöW2äõB7F÷ ¢òòF†RG&—fR‡F†RW†—7F–ær÷fW"Ö7W'&VçBö÷fW"×7VVB&÷FV7F–öâwV&G2F†R…r“²õ ¢òòW6ÖôÆ÷7DÆö6²–çFò–÷W"7F÷Æöv–2–b–÷RvçBWFò×6‡WFF÷vâæV"F†R&÷VæF'’à¢òò…7FFRf&–&ÆW2&RFVf–æVBæV"F†RF÷öbF†—2f–ÆRÂ&÷fRF†R&W6WBâ§fö–BWFFU6Vç6÷&ÆW74Æö6µvF6‚„ÔõDõ%õf'5÷B§Ö÷F÷"ÂV–çCe÷BÖ÷F÷$–G‚§°¢fÆöC3%÷BætW'$FVs°¢fÆöC3%÷B7DW'%S°¢V–çC3%÷BFV&÷Væ6UF–6·3° ¢–b†Ö÷F÷$–G‚âR¢°¢&WGW&ã°¢Ğ ¢òòöæÇ’ÖVæ–ævgVÂöæ6RF†RU4Ôò—27GVÆÇ’7FVW&–æs¢6Vç6÷&ÆW726öçG&öÂÀ¢òò6Æ÷6VBÆö÷„DôäR’ÂæB7BF†RF¶V÷fW"&ÆVæBâ÷WG6–FRF†BÂ†öÆBF†P¢òòFV&÷Væ6R6÷VçFW"B'WBFòäõB6ÆV"F†RÆF6‚†Æ÷72V&Æ–W"–âF†P¢òò'Vâ×W7B7F–ÆÂ&R&W÷'FVB’âF†RÆF6‚6ÆV'2öâ5Dõà¢–b‚†—56Vç6÷&ÆW746öçG&öÂ‡Ö÷F÷"’ÓÒR’ÇÀ¢‡Ö÷F÷"ÓçG$d4ÂÓæÇ7rÒTä5ô4Ä”%$D”ôåôDôäR’ÇÀ¢†—56Vç6÷&ÆW75F¶V÷fW$7F—fR‡Ö÷F÷"’ÒR’¢°¢W6ÖôÆö6´&D6çG%¶Ö÷F÷$–G…ÒÒS°¢&WGW&ã°¢Ğ ¢ætW'$FVrÒf'6b‡Ö÷F÷"ÓæW6ÖôævÆTW'%R’¢3cãc²òòR×&WbÓâVÆV2FVp¢7DW'%RÒf'6b‡Ö÷F÷"ÓæW6Öõ7VVDW'%R“°¢W6ÖôÆö6´ætW'$FVu¶Ö÷F÷$–G…ÒÒætW'$FVs° ¢FV&÷Væ6UF–6·2Ò‡V–çC3%÷B’‚‚†fÆöC3%÷B–W6ÖôÆö6´FV&÷Væ6T×5¶Ö÷F÷$–G…Ò’ ¢ãbòÖ÷F÷"ÓåG2“²òò×2Óâ•5"F–6·0 ¢–b‚†ætW'$FVrâW6ÖôÆö6´ætW'%F‡$FVu¶Ö÷F÷$–G…Ò’ÇÀ¢‡7DW'%RâW6ÖôÆö6µ7DW'%F‡%U¶Ö÷F÷$–G…Ò’¢°¢W6ÖôÆö6´&D6çG%¶Ö÷F÷$–G…Ò²³°¢–b‚‡V–çC3%÷B–W6ÖôÆö6´&D6çG%¶Ö÷F÷$–G…ÒãÒFV&÷Væ6UF–6·2¢°¢W6ÖôÆ÷7DÆö6µ¶Ö÷F÷$–G…ÒÒS²òòÆF6†V@¢W6ÖôÆö6´&D6çG%¶Ö÷F÷$–G…ÒÒ‡V–çCe÷B–FV&÷Væ6UF–6·3²òòæòw& ¢Ğ¢Ğ¢VÇ6P¢°¢W6ÖôÆö6´&D6çG%¶Ö÷F÷$–G…ÒÒS°¢Ğ ¢&WGW&ã°§Ğ §fö–B'Väf–VÆEvV¶Væ–ær„ÔõDõ%õf'5÷B§Ö÷F÷"ÂV–çCe÷BÖ÷F÷$–G‚§°¢fÆöC3%÷BfC°¢fÆöC3%÷Bg°¢fÆöC3%÷Bg3°¢fÆöC3%÷BdÆ–Ö—C°¢fÆöC3%÷Bg5&Vc°¢fÆöC3%÷BævÆTÖ–ã°¢fÆöC3%÷BW'#°¢fÆöC3%÷B÷WC°¢fÆöC3%÷BævÆT7W'&VçC°¢fÆöC3%÷B—5&Vc°¢fÆöC3%÷B—4Ös°¢fÆöC3%÷B—56–vã°¢fÆöC3%÷B–C°¢fÆöC3%÷B—°¢fÆöC3%÷B–DÖ„æVs° ¢–b†Ö÷F÷$–G‚âR¢°¢&WGW&ã°¢Ğ ¢òòVævvRôäÅ’–â7FVG’6Æ÷6VBÆö÷†gFW"F†RF¶V÷fW"&ÆVæB’v—F‚ep¢òòVæ&ÆVBâ÷F†W'v—6R6ÆV"F†R–çFVw&F÷"æBÆVfRF†R&VfW&Væ6W2F†@¢òò'V–ÆDÆWfVÃCbÇ&VG’6WBÒ6òeröfb—2&—BÖ–FVçF–6ÂFòF†RöÆBF‚À¢òòæB7F'GWòÆ–væÖVçBò†æFöfb&RæWfW"F÷V6†VBà¢–b‚†fÆtVæ&ÆTet5¶Ö÷F÷$–G…ÒÓÒR’ÇÀ¢†—56Vç6÷&ÆW746öçG&öÂ‡Ö÷F÷"’ÓÒR’ÇÀ¢‡Ö÷F÷"ÓçG$d4ÂÓæÇ7rÒTä5ô4Ä”%$D”ôåôDôäR’ÇÀ¢†—56Vç6÷&ÆW75F¶V÷fW$7F—fR‡Ö÷F÷"’ÒR’¢°¢gv5V•¶Ö÷F÷$–G…ÒÒãc°¢gv4–E&VeU¶Ö÷F÷$–G…ÒÒãc°¢&WGW&ã°¢Ğ ¢òò7FF÷"föÇFvRÖvæ—GVFRæBF†RÖöGVÆF–öâÖÆ–Ö—BÖ&6VB&VfW&Væ6Rà¢fBÒÖ÷F÷"Óç•ö–Bæ÷WC°¢gÒÖ÷F÷"ÓçG$d4ÂÓç•ö—æ÷WC°¢g2Ò7'Fb‚‡fB¢fB’²‡g¢g’“° ¢dÆ–Ö—BÒf'6b‡Ö÷F÷"Óç•ö–BåVÖ‚“° ¢–b†f'6b‡Ö÷F÷"ÓçG$d4ÂÓç•ö—åVÖ‚’ÂdÆ–Ö—B¢°¢dÆ–Ö—BÒf'6b‡Ö÷F÷"ÓçG$d4ÂÓç•ö—åVÖ‚“°¢Ğ ¢g5&VbÒgv5g5&Ve4e¶Ö÷F÷$–G…Ò¢dÆ–Ö—C° ¢òòföÇFvRÖÖvæ—GVFR’â÷WGWB†ævÆR–â&B’—26Æ×VBFğ¢òò¶ævÆTÖ–âƒÃ’ÂÓ¢—B6—G2B†æòer’v†–ÆRg2ÃÒg5&VbæBöæÇ¢òòvöW2æVvF—fRv†Vâg2÷fW'6†ö÷G2F†R&VfW&Væ6RâævÆTÖ–â62F†P¢òòf–VÆB×vV¶Væ–ærF–ÇBà¢ævÆTÖ–âÒgv4ævÆTÖ…&E¶Ö÷F÷$–G…Ó°¢W'"Òg5&VbÒg3° ¢gv5V•¶Ö÷F÷$–G…Ò³Ò†gv4¶•¶Ö÷F÷$–G…Ò¢W'"“° ¢–b†gv5V•¶Ö÷F÷$–G…Òâãb¢°¢gv5V•¶Ö÷F÷$–G…ÒÒãc°¢Ğ¢VÇ6R–b†gv5V•¶Ö÷F÷$–G…ÒÂævÆTÖ–â¢°¢gv5V•¶Ö÷F÷$–G…ÒÒævÆTÖ–ã°¢Ğ ¢÷WBÒ†gv4·¶Ö÷F÷$–G…Ò¢W'"’²gv5V•¶Ö÷F÷$–G…Ó° ¢–b†÷WBâãb¢°¢÷WBÒãc°¢Ğ¢VÇ6R–b†÷WBÂævÆTÖ–â¢°¢÷WBÒævÆTÖ–ã°¢Ğ ¢ævÆT7W'&VçBÒÔD…õ•ôõdU%õEtòÒ÷WC²òòãÒ’ó  ¢gv5g5U¶Ö÷F÷$–G…ÒÒg3°¢gv5g5&VeU¶Ö÷F÷$–G…ÒÒg5&Vc°¢gv4ævÆU&E¶Ö÷F÷$–G…ÒÒævÆT7W'&VçC° ¢òò7Æ—BF†R&W6VçB—6öÖÖæBÖvæ—GVFR–çFò†æVvF—fR–BÂ&VGV6VB—’À¢òò&W6W'f–ærF÷'VRF—&V7F–öââBævÆT7W'&VçBÒ’ó"F†—2v—fW2–BÒ ¢òòæB—Væ6†ævVBÂ6òF†RVævvR—26VÖÆW72à¢—5&VbÒÖ÷F÷"ÓçG$d4ÂÓç•ö—ç&Vc°¢—4ÖrÒf'6b„—5&Vb“°¢—56–vâÒ„—5&VbãÒãb’òãb¢Óãc° ¢–BÒ—4Ör¢6÷6b†ævÆT7W'&VçB“²òòÃÒ–âf–VÆBvV¶Væ–æp¢—Ò—4Ör¢6–æb†ævÆT7W'&VçB“²òòãÒ  ¢òò†&B6fWG’6Æ×öâF†R–æ¦V7FVBæVvF—fR–Bà¢–DÖ„æVrÒgv4–DÖ„æVuU¶Ö÷F÷$–G…Ó° ¢–b„–BÂÔ–DÖ„æVr¢°¢–BÒÔ–DÖ„æVs°¢Ğ¢VÇ6R–b„–Bâãb¢°¢–BÒãc°¢Ğ ¢Ö÷F÷"Óç•ö–Bç&VbÒ–C°¢Ö÷F÷"ÓçG$d4ÂÓç•ö—ç&VbÒ—56–vâ¢—°¢gv4–E&VeU¶Ö÷F÷$–G…ÒÒ–C° ¢&WGW&ã°§Ğ