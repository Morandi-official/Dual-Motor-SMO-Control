//#############################################################################
//
// FILE:    dual_axis_servo_dirve.c
//
// TITLE:   dual-axis motor drive on the related kits
//
// Group:   C2000
//
// Target Family: F2837x/F28004x
//
//#############################################################################
// $TI Release: MotorControl SDK v2.01.00.00 $
// $Release Date: Mon Nov 11 15:18:10 CST 2019 $
// $Copyright:
// Copyright (C) 2017-2019 Texas Instruments Incorporated
//
//     http://www.ti.com/ ALL RIGHTS RESERVED
// $
//#############################################################################

//
// includes
//
#include "dual_axis_servo_drive_settings.h"
#include "dual_axis_servo_drive_user.h"
#include "dual_axis_servo_drive_hal.h"

#include "dual_axis_servo_drive.h"

#include "fcl_cpu_cla_dm.h"
#include "fcl_enum.h"

#include "sfra_settings.h"

//
// Instrumentation code for timing verifications
// display variable A (in pu) on DAC
//
#define  DAC_MACRO_PU(A)  ((1.0 + A)*2048)

//
// Functions
//
#ifdef _FLASH
#ifndef __cplusplus
#pragma CODE_SECTION(motor1ControlISR, ".TI.ramfunc");
#pragma CODE_SECTION(motor2ControlISR, ".TI.ramfunc");
#endif
#endif


//
//  Prototype statements for Local Functions
//
#pragma INTERRUPT (motor1ControlISR, HPI)
#pragma INTERRUPT (motor2ControlISR, HPI)
__interrupt void motor1ControlISR(void);
__interrupt void motor2ControlISR(void);

//
// Motor drive utility functions
//

#if(BUILDLEVEL > FCL_LEVEL2)
static inline void getFCLTime(MOTOR_Num_e motorNum);
#endif
static void captureESMOCompareLog(MOTOR_Vars_t *pMotor,
                                  uint16_t motorNum);
static void clearESMOHandoffLatch(void);
static void captureESMOHandoffLatch(MOTOR_Vars_t *pMotor,
                                    uint16_t motorNum,
                                    uint16_t stage);
static void updateESMOVoltageUseMonitor(MOTOR_Vars_t *pMotor,
                                        uint16_t motorNum);
static void applyStoppedCurrentLoopVoltageLimits(MOTOR_Vars_t *pMotor,
                                                 uint16_t motorNum);
// The QEP diagnostic helpers below are intentionally NOT inline: they are
// called from both motor ISRs and inlining duplicates several hundred words
// of code into the already-tight .TI.ramfunc/ISR code space.
static void resetESMOQepReference(MOTOR_Vars_t *pMotor,
                                  uint16_t motorNum);
static void updateESMOQepReference(MOTOR_Vars_t *pMotor,
                                   uint16_t motorNum);
static void updateESMOQepIndexReference(MOTOR_Vars_t *pMotor,
                                        uint16_t motorNum);

#pragma FUNC_CANNOT_INLINE(captureESMOCompareLog);
#pragma FUNC_CANNOT_INLINE(updateESMOVoltageUseMonitor);
#pragma FUNC_CANNOT_INLINE(resetESMOQepReference);
#pragma FUNC_CANNOT_INLINE(updateESMOQepReference);
#pragma FUNC_CANNOT_INLINE(updateESMOQepIndexReference);

//
// SFRA utility functions
//
#if(BUILDLEVEL == FCL_LEVEL6)
void injectSFRA(void);
void collectSFRA(MOTOR_Vars_t *pMotor);
#endif

//
// State Machine function prototypes
//

// Alpha states
void A0(void);  //state A0
void B0(void);  //state B0
void C0(void);  //state C0

// A branch states
void A1(void);  //state A1
void A2(void);  //state A2
void A3(void);  //state A3

// B branch states
void B1(void);  //state B1
void B2(void);  //state B2
void B3(void);  //state B3

// C branch states
void C1(void);  //state C1
void C2(void);  //state C2
void C3(void);  //state C3

// Variable declarations
void (*Alpha_State_Ptr)(void);  // Base States pointer
void (*A_Task_Ptr)(void);       // State pointer A branch
void (*B_Task_Ptr)(void);       // State pointer B branch
void (*C_Task_Ptr)(void);       // State pointer C branch

uint16_t vTimer0[4] = {0};  // Virtual Timers slaved off CPU Timer 0 (A events)
uint16_t vTimer1[4] = {0};  // Virtual Timers slaved off CPU Timer 1 (B events)
uint16_t vTimer2[4] = {0};  // Virtual Timers slaved off CPU Timer 2 (C events)
uint16_t serialCommsTimer = 0;

//
// USER Variables
//

//
// Global variables used in this system
//
MOTOR_Vars_t motorVars[2] = {MOTOR1_DEFAULTS, MOTOR2_DEFAULTS};

// ������˫����ٶ�ͬ��Эͬ PI ������
// Dual-motor speed synchronization PI. Its output is a differential Iq
// compensation in pu.
PID_CONTROLLER pid_sync_spd = {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS};
float32_t syncPI_out = 0.0;
float32_t syncSpdErr = 0.0;
uint16_t syncPIActive = 0;

#pragma DATA_SECTION(motorVars, "ClaData");

//
// Variables for current measurement
//

//
// CMPSS parameters for Over Current Protection
//
uint16_t clkPrescale = 20;
uint16_t sampWin     = 30;
uint16_t thresh      = 18;

//
// Flag variables
//
volatile uint16_t enableFlag = false;

uint16_t backTicker = 0;

uint16_t led1Cnt = 0;
uint16_t led2Cnt = 0;

// Variables for Field Oriented Control
float32_t VdTesting = 0.0;          // Vd reference (pu)
float32_t VqTesting = 0.10;         // Vq reference (pu)

// Variables for position reference generation and control
float32_t posArray[8] = {2.5, -2.5, 3.5, -3.5, 5.0, -5.0, 8.0, -8.0};
float32_t posPtrMax = 4;

// Variables for Datalog module
float32_t DBUFF_4CH1[200] = {0};
float32_t DBUFF_4CH2[200] = {0};
float32_t DBUFF_4CH3[200] = {0};
float32_t DBUFF_4CH4[200] = {0};
float32_t dlogCh1 = 0;
float32_t dlogCh2 = 0;
float32_t dlogCh3 = 0;
float32_t dlogCh4 = 0;

// Create an instance of DATALOG Module
DLOG_4CH_F dlog_4ch1;

// Variables for SFRA module
#if(BUILDLEVEL == FCL_LEVEL6)
extern SFRA_F32 sfra1;
SFRATest_e      sfraTestLoop = SFRA_TEST_D_AXIS;  //speedLoop;
uint32_t        sfraCollectStart = 0;
float32_t       sfraNoiseD = 0;
float32_t       sfraNoiseQ = 0;
float32_t       sfraNoiseW = 0;
#endif

HAL_Handle    halHandle;    //!< the handle for the hardware abstraction layer
HAL_Obj       hal;          //!< the hardware abstraction layer object

HAL_MTR_Handle halMtrHandle[2];   //!< the handle for the hardware abstraction
                                  //!< layer to motor control
HAL_MTR_Obj    halMtr[2];         //!< the hardware abstraction layer object
                                  //!< to motor control

// FCL Latency variables
volatile uint16_t FCL_cycleCount[2];

// eSMO startup transient capture. Disabled while high-rate compare log is used.
#define ESMO_STARTUP_LOG_ENABLE     0U
#define ESMO_STARTUP_LOG_SIZE       ((ESMO_STARTUP_LOG_ENABLE != 0U) ? 96U : 1U)
#define ESMO_STARTUP_LOG_RATE_HZ    12U
#define ESMO_STARTUP_LOG_DECIMATION (M1_SAMPLING_FREQ / ESMO_STARTUP_LOG_RATE_HZ)

typedef struct _ESMO_StartupLog_t_
{
    uint16_t lsw;
    uint16_t flags;             // bit0 runMotor, bit1 tripFlagDMC
    int16_t rcSetpoint_q15;
    int16_t rampAngle_q15;
    int16_t speedFbk_q15;
    int16_t esmoSpeed_q15;
    int16_t qepSpeed_q15;
    int16_t esmoAngle_q15;
    int16_t esmoAngleErr_q15;
    int16_t qepAngle_q15;
    int16_t eqMag_q15;
    int16_t thetaErr_q15;
    int16_t pllOut_q15;
    int16_t piIqRef_q15;
    int16_t piIqFbk_q15;
} ESMO_StartupLog_t;

volatile ESMO_StartupLog_t esmoStartupLog[ESMO_STARTUP_LOG_SIZE];
volatile uint16_t esmoStartupLogMotor = 0;
volatile uint16_t esmoStartupLogArmed = 1;
volatile uint16_t esmoStartupLogActive = 0;
volatile uint16_t esmoStartupLogDone = 0;
volatile uint16_t esmoStartupLogIndex = 0;
volatile uint16_t esmoStartupLogDecimation = ESMO_STARTUP_LOG_DECIMATION;
volatile uint16_t esmoStartupLogDecimCnt = 0;
volatile uint16_t esmoStartupLogPrevRun = MOTOR_STOP;
volatile float32_t esmoStartupPeakSpeedPu = 0.0f;
volatile float32_t esmoStartupPeakIqRef = 0.0f;
volatile float32_t esmoStartupPeakIqFbk = 0.0f;
volatile float32_t esmoStartupPeakEqMag = 0.0f;
volatile float32_t esmoStartupPeakSpeedLsw1Pu = 0.0f;
volatile float32_t esmoStartupPeakIqRefLsw1 = 0.0f;
volatile float32_t esmoStartupPeakIqFbkLsw1 = 0.0f;
volatile float32_t esmoStartupPeakEqMagLsw1 = 0.0f;
volatile float32_t esmoStartupPeakSpeedTakeoverPu = 0.0f;
volatile float32_t esmoStartupPeakIqRefTakeover = 0.0f;
volatile float32_t esmoStartupPeakIqFbkTakeover = 0.0f;
volatile float32_t esmoStartupPeakEqMagTakeover = 0.0f;
volatile float32_t esmoStartupPeakSpeedClosedPu = 0.0f;
volatile float32_t esmoStartupPeakIqRefClosed = 0.0f;
volatile float32_t esmoStartupPeakIqFbkClosed = 0.0f;
volatile float32_t esmoStartupPeakEqMagClosed = 0.0f;

// Keep the M1-only logger disarmed during normal dual-motor operation. At
// 500Hz, 96 entries cover 192ms without adding continuous ISR work.
#define ESMO_COMPARE_LOG_ENABLE      1U
#define ESMO_COMPARE_LOG_SIZE        96U
#define ESMO_COMPARE_LOG_RATE_HZ     500U
#define ESMO_COMPARE_LOG_DECIMATION  (M1_SAMPLING_FREQ / ESMO_COMPARE_LOG_RATE_HZ)
#define ESMO_COMPARE_REQUIRE_QEP_INDEX  1U
#define ESMO_COMPARE_LOG_M2_ENABLE   0U
// Debug experiment: seed the eSMO from the QEP angle at handoff and skip
// the takeover blending. Disabled: the instant FOC-angle jump applies any
// seed/PLL residual in one step and can reverse the torque until the PLL
// re-locks. With the QEP frame offset fix in place this experiment is no
// longer needed; both motors use the proven rg.Out + blending handoff.
#define ESMO_DEBUG_QEP_HANDOFF_SYNC  0U
#define ESMO_QEP_INDEX_SYNC_FLAG     0x00F0U
#define ESMO_QEP_IEI_RISING          0x0200U
#define ESMO_QEP_IEI_MASK            0x0300U
#define ESMO_QEP_IEL_FLAG            0x0400U   // QFLG/QCLR index event latch

// Handoff transient latch. The STOP/RUN handoff diagnosis is complete, so
// this is disabled by default to save scarce ISR code/data RAM. Set to 1U
// to re-enable the capture; the symbols stay available for CCS watch.
#define ESMO_HANDOFF_LATCH_ENABLE         0U

#define ESMO_HANDOFF_LATCH_PRE_SYNC       0U
#define ESMO_HANDOFF_LATCH_POST_SYNC      1U
#define ESMO_HANDOFF_LATCH_TAKEOVER_DONE  2U
#define ESMO_HANDOFF_LATCH_COUNT          3U
#define ESMO_HANDOFF_LATCH_ALLOC  ((ESMO_HANDOFF_LATCH_ENABLE != 0U) ? \
                                   ESMO_HANDOFF_LATCH_COUNT : 1U)

typedef struct _ESMO_CompareLog_t_
{
    uint16_t lsw;
    uint16_t flags;             // bit0 runMotor, bit1 tripFlagDMC, bit2 takeover,
                                // bit3 QEP index calibrated
    int16_t rcSetpoint_q15;
    int16_t esmoAngle_q15;      // final eSMO angle used by FOC
    int16_t qepAngle_q15;
    int16_t angleErr_q15;
    int16_t esmoSpeed_q15;
    int16_t qepSpeed_q15;
    int16_t iqRef_q15;
    int16_t iqFbk_q15;
    int16_t idFbk_q15;
    int16_t vdOut_q15;
    int16_t vqOut_q15;
    int16_t vsSq_q15;           // d/q PI-limit normalized use^2
    int16_t vsLimitSq_q15;      // fixed 1.0; vsSq/vsLimitSq = limit use
    int16_t eqMag_q15;
    int16_t thetaErr_q15;
} ESMO_CompareLog_t;

typedef struct _ESMO_HandoffLatch_t_
{
    uint16_t valid;
    uint16_t stage;
    uint16_t motorNum;
    uint16_t lsw;
    uint16_t takeoverCnt;
    int16_t rgAngle_q15;
    int16_t rawAngle_q15;
    int16_t finalAngle_q15;
    int16_t qepAngle_q15;
    int16_t angleErr_q15;
    int16_t esmoSpeed_q15;
    int16_t iqRef_q15;
} ESMO_HandoffLatch_t;

#pragma DATA_SECTION(esmoCompareLog, "ESMO_COMPARE_LOG_DATA")
volatile ESMO_CompareLog_t esmoCompareLog[ESMO_COMPARE_LOG_SIZE];
volatile uint16_t esmoCompareLogMotor = 0;
volatile uint16_t esmoCompareLogArmed = 0U;
volatile uint16_t esmoCompareLogActive = 0;
volatile uint16_t esmoCompareLogDone = 0;
volatile uint16_t esmoCompareLogIndex = 0;
volatile uint16_t esmoCompareLogDecimation = ESMO_COMPARE_LOG_DECIMATION;
volatile uint16_t esmoCompareLogDecimCnt = 0;
volatile uint16_t esmoCompareRequireQepIndex = ESMO_COMPARE_REQUIRE_QEP_INDEX;
volatile float32_t esmoCompareTriggerCommandPu = 0.82f;
volatile float32_t esmoCompareTriggerSpeedPu = 0.78f;

volatile ESMO_HandoffLatch_t
        esmoHandoffLatch[ESMO_HANDOFF_LATCH_ALLOC];
volatile uint16_t esmoHandoffLatchValidMask = 0U;

volatile float32_t esmoQepSpeedPu[2] = {0.0f, 0.0f};
volatile float32_t esmoQepAngleErrPu[2] = {0.0f, 0.0f};
volatile float32_t esmoQepSpeedErrPu[2] = {0.0f, 0.0f};
volatile uint16_t esmoQepIndexCalibrated[2] = {0U, 0U};
volatile uint32_t esmoQepIndexCount[2] = {0U, 0U};

// Sensorless speed-loop feedback mirror. In the official UMC eSMO path the
// speed used by the speed loop is obtained by differentiating the compensated
// eSMO PLL angle through SPDFR. In this FCL project esmoRawAnglePu is the
// equivalent compensated PLL angle; esmoAnglePu is the final FOC angle after
// startup handoff/offset logic and must not be differentiated for speed.
volatile float32_t esmoSpeedLoopFbkPu[2] = {0.0f, 0.0f};

// Real-time voltage utilization monitor for CCS Watch. These values are
// diagnostic only and do not alter the control path. The d/q PI output limits
// are intentionally asymmetric in this FCL project, so the useful saturation
// indicator is the per-axis normalized limit use, not a circular limit formed
// from the smaller of the two limits.
volatile float32_t esmoVdOutPu[2];
volatile float32_t esmoVqOutPu[2];
volatile float32_t esmoVsSqPu[2];
volatile float32_t esmoVsLimitSqPu[2];
volatile float32_t esmoVsUseSqPct[2];
volatile float32_t esmoVsMarginSqPct[2] = {100.0f, 100.0f};
volatile float32_t esmoVdUsePct[2];
volatile float32_t esmoVqUsePct[2];
volatile float32_t esmoPiLimitUsePct[2];
volatile float32_t esmoPiVectorUseSqPct[2];
volatile float32_t esmoModUsePct[2];
volatile float32_t esmoModMarginPct[2] = {100.0f, 100.0f};
volatile float32_t esmoIdPiLimitSf[2] = {0.75f, 0.75f};
volatile float32_t esmoIqPiLimitSf[2] = {0.80f, 0.80f};
volatile float32_t esmoPiVectorBudgetSf[2] = {0.943398f, 0.943398f};
volatile float32_t esmoIdPiAppliedSf[2] = {0.0f, 0.0f};
volatile float32_t esmoIqPiAppliedSf[2] = {0.0f, 0.0f};
volatile float32_t esmoIdPiLimitPu[2] = {0.0f, 0.0f};
volatile float32_t esmoIqPiLimitPu[2] = {0.0f, 0.0f};
volatile float32_t esmoRunIqLimitPu[2] = {0.0f, 0.0f};
volatile uint16_t esmoPiLimitScaledFlag[2] = {0U, 0U};
volatile uint16_t esmoPiLimitTightFlag[2] = {0U, 0U};
volatile uint16_t esmoPiLimitOverFlag[2] = {0U, 0U};
volatile uint16_t esmoVoltageTightFlag[2] = {0U, 0U};
volatile uint16_t esmoVoltageOverFlag[2] = {0U, 0U};
volatile uint16_t esmoVoltageMainAxis[2] = {0U, 0U};

// Electrical angle (pu) held by the FOC at the moment QPOSCNT was zeroed at
// the end of alignment. rg.Out is frozen at a run-dependent value across
// STOP/RUN cycles, so the alignment position is NOT the stator-frame zero.
// Adding this offset puts the diagnostic QEP angle back into the same
// absolute stator frame that the eSMO estimates, which makes angleErr
// comparable from run to run.
volatile float32_t esmoQepFrameOffsetPu[2] = {0.0f, 0.0f};

// Per-run QEP diagnostic frame. STOP/RUN tests showed that keeping the first
// frame captured after power-up made later runs depend on stale incremental
// QEP state. The frame is therefore rebuilt from the next alignment after
// every STOP so the QEP diagnostic angle follows the same zeroing convention
// as the eSMO/FOC startup used in that run.
volatile uint16_t esmoQepFrameCaptured[2] = {0U, 0U};

// Index-based frame correction. The per-run alignment zero carries the
// alignment settling residual (friction/cogging leaves the rotor several
// degrees off the commanded angle, randomly per run) - hardware data shows
// the eSMO's true bias is run-invariant while the measured angleErr scatters
// by exactly that residual. The physical index pulse sits at a fixed
// electrical angle (independent of which mechanical equivalent the rotor
// aligned to, as long as 4*lines/polePairs is an integer), so its electrical
// angle is calibrated ONCE on the first run after power-up and every later
// run shifts its frame so the index lands on the calibrated angle. This
// cancels the alignment residual to within one encoder count.
// Set esmoQepIndexCalValid[motor]=0 from CCS to force a re-calibration.
volatile float32_t esmoQepIndexAngleCal[2] = {0.0f, 0.0f};
volatile uint16_t esmoQepIndexCalValid[2] = {0U, 0U};
volatile uint16_t esmoQepIndexSeen[2] = {0U, 0U};

// Absolute index-angle anchor. By default (UseFixed = 0) the index
// electrical angle is auto-captured on the FIRST run after each power-up,
// so it carries that run's alignment settling residual - which makes the
// diagnostic QEP frame, and therefore the absolute angleErr, shift a few
// degrees from one power-up to the next (verified: ~7 deg between the
// 06-13 14:45 and 15:24 sessions). For trustworthy CROSS-SESSION angle
// quantification, measure esmoQepIndexAngleCal over several power-ups,
// average it, write that constant into esmoQepIndexAngleFixed[motor] and
// set esmoQepIndexUseFixed[motor] = 1: every session then anchors its
// frame to the same absolute index angle (to within one encoder count),
// so any residual session-to-session angleErr change is genuine eSMO
// drift (thermal / operating point), not a measurement-frame artifact.
//
// M1 value 0.8987 = mean of three 0613 power-ups (0.897/0.903/0.896,
// spread 2.5 elec deg), measured at 0.3pu. M1 anchoring is ON. M2 is still
// auto (UseFixed 0); measure esmoQepIndexAngleCal[1] over a few power-ups
// and fill esmoQepIndexAngleFixed[1] before enabling it.
volatile float32_t esmoQepIndexAngleFixed[2] = {0.8987f, 0.933f};
volatile uint16_t esmoQepIndexUseFixed[2] = {1U, 0U};
static uint16_t esmoQepRefValid[2] = {0U, 0U};
static uint16_t esmoQepColdResetDone[2] = {0U, 0U};
static SPEED_MEAS_QEP esmoQepSpeedForCompare[2] =
{
    SPEED_MEAS_QEP_DEFAULTS,
    SPEED_MEAS_QEP_DEFAULTS
};

// control dual motor with the same speed and acceleration at the same time
float32_t speedRef = 0.1;
float32_t IdRef = 0.0;
float32_t IqRef = 0.10;
MotorRunStop_e runMotor = MOTOR_STOP;
uint32_t rampDelayMax = 0;
bool flagSyncRun = false;
bool flagSyncPI = false;
//
// These are defined by the linker file
//
extern uint32_t Cla1funcsLoadStart;
extern uint32_t Cla1funcsLoadEnd;
extern uint32_t Cla1funcsRunStart;
extern uint32_t Cla1funcsLoadSize;

//
// main() function enter
//
void main(void)
{
    // initialize device clock and peripherals
    Device_init();

    // initialize the driver
    halHandle = HAL_init(&hal, sizeof(hal));

    // initialize the driver for motor 1
    halMtrHandle[MTR_1] =
            HAL_MTR_init(&halMtr[MTR_1], sizeof(halMtr[MTR_1]));

    // initialize the driver for motor 1
    halMtrHandle[MTR_2] =
            HAL_MTR_init(&halMtr[MTR_2], sizeof(halMtr[MTR_2]));

    // set the driver parameters
    HAL_setParams(halHandle);

    // set the driver parameters for motor 1
    HAL_setMotorParams(halMtrHandle[MTR_1]);

    // set the driver parameters for motor 2
    HAL_setMotorParams(halMtrHandle[MTR_2]);

    // PWM Clocks Enable
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // initialize motor parameters for motor_1
    initMotorParameters(&motorVars[0], halMtrHandle[0]);

    // initialize motor parameters for motor_2
    initMotorParameters(&motorVars[1], halMtrHandle[1]);

    // initialize motor control variables for motor_1
    initControlVars(&motorVars[0]);

    // initialize motor control variables for motor_2
    initControlVars(&motorVars[1]);

    applyStoppedCurrentLoopVoltageLimits(&motorVars[0], MTR_1);
    applyStoppedCurrentLoopVoltageLimits(&motorVars[1], MTR_2);

    // ��ʼ��Эͬ�ٶ� PI ���������� (����ֵ�ɸ���ʵ���������)
    // Speed synchronization PI. Keep flagSyncPI disabled until both eSMO
    // loops are verified closed and stable on hardware.
    pid_sync_spd.param.Kp   = 0.05;
    pid_sync_spd.param.Ki   = 0.0;
    pid_sync_spd.param.Kd   = 0.0;
    pid_sync_spd.param.Kr   = 1.0;
    pid_sync_spd.param.Umax = 0.05;
    pid_sync_spd.param.Umin = -0.05;

    motorVars[0].curLimit = 6.0;        // 6A
    motorVars[1].curLimit = 6.0;        // 6A


    // setup faults protection for motor_1
    HAL_setupMotorFaultProtection(halMtrHandle[MTR_1],
                                  motorVars[MTR_1].curLimit);

    // setup faults protection for motor_2
    HAL_setupMotorFaultProtection(halMtrHandle[MTR_2],
                                  motorVars[MTR_2].curLimit);

// Note that the vectorial sum of d-q PI outputs should be less than 1.0 which
// refers to maximum duty cycle for SVGEN. Another duty cycle limiting factor
// is current sense through shunt resistors which depends on hardware/software
// implementation. Depending on the application requirements 3,2 or a single
// shunt resistor can be used for current waveform reconstruction. The higher
// number of shunt resistors allow the higher duty cycle operation and better
// dc bus utilization. The users should adjust the PI saturation levels
// carefully during open loop tests (i.e pi_id.Umax, pi_iq.Umax and Umins) as
// in project manuals. Violation of this procedure yields distorted  current
// waveforms and unstable closed loop operations that may damage the inverter.
    // reset some control variables for motor_1
    resetControlVars(&motorVars[0]);

    // reset some control variables for motor_2
    resetControlVars(&motorVars[1]);

    // clear any spurious OST & DCAEVT1 flags for motor_1
    HAL_clearTZFlag(halMtrHandle[MTR_1]);

    // clear any spurious OST & DCAEVT1 flags for motor_2
    HAL_clearTZFlag(halMtrHandle[MTR_2]);

    // Clear LED counter
    led1Cnt = 0;
    led2Cnt = 0;

    // Waiting for enable flag set
    while(enableFlag == false)
    {
        backTicker++;
    }

    //find out the FCL SW version information
    while(FCL_getSwVersion() != 0x00000008)
    {
        backTicker++;
    }

    // *************** SFRA & SFRA_GUI COMM INIT CODE START *******************
#if BUILDLEVEL == FCL_LEVEL6
    // ************************************************************************
    // NOTE:
    // =====
    // In configureSFRA() below, use 'SFRA_GUI_PLOT_GH_H' to get open loop and
    // plant Bode plots using SFRA_GUI and open loop and closed loop Bode plots
    // using SFRA_GUI_MC. 'SFRA_GUI_PLOT_GH_CL' gives same plots for both GUIs.
    // The CL plot inferences shown in SFRA_GUI is not according to
    // NEMA ICS16 or GBT-16439-2009, so it is not recommended for bandwidth
    // determination purposes in servo drive evaluations. Use SFRA_GUI_MC for
    // that. Recommended to use the default setting (SFRA_GUI_PLOT_GH_H).
    // ************************************************************************
    //
    // configure the SFRA module. SFRA module and settings found in
    // sfra_gui.c/.h
    //
#if SFRA_MOTOR == MOTOR_1
    // Plot GH & H plots using SFRA_GUI, GH & CL plots using SFRA_GUI_MC
    configureSFRA(SFRA_GUI_PLOT_GH_H, M1_SAMPLING_FREQ);
#endif

#if SFRA_MOTOR == MOTOR_2
    // Plot GH & H plots using SFRA_GUI, GH & CL plots using SFRA_GUI_MC
    configureSFRA(SFRA_GUI_PLOT_GH_H, M2_SAMPLING_FREQ);
#endif

#endif
    // **************** SFRA & SFRA_GUI COMM INIT CODE END ********************

    // Tasks State-machine init
    Alpha_State_Ptr = &A0;
    A_Task_Ptr = &A1;
    B_Task_Ptr = &B1;
    C_Task_Ptr = &C1;

    // Set up the initialization value for some variables
    motorVars[0].state = 0;
    motorVars[0].IdRef_start = M1_STARTUP_ID_REF;
    motorVars[0].IqRef = M1_STARTUP_IQ_REF;
    motorVars[0].speedRef = 0.05;
    motorVars[0].lsw1Speed = 0.02;

    motorVars[0].posPtr = 0;
    motorVars[0].posPtrMax = posPtrMax;
    motorVars[0].posCntrMax = 5000;
    motorVars[0].posSlewRate =  0.001;
    motorVars[0].fclClrCntr = 1;

    motorVars[1].state = 0;
    motorVars[1].IdRef_start = M2_STARTUP_ID_REF;
    motorVars[1].IqRef = M2_STARTUP_IQ_REF;
    motorVars[1].speedRef = 0.05;
    motorVars[1].lsw1Speed = 0.02;

    motorVars[1].posPtr = 0;
    motorVars[1].posPtrMax = posPtrMax;
    motorVars[1].posCntrMax = 5000;
    motorVars[1].posSlewRate =  0.001;
    motorVars[1].fclClrCntr = 1;

//
// Initialize Datalog module for motor 1 or motor 2
//
    DLOG_4CH_F_init(&dlog_4ch1);
    dlog_4ch1.input_ptr1 = &dlogCh1;    //data value
    dlog_4ch1.input_ptr2 = &dlogCh2;
    dlog_4ch1.input_ptr3 = &dlogCh3;
    dlog_4ch1.input_ptr4 = &dlogCh4;
    dlog_4ch1.output_ptr1 = &DBUFF_4CH1[0];
    dlog_4ch1.output_ptr2 = &DBUFF_4CH2[0];
    dlog_4ch1.output_ptr3 = &DBUFF_4CH3[0];
    dlog_4ch1.output_ptr4 = &DBUFF_4CH4[0];
    dlog_4ch1.size = 200;
    dlog_4ch1.pre_scalar = 5;
    dlog_4ch1.trig_value = 0.01;
    dlog_4ch1.status = 2;

    // Configure interrupt for motor_1
    HAL_setupInterrupts(halMtrHandle[MTR_1]);

    // Configure interrupt for motor_2
    HAL_setupInterrupts(halMtrHandle[MTR_2]);

    // current feedback offset calibration for motor_1
    runOffsetsCalculation(&motorVars[0]);

    // current feedback offset calibration for motor_1
    runOffsetsCalculation(&motorVars[1]);

    // Configure interrupt for motor_1
    HAL_enableInterrupts(halMtrHandle[MTR_1]);

    // Configure interrupt for motor_2
    HAL_enableInterrupts(halMtrHandle[MTR_2]);

    //Clear the latch flag
    motorVars[0].clearTripFlagDMC = 1;
    motorVars[1].clearTripFlagDMC = 1;

    // enable global interrupt
    EINT;          // Enable Global interrupt INTM

    ERTM;          // Enable Global realtime interrupt DBGM

    //
    // Initializations COMPLETE
    //  - IDLE loop. Just loop forever
    //
    for(;;)  //infinite loop
    {
        // State machine entry & exit point
        //===========================================================
        (*Alpha_State_Ptr)();   // jump to an Alpha state (A0,B0,...)
        //===========================================================
    }
} //END MAIN CODE

//=============================================================================
//  STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//=============================================================================

//--------------------------------- FRAMEWORK ---------------------------------
void A0(void)
{
    // loop rate synchronizer for A-tasks
    if(CPUTimer_getTimerOverflowStatus(CPUTIMER0_BASE))
    {
        CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);  // clear flag

        //-----------------------------------------------------------
        (*A_Task_Ptr)();        // jump to an A Task (A1,A2,A3,...)
        //-----------------------------------------------------------

        vTimer0[0]++;           // virtual timer 0, instance 0 (spare)
        serialCommsTimer++;
    }

    Alpha_State_Ptr = &B0;      // Comment out to allow only A tasks
}

void B0(void)
{
    // loop rate synchronizer for B-tasks
    if(CPUTimer_getTimerOverflowStatus(CPUTIMER1_BASE))
    {
        CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);  // clear flag

        //-----------------------------------------------------------
        (*B_Task_Ptr)();        // jump to a B Task (B1,B2,B3,...)
        //-----------------------------------------------------------
        vTimer1[0]++;           // virtual timer 1, instance 0 (spare)
    }

    Alpha_State_Ptr = &C0;      // Allow C state tasks
}

void C0(void)
{
    // loop rate synchronizer for C-tasks
    if(CPUTimer_getTimerOverflowStatus(CPUTIMER2_BASE))
    {
        CPUTimer_clearOverflowFlag(CPUTIMER2_BASE);  // clear flag

        //-----------------------------------------------------------
        (*C_Task_Ptr)();        // jump to a C Task (C1,C2,C3,...)
        //-----------------------------------------------------------

        vTimer2[0]++;           //virtual timer 2, instance 0 (spare)
    }

    Alpha_State_Ptr = &A0;  // Back to State A0
}

//==============================================================================
//  A - TASKS (executed in every 50 usec)
//==============================================================================

//--------------------------------------------------------
void A1(void) // SPARE (not used)
//--------------------------------------------------------
{
    applyStoppedCurrentLoopVoltageLimits(&motorVars[0], MTR_1);

    // motor_1 running logic control
    runMotorControl(&motorVars[0], halMtrHandle[0]);

    //-------------------
    //the next time CpuTimer0 'counter' reaches Period value go to A2
    A_Task_Ptr = &A2;
    //-------------------
}

//-----------------------------------------------------------------
void A2(void) // SPARE (not used)
//-----------------------------------------------------------------
{
    applyStoppedCurrentLoopVoltageLimits(&motorVars[1], MTR_2);

    // motor_2 running logic control
    runMotorControl(&motorVars[1], halMtrHandle[1]);

    //-------------------
    //the next time CpuTimer0 'counter' reaches Period value go to A3
    A_Task_Ptr = &A3;
    //-------------------
}

//-----------------------------------------
void A3(void) // SPARE (not used)
//-----------------------------------------
{
    led1Cnt++;

    if(led1Cnt >= LPD_LED1_WAIT_TIME)
    {
        led1Cnt = 0;

        GPIO_togglePin(LPD_RED_LED1);   // LED
    }


    //-----------------
    //the next time CpuTimer0 'counter' reaches Period value go to A1
    A_Task_Ptr = &A1;
    //-----------------
}

//==============================================================================
//  B - TASKS (executed in every 100 usec)
//==============================================================================

//----------------------------------- USER -------------------------------------

//----------------------------------------
void B1(void) // Toggle GPIO-00
//----------------------------------------
{
#if BUILDLEVEL == FCL_LEVEL6
    //
    // SFRA test
    //
    SFRA_F32_runBackgroundTask(&sfra1);
    SFRA_GUI_runSerialHostComms(&sfra1);

#endif

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B2
    B_Task_Ptr = &B2;
    //-----------------
}

//----------------------------------------
void B2(void) // SPARE
//----------------------------------------
{

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B3
    B_Task_Ptr = &B3;
    //-----------------
}

//----------------------------------------
void B3(void) // SPARE
//----------------------------------------
{

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B1
    B_Task_Ptr = &B1;
    //-----------------
}

//==============================================================================
//  C - TASKS (executed in every 150 usec)
//==============================================================================

//--------------------------------- USER ---------------------------------------

//----------------------------------------
void C1(void)   // Toggle GPIO-34
//----------------------------------------
{
    led2Cnt++;

    if(led2Cnt >= LPD_LED2_WAIT_TIME)
    {
        led2Cnt = 0;

        GPIO_togglePin(LPD_BLUE_LED2);   // LED
    }

    //-----------------
    //the next time CpuTimer2 'counter' reaches Period value go to C2
    C_Task_Ptr = &C2;

    //-----------------

}

//----------------------------------------
void C2(void) // SPARE
//----------------------------------------
{

    //-----------------
    //the next time CpuTimer2 'counter' reaches Period value go to C3
    C_Task_Ptr = &C3;
    //-----------------
}

//-----------------------------------------
void C3(void) // SPARE
//-----------------------------------------
{

    //-----------------
    //the next time CpuTimer2 'counter' reaches Period value go to C1
    C_Task_Ptr = &C1;
    //-----------------
}


// ****************************************************************************
// Get FCL timing details - time stamp taken in library after PWM update
// ****************************************************************************
#if(BUILDLEVEL > FCL_LEVEL2)
#pragma FUNC_ALWAYS_INLINE(getFCLTime)

static inline void getFCLTime(MOTOR_Num_e motorNum)
{
    // SETGPIO18_HIGH; // only for debug

    if(EPWM_getTimeBaseCounterValue(halMtrHandle[motorNum]->pwmHandle[0]) <
            FCL_cycleCount[motorNum])
    {
        FCL_cycleCount[motorNum] =
                EPWM_getTimeBasePeriod(halMtrHandle[motorNum]->pwmHandle[0]) -
                FCL_cycleCount[motorNum];
    }

    if(motorVars[motorNum].fclCycleCountMax < FCL_cycleCount[motorNum])
    {
        motorVars[motorNum].fclCycleCountMax =
                FCL_cycleCount[motorNum];
    }

    if(motorVars[motorNum].fclClrCntr)
    {
        motorVars[motorNum].fclCycleCountMax = 0;
        motorVars[motorNum].fclClrCntr = 0;
    }

    //for 100MHz PWM clock
    motorVars[motorNum].fclLatencyInMicroSec =
            (motorVars[motorNum].fclCycleCountMax) * 0.01;

    // SETGPIO18_LOW;  // only for debug

    return;
}
#endif

static inline int16_t q15Log(float32_t value)
{
    return((int16_t)(__fsat(value, 0.999969f, -1.0f) * 32767.0f));
}

static inline float32_t absF32(float32_t value)
{
    return((value >= 0.0f) ? value : -value);
}

static void applyStoppedCurrentLoopVoltageLimits(MOTOR_Vars_t *pMotor,
                                                 uint16_t motorNum)
{
    float32_t idScale;
    float32_t iqScale;
    float32_t idLimit;
    float32_t iqLimit;

    if((motorNum > MTR_2) || (pMotor->runMotor == MOTOR_RUN))
    {
        return;
    }

    idScale = __fsat(esmoIdPiLimitSf[motorNum], 0.95f, 0.10f);
    iqScale = __fsat(esmoIqPiLimitSf[motorNum], 0.95f, 0.10f);
    idLimit = idScale * pMotor->maxModIndex;
    iqLimit = iqScale * pMotor->maxModIndex;

    pMotor->pi_id.Umax = idLimit;
    pMotor->pi_id.Umin = -idLimit;
    pMotor->ptrFCL->pi_iq.Umax = iqLimit;
    pMotor->ptrFCL->pi_iq.Umin = -iqLimit;

    esmoIdPiAppliedSf[motorNum] = idScale;
    esmoIqPiAppliedSf[motorNum] = iqScale;
    esmoIdPiLimitPu[motorNum] = idLimit;
    esmoIqPiLimitPu[motorNum] = iqLimit;
    esmoPiLimitScaledFlag[motorNum] = 0U;
}

static void updateESMOVoltageUseMonitor(MOTOR_Vars_t *pMotor,
                                        uint16_t motorNum)
{
    float32_t vdOut = pMotor->pi_id.out;
    float32_t vqOut = pMotor->ptrFCL->pi_iq.out;
    float32_t vsSq = (vdOut * vdOut) + (vqOut * vqOut);
    float32_t vdLimit = absF32(pMotor->pi_id.Umax);
    float32_t iqLimit = absF32(pMotor->ptrFCL->pi_iq.Umax);
    float32_t vdUsePct;
    float32_t vqUsePct;
    float32_t vectorUseSqPct;
    float32_t piVectorLimitSq;
    float32_t maxAxisUsePct;
    float32_t modLimit;
    float32_t modUsePct;

    if(motorNum > MTR_2)
    {
        return;
    }

    vdUsePct = (vdLimit > 0.000001f) ?
            (100.0f * absF32(vdOut) / vdLimit) : 0.0f;
    vqUsePct = (iqLimit > 0.000001f) ?
            (100.0f * absF32(vqOut) / iqLimit) : 0.0f;
    maxAxisUsePct = (vdUsePct > vqUsePct) ? vdUsePct : vqUsePct;

    modLimit = absF32(pMotor->maxModIndex);
    piVectorLimitSq = esmoPiVectorBudgetSf[motorNum] * modLimit;
    piVectorLimitSq *= piVectorLimitSq;
    vectorUseSqPct = (piVectorLimitSq > 0.000001f) ?
            (100.0f * vsSq / piVectorLimitSq) : 0.0f;
    modUsePct = (modLimit > 0.000001f) ?
            (100.0f * __sqrt(vsSq) / modLimit) : 0.0f;

    esmoVdOutPu[motorNum] = vdOut;
    esmoVqOutPu[motorNum] = vqOut;
    esmoVsSqPu[motorNum] = vsSq;
    esmoVsLimitSqPu[motorNum] = piVectorLimitSq;
    esmoVsUseSqPct[motorNum] = vectorUseSqPct;
    esmoVsMarginSqPct[motorNum] = 100.0f - vectorUseSqPct;
    esmoVdUsePct[motorNum] = vdUsePct;
    esmoVqUsePct[motorNum] = vqUsePct;
    esmoPiLimitUsePct[motorNum] = maxAxisUsePct;
    esmoPiVectorUseSqPct[motorNum] = vectorUseSqPct;
    esmoModUsePct[motorNum] = modUsePct;
    esmoModMarginPct[motorNum] = 100.0f - modUsePct;
    esmoPiLimitTightFlag[motorNum] =
            ((maxAxisUsePct >= 80.0f) ||
             (vectorUseSqPct >= 80.0f)) ? 1U : 0U;
    esmoPiLimitOverFlag[motorNum] =
            ((maxAxisUsePct >= 100.0f) ||
             (vectorUseSqPct >= 100.0f)) ? 1U : 0U;
    esmoVoltageTightFlag[motorNum] = (modUsePct >= 80.0f) ? 1U : 0U;
    esmoVoltageOverFlag[motorNum] = (modUsePct >= 100.0f) ? 1U : 0U;
    esmoVoltageMainAxis[motorNum] = (vdUsePct > vqUsePct) ? 1U : 2U;
}

static inline float32_t wrapPuHalfLocal(float32_t value)
{
    while(value > 0.5f)
    {
        value -= 1.0f;
    }

    while(value < -0.5f)
    {
        value += 1.0f;
    }

    return(value);
}

static inline float32_t normalizePuLocal(float32_t value)
{
    while(value >= 1.0f)
    {
        value -= 1.0f;
    }

    while(value < 0.0f)
    {
        value += 1.0f;
    }

    return(value);
}

static inline float32_t getSensorlessSpeedLoopFbk(MOTOR_Vars_t *pMotor,
                                                  uint16_t motorNum)
{
    float32_t anglePu;
    float32_t speedPu;

    if((motorNum > MTR_2) ||
       (pMotor->runMotor != MOTOR_RUN) ||
       (pMotor->ptrFCL->lsw != ENC_CALIBRATION_DONE) ||
       (isSensorlessTakeoverActive(pMotor) != 0U))
    {
        if(motorNum <= MTR_2)
        {
            esmoSpeedLoopFbkPu[motorNum] = pMotor->esmoSpeedPu;
        }

        anglePu = pMotor->esmoRawAnglePu;
        pMotor->speed.ElecTheta = anglePu;
        pMotor->speed.OldElecTheta = anglePu;
        pMotor->speed.Speed = pMotor->esmoSpeedPu;
        pMotor->speed.SpeedRpm =
                (int32_t)(pMotor->speed.BaseRpm * pMotor->speed.Speed);

        return(pMotor->esmoSpeedPu);
    }

    pMotor->speed.ElecTheta = pMotor->esmoRawAnglePu;
    runSpeedFR(&pMotor->speed);

    speedPu = pMotor->speed.Speed;
    esmoSpeedLoopFbkPu[motorNum] = speedPu;

    return(speedPu);
}

static void clearESMOHandoffLatch(void)
{
#if(ESMO_HANDOFF_LATCH_ENABLE != 0U)
    uint16_t i;

    for(i = 0U; i < ESMO_HANDOFF_LATCH_COUNT; i++)
    {
        esmoHandoffLatch[i].valid = 0U;
    }

    esmoHandoffLatchValidMask = 0U;
#endif

    return;
}

static void captureESMOHandoffLatch(MOTOR_Vars_t *pMotor,
                                    uint16_t motorNum,
                                    uint16_t stage)
{
#if(ESMO_HANDOFF_LATCH_ENABLE == 0U)
    return;
#else
    ESMO_HandoffLatch_t *pLatch;

    if(stage >= ESMO_HANDOFF_LATCH_COUNT)
    {
        return;
    }

    if(motorNum != esmoCompareLogMotor)
    {
        return;
    }

    pLatch = (ESMO_HandoffLatch_t *)&esmoHandoffLatch[stage];

    if(pLatch->valid != 0U)
    {
        return;
    }

    pLatch->valid = 1U;
    pLatch->stage = stage;
    pLatch->motorNum = motorNum;
    pLatch->lsw = pMotor->ptrFCL->lsw;
    pLatch->takeoverCnt = pMotor->esmoTakeoverCntr;
    pLatch->rgAngle_q15 = q15Log(pMotor->ptrFCL->rg.Out);
    pLatch->rawAngle_q15 = q15Log(pMotor->esmoRawAnglePu);
    pLatch->finalAngle_q15 = q15Log(pMotor->esmoAnglePu);
    pLatch->qepAngle_q15 = q15Log(pMotor->esmoQepAnglePu);
    pLatch->angleErr_q15 = q15Log(wrapPuHalfLocal(pMotor->esmoAnglePu -
                                                  pMotor->esmoQepAnglePu));
    pLatch->esmoSpeed_q15 = q15Log(pMotor->esmoSpeedPu);
    pLatch->iqRef_q15 = q15Log(pMotor->ptrFCL->pi_iq.ref);

    esmoHandoffLatchValidMask |= (uint16_t)(1U << stage);

    return;
#endif
}

#if(ESMO_STARTUP_LOG_ENABLE != 0U)
static inline void updatePeakF32(volatile float32_t *pPeak, float32_t value)
{
    if(value > *pPeak)
    {
        *pPeak = value;
    }

    return;
}
#endif

static inline float32_t limitSensorlessTakeoverIqRef(MOTOR_Vars_t *pMotor,
                                                     float32_t iqRef)
{
    uint16_t motorNum = (pMotor == &motorVars[0]) ? MTR_1 : MTR_2;
    float32_t limit;

    if(isSensorlessTakeoverActive(pMotor) == 0U)
    {
        esmoRunIqLimitPu[motorNum] = 0.0f;
        return(iqRef);
    }

    limit = (pMotor == &motorVars[0]) ? M1_ESMO_TAKEOVER_IQ_LIMIT :
                                        M2_ESMO_TAKEOVER_IQ_LIMIT;

    if(limit <= 0.0f)
    {
        esmoRunIqLimitPu[motorNum] = 0.0f;
        return(iqRef);
    }

    esmoRunIqLimitPu[motorNum] = limit;

    return(__fsat(iqRef, limit, -limit));
}

static inline void limitSensorlessTakeoverSpeedPid(MOTOR_Vars_t *pMotor)
{
    float32_t limitedOut;

    if(isSensorlessTakeoverActive(pMotor) == 0U)
    {
        return;
    }

    limitedOut = limitSensorlessTakeoverIqRef(pMotor,
                                               pMotor->pid_spd.term.Out);

    if(limitedOut != pMotor->pid_spd.term.Out)
    {
        pMotor->pid_spd.term.Out = limitedOut;

        if(pMotor->pid_spd.param.Kp > 0.0f)
        {
            pMotor->pid_spd.data.i1 = limitedOut / pMotor->pid_spd.param.Kp;
            pMotor->pid_spd.data.ui = pMotor->pid_spd.data.i1;
        }
    }

    return;
}

static inline float32_t getSensorlessStartupIqRef(MOTOR_Vars_t *pMotor)
{
    float32_t iqRef = pMotor->IqRef;
    float32_t setpointAbs;
    float32_t speedAbs;
    float32_t overspeed;
    float32_t band;
    float32_t minScale;
    float32_t scale;
    float32_t rampScale;

    if((isSensorlessControl(pMotor) == 0U) ||
       ((pMotor->ptrFCL->lsw != ENC_WAIT_FOR_INDEX) &&
        (isSensorlessTakeoverActive(pMotor) == 0U)))
    {
        return(iqRef);
    }

    if(pMotor == &motorVars[0])
    {
        band = M1_STARTUP_OVERSPEED_BAND;
        minScale = M1_STARTUP_IQ_MIN_SCALE;
    }
    else
    {
        band = M2_STARTUP_OVERSPEED_BAND;
        minScale = M2_STARTUP_IQ_MIN_SCALE;
    }

    setpointAbs = absF32(pMotor->rc.SetpointValue);
    speedAbs = absF32(pMotor->esmoSpeedPu);

    if((pMotor->ptrFCL->lsw == ENC_WAIT_FOR_INDEX) &&
       (pMotor->esmoForceRunCntMax > 0U))
    {
        rampScale = (float32_t)pMotor->esmoForceRunCntr /
                    (float32_t)pMotor->esmoForceRunCntMax;
        rampScale = minScale + ((1.0f - minScale) * rampScale);
        rampScale = __fsat(rampScale, 1.0f, minScale);
        iqRef *= rampScale;
    }

    if((band <= 0.0f) || (speedAbs <= setpointAbs))
    {
        return(iqRef);
    }

    overspeed = speedAbs - setpointAbs;
    scale = 1.0f - (overspeed / band);
    scale = __fsat(scale, 1.0f, minScale);

    return(iqRef * scale);
}

static inline void seedSensorlessSpeedController(MOTOR_Vars_t *pMotor)
{
    float32_t iqRef = getSensorlessStartupIqRef(pMotor);

    pMotor->pid_spd.data.d1 = 0.0f;
    pMotor->pid_spd.data.d2 = 0.0f;
    pMotor->pid_spd.data.i1 = iqRef / pMotor->pid_spd.param.Kp;
    pMotor->pid_spd.data.ud = 0.0f;
    pMotor->pid_spd.data.ui = pMotor->pid_spd.data.i1;
    pMotor->pid_spd.data.up = 0.0f;
    pMotor->pid_spd.term.Out = iqRef;

    return;
}

static void resetESMOQepReference(MOTOR_Vars_t *pMotor,
                                  uint16_t motorNum)
{
    SPEED_MEAS_QEP *pQepSpeed;
    float32_t qepAngle = 0.0f;

    if(motorNum > MTR_2)
    {
        return;
    }

    esmoQepIndexCalibrated[motorNum] = 0U;
    esmoQepIndexCount[motorNum] = 0U;
    esmoQepFrameOffsetPu[motorNum] = 0.0f;
    esmoQepFrameCaptured[motorNum] = 0U;
    esmoQepIndexSeen[motorNum] = 0U;
    esmoQepRefValid[motorNum] = 0U;
    esmoQepSpeedPu[motorNum] = 0.0f;
    esmoQepAngleErrPu[motorNum] = 0.0f;
    esmoQepSpeedErrPu[motorNum] = 0.0f;

    pMotor->esmoQepAnglePu = 0.0f;
    pMotor->esmoAngleErrPu = 0.0f;
    pMotor->esmoSpeedErrPu = 0.0f;

    if(pMotor->ptrFCL->ptrQEP != (volatile struct EQEP_REGS *)0)
    {
        pMotor->ptrFCL->qep.IndexSyncFlag = 0U;
        pMotor->ptrFCL->qep.QepCountIndex = 0U;

        if(pMotor->positionFeedback != POSITION_FEEDBACK_QEP)
        {
            if(esmoQepColdResetDone[motorNum] == 0U)
            {
                HAL_setupQEP(halMtrHandle[motorNum]);
                esmoQepColdResetDone[motorNum] = 1U;
            }

            // Restart the diagnostic QEP frame for the next eSMO run. The
            // frame will be captured again at the end of alignment.
            pMotor->ptrFCL->ptrQEP->QEPCTL.all &=
                    (uint16_t)(~ESMO_QEP_IEI_MASK);
            pMotor->ptrFCL->ptrQEP->QPOSINIT = 0U;
            pMotor->ptrFCL->ptrQEP->QPOSCNT = 0U;
        }

        qepAngle = pMotor->ptrFCL->qep.ElecTheta;
    }

    pQepSpeed = &esmoQepSpeedForCompare[motorNum];
    pQepSpeed->ElecTheta = qepAngle;
    pQepSpeed->OldElecTheta = qepAngle;
    pQepSpeed->Speed = 0.0f;
    pQepSpeed->SpeedRpm = 0;
    pQepSpeed->Tmp = 0.0f;
    pQepSpeed->DirectionQep = 1U;
    pQepSpeed->K1 = pMotor->speed.K1;
    pQepSpeed->K2 = pMotor->speed.K2;
    pQepSpeed->K3 = pMotor->speed.K3;
    pQepSpeed->BaseRpm = pMotor->speed.BaseRpm;

    return;
}

static void updateESMOQepIndexReference(MOTOR_Vars_t *pMotor,
                                        uint16_t motorNum)
{
    if((motorNum > MTR_2) ||
       (pMotor->ptrFCL->ptrQEP == (volatile struct EQEP_REGS *)0))
    {
        return;
    }

    if(pMotor->runMotor != MOTOR_RUN)
    {
        resetESMOQepReference(pMotor, motorNum);
        return;
    }

    if(esmoQepIndexCalibrated[motorNum] != 0U)
    {
        return;
    }

    if(pMotor->ptrFCL->qep.IndexSyncFlag == ESMO_QEP_INDEX_SYNC_FLAG)
    {
        uint32_t indexCount = pMotor->ptrFCL->qep.QepCountIndex;

        esmoQepIndexCount[motorNum] = indexCount;

        // Match the original FCL/QEP calibration convention: alignment sets
        // the FOC zero, and the index pulse only preserves that zero on later
        // index reloads. This keeps QEP diagnostics in the same frame as the
        // FOC/eSMO final angle instead of redefining zero at the physical index.
        pMotor->ptrFCL->ptrQEP->QPOSINIT = indexCount;
        pMotor->ptrFCL->ptrQEP->QEPCTL.all |= ESMO_QEP_IEI_RISING;

        esmoQepIndexCalibrated[motorNum] = 1U;
        esmoQepRefValid[motorNum] = 0U;
        pMotor->esmoQepAnglePu = pMotor->ptrFCL->qep.ElecTheta;
        pMotor->esmoAngleErrPu = wrapPuHalfLocal(pMotor->esmoAnglePu -
                                                 pMotor->esmoQepAnglePu);
    }

    return;
}

static void updateESMOQepReference(MOTOR_Vars_t *pMotor,
                                   uint16_t motorNum)
{
    SPEED_MEAS_QEP *pQepSpeed;
    float32_t qepAngle;
    float32_t qepSpeed;

    if((motorNum > MTR_2) ||
       (pMotor->positionFeedback != POSITION_FEEDBACK_ESMO) ||
       (pMotor->ptrFCL->ptrQEP == (volatile struct EQEP_REGS *)0))
    {
        if(motorNum <= MTR_2)
        {
            esmoQepRefValid[motorNum] = 0U;
        }

        return;
    }

    if(pMotor->runMotor == MOTOR_RUN)
    {
        esmoQepColdResetDone[motorNum] = 0U;
    }

    updateESMOQepIndexReference(pMotor, motorNum);

    pQepSpeed = &esmoQepSpeedForCompare[motorNum];

    // Keep physical index reload disabled in eSMO diagnostic mode. Otherwise
    // a later index edge can silently move QPOSCNT away from the alignment-zero
    // frame and reintroduce run-to-run angle offsets.
    pMotor->ptrFCL->ptrQEP->QEPCTL.all &= (uint16_t)(~ESMO_QEP_IEI_MASK);

    // Re-reference the per-run frame to the physical index pulse. QPOSILAT
    // latches QPOSCNT on every index rising edge; the IEL flag was cleared
    // at alignment end, so the first latch consumed here is from this run.
    // First run after power-up calibrates the index electrical angle; later
    // runs shift their frame so the index lands on the calibrated angle,
    // cancelling the per-run alignment settling residual.
    if((pMotor->runMotor == MOTOR_RUN) &&
       (esmoQepFrameCaptured[motorNum] != 0U) &&
       (esmoQepIndexSeen[motorNum] == 0U) &&
       ((pMotor->ptrFCL->ptrQEP->QFLG.all & ESMO_QEP_IEL_FLAG) != 0U))
    {
        float32_t idxAngle;

        esmoQepIndexCount[motorNum] = pMotor->ptrFCL->ptrQEP->QPOSILAT;

        idxAngle = pMotor->ptrFCL->qep.MechScaler *
                (float32_t)pMotor->ptrFCL->ptrQEP->QPOSILAT;
        idxAngle *= (float32_t)pMotor->ptrFCL->qep.PolePairs;
        idxAngle -= (float32_t)((int32_t)idxAngle);
        idxAngle = normalizePuLocal(idxAngle +
                                    esmoQepFrameOffsetPu[motorNum]);

        if((esmoQepIndexCalValid[motorNum] == 0U) &&
           (esmoQepIndexUseFixed[motorNum] == 0U))
        {
            // Auto mode, first run after power-up: capture the reference.
            esmoQepIndexAngleCal[motorNum] = idxAngle;
            esmoQepIndexCalValid[motorNum] = 1U;
        }
        else
        {
            // Fixed mode (every run) or auto mode (later runs): snap this
            // run's frame so the index lands on the calibrated angle. In
            // fixed mode the reference is the hardcoded absolute angle, so
            // the frame is identical across power-ups.
            if(esmoQepIndexUseFixed[motorNum] != 0U)
            {
                esmoQepIndexAngleCal[motorNum] =
                        esmoQepIndexAngleFixed[motorNum];
                esmoQepIndexCalValid[motorNum] = 1U;
            }

            esmoQepFrameOffsetPu[motorNum] = normalizePuLocal(
                    esmoQepFrameOffsetPu[motorNum] +
                    wrapPuHalfLocal(esmoQepIndexAngleCal[motorNum] -
                                    idxAngle));
        }

        esmoQepIndexSeen[motorNum] = 1U;
    }

    // Build the diagnostic QEP angle directly from the eQEP position count.
    // In eSMO mode this keeps the comparison frame tied to the alignment-time
    // QPOSCNT reset instead of the FCL/CLA QEP angle state or physical index.
    // esmoQepFrameOffsetPu translates the alignment-relative count into the
    // absolute stator frame so it can be compared against the eSMO angle.
    qepAngle = pMotor->ptrFCL->qep.MechScaler *
            (float32_t)pMotor->ptrFCL->ptrQEP->QPOSLAT;
    qepAngle *= (float32_t)pMotor->ptrFCL->qep.PolePairs;
    qepAngle -= (float32_t)((int32_t)qepAngle);
    qepAngle = normalizePuLocal(qepAngle + esmoQepFrameOffsetPu[motorNum]);

    pMotor->esmoQepAnglePu = qepAngle;
    pMotor->esmoAngleErrPu = wrapPuHalfLocal(pMotor->esmoAnglePu - qepAngle);

    if((pMotor->runMotor != MOTOR_RUN) ||
       (pMotor->baseFreq <= 0.0f) ||
       (pMotor->Ts <= 0.0f))
    {
        esmoQepRefValid[motorNum] = 0U;
        esmoQepSpeedPu[motorNum] = 0.0f;
        esmoQepSpeedErrPu[motorNum] = 0.0f;
        pMotor->esmoSpeedErrPu = 0.0f;
        return;
    }

    if((esmoCompareRequireQepIndex != 0U) &&
       (esmoQepIndexCalibrated[motorNum] == 0U))
    {
        esmoQepRefValid[motorNum] = 0U;
        esmoQepSpeedPu[motorNum] = 0.0f;
        esmoQepSpeedErrPu[motorNum] = 0.0f;
        pMotor->esmoAngleErrPu = 0.0f;
        pMotor->esmoSpeedErrPu = 0.0f;
        return;
    }

    if(esmoQepRefValid[motorNum] == 0U)
    {
        pQepSpeed->ElecTheta = qepAngle;
        pQepSpeed->OldElecTheta = qepAngle;
        pQepSpeed->Speed = 0.0f;
        pQepSpeed->SpeedRpm = 0;
        pQepSpeed->Tmp = 0.0f;
        pQepSpeed->DirectionQep = 1U;
        pQepSpeed->K1 = pMotor->speed.K1;
        pQepSpeed->K2 = pMotor->speed.K2;
        pQepSpeed->K3 = pMotor->speed.K3;
        pQepSpeed->BaseRpm = pMotor->speed.BaseRpm;

        esmoQepRefValid[motorNum] = 1U;
        esmoQepSpeedPu[motorNum] = 0.0f;
        esmoQepSpeedErrPu[motorNum] = 0.0f;
        pMotor->esmoSpeedErrPu = 0.0f;
        return;
    }

    pQepSpeed->ElecTheta = qepAngle;
    pQepSpeed->K1 = pMotor->speed.K1;
    pQepSpeed->K2 = pMotor->speed.K2;
    pQepSpeed->K3 = pMotor->speed.K3;
    pQepSpeed->BaseRpm = pMotor->speed.BaseRpm;
    runSpeedFR(pQepSpeed);
    qepSpeed = pQepSpeed->Speed;

    esmoQepSpeedPu[motorNum] = qepSpeed;
    esmoQepAngleErrPu[motorNum] = pMotor->esmoAngleErrPu;
    esmoQepSpeedErrPu[motorNum] = pMotor->esmoSpeedPu - qepSpeed;
    pMotor->esmoSpeedErrPu = esmoQepSpeedErrPu[motorNum];

    return;
}

#if(ESMO_STARTUP_LOG_ENABLE != 0U)
static inline void captureESMOStartupLog(MOTOR_Vars_t *pMotor,
                                         uint16_t motorNum)
{
#if(ESMO_STARTUP_LOG_ENABLE == 0U)
    return;
#else
    ESMO_Obj *pEsmo = (ESMO_Obj *)pMotor->esmoHandle;
    float32_t speedAbs;
    float32_t iqRefAbs;
    float32_t iqFbkAbs;
    float32_t eqMag;
    uint16_t shouldLog = 0U;

    if(motorNum != esmoStartupLogMotor)
    {
        return;
    }

    if((esmoStartupLogArmed != 0U) &&
       (esmoStartupLogActive == 0U) &&
       (esmoStartupLogPrevRun == MOTOR_STOP) &&
       (pMotor->runMotor == MOTOR_RUN))
    {
        esmoStartupLogIndex = 0;
        esmoStartupLogDecimCnt = 0;
        esmoStartupLogActive = 1;
        esmoStartupLogDone = 0;
        esmoStartupLogArmed = 0;
        esmoStartupPeakSpeedPu = 0.0f;
        esmoStartupPeakIqRef = 0.0f;
        esmoStartupPeakIqFbk = 0.0f;
        esmoStartupPeakEqMag = 0.0f;
        esmoStartupPeakSpeedLsw1Pu = 0.0f;
        esmoStartupPeakIqRefLsw1 = 0.0f;
        esmoStartupPeakIqFbkLsw1 = 0.0f;
        esmoStartupPeakEqMagLsw1 = 0.0f;
        esmoStartupPeakSpeedTakeoverPu = 0.0f;
        esmoStartupPeakIqRefTakeover = 0.0f;
        esmoStartupPeakIqFbkTakeover = 0.0f;
        esmoStartupPeakEqMagTakeover = 0.0f;
        esmoStartupPeakSpeedClosedPu = 0.0f;
        esmoStartupPeakIqRefClosed = 0.0f;
        esmoStartupPeakIqFbkClosed = 0.0f;
        esmoStartupPeakEqMagClosed = 0.0f;
    }

    esmoStartupLogPrevRun = pMotor->runMotor;

    if(esmoStartupLogActive == 0U)
    {
        return;
    }

    speedAbs = absF32(pMotor->esmoSpeedPu);
    iqRefAbs = absF32(pMotor->ptrFCL->pi_iq.ref);
    iqFbkAbs = absF32(pMotor->ptrFCL->pi_iq.fbk);
    eqMag = (pEsmo != (ESMO_Obj *)0) ? pEsmo->Eq_mag : 0.0f;

    updatePeakF32(&esmoStartupPeakSpeedPu, speedAbs);
    updatePeakF32(&esmoStartupPeakIqRef, iqRefAbs);
    updatePeakF32(&esmoStartupPeakIqFbk, iqFbkAbs);
    updatePeakF32(&esmoStartupPeakEqMag, eqMag);

    if(pMotor->ptrFCL->lsw == ENC_WAIT_FOR_INDEX)
    {
        updatePeakF32(&esmoStartupPeakSpeedLsw1Pu, speedAbs);
        updatePeakF32(&esmoStartupPeakIqRefLsw1, iqRefAbs);
        updatePeakF32(&esmoStartupPeakIqFbkLsw1, iqFbkAbs);
        updatePeakF32(&esmoStartupPeakEqMagLsw1, eqMag);
    }
    else if(isSensorlessTakeoverActive(pMotor) != 0U)
    {
        updatePeakF32(&esmoStartupPeakSpeedTakeoverPu, speedAbs);
        updatePeakF32(&esmoStartupPeakIqRefTakeover, iqRefAbs);
        updatePeakF32(&esmoStartupPeakIqFbkTakeover, iqFbkAbs);
        updatePeakF32(&esmoStartupPeakEqMagTakeover, eqMag);
    }
    else if(pMotor->ptrFCL->lsw == ENC_CALIBRATION_DONE)
    {
        updatePeakF32(&esmoStartupPeakSpeedClosedPu, speedAbs);
        updatePeakF32(&esmoStartupPeakIqRefClosed, iqRefAbs);
        updatePeakF32(&esmoStartupPeakIqFbkClosed, iqFbkAbs);
        updatePeakF32(&esmoStartupPeakEqMagClosed, eqMag);
    }

    if(esmoStartupLogDecimCnt == 0U)
    {
        shouldLog = 1U;
    }

    esmoStartupLogDecimCnt++;
    if(esmoStartupLogDecimCnt >= esmoStartupLogDecimation)
    {
        esmoStartupLogDecimCnt = 0;
    }

    if(shouldLog == 0U)
    {
        return;
    }

    esmoStartupLog[esmoStartupLogIndex].lsw = pMotor->ptrFCL->lsw;
    esmoStartupLog[esmoStartupLogIndex].flags =
            ((pMotor->runMotor == MOTOR_RUN) ? 1U : 0U) |
            ((pMotor->tripFlagDMC != 0U) ? 2U : 0U);
    esmoStartupLog[esmoStartupLogIndex].rcSetpoint_q15 =
            q15Log(pMotor->rc.SetpointValue);
    esmoStartupLog[esmoStartupLogIndex].rampAngle_q15 =
            q15Log(pMotor->ptrFCL->rg.Out);
    esmoStartupLog[esmoStartupLogIndex].speedFbk_q15 =
            q15Log(pMotor->speed.Speed);
    esmoStartupLog[esmoStartupLogIndex].esmoSpeed_q15 =
            q15Log(pMotor->esmoSpeedPu);
    esmoStartupLog[esmoStartupLogIndex].qepSpeed_q15 =
            q15Log((motorNum <= MTR_2) ? esmoQepSpeedPu[motorNum] : 0.0f);
    esmoStartupLog[esmoStartupLogIndex].esmoAngle_q15 =
            q15Log(pMotor->esmoAnglePu);
    esmoStartupLog[esmoStartupLogIndex].esmoAngleErr_q15 =
            q15Log(pMotor->esmoAngleErrPu);
    esmoStartupLog[esmoStartupLogIndex].qepAngle_q15 =
            q15Log(pMotor->esmoQepAnglePu);
    esmoStartupLog[esmoStartupLogIndex].eqMag_q15 =
            q15Log((pEsmo != (ESMO_Obj *)0) ? pEsmo->Eq_mag : 0.0f);
    esmoStartupLog[esmoStartupLogIndex].thetaErr_q15 =
            q15Log((pEsmo != (ESMO_Obj *)0) ? pEsmo->thetaErr : 0.0f);
    esmoStartupLog[esmoStartupLogIndex].pllOut_q15 =
            q15Log((pEsmo != (ESMO_Obj *)0) ? pEsmo->pll_Out : 0.0f);
    esmoStartupLog[esmoStartupLogIndex].piIqRef_q15 =
            q15Log(pMotor->ptrFCL->pi_iq.ref);
    esmoStartupLog[esmoStartupLogIndex].piIqFbk_q15 =
            q15Log(pMotor->ptrFCL->pi_iq.fbk);

    esmoStartupLogIndex++;

    if((esmoStartupLogIndex >= ESMO_STARTUP_LOG_SIZE) ||
       (pMotor->tripFlagDMC != 0U) ||
       (pMotor->runMotor == MOTOR_STOP))
    {
        esmoStartupLogActive = 0;
        esmoStartupLogDone = 1;
    }

    return;
#endif
}
#endif

static void captureESMOCompareLog(MOTOR_Vars_t *pMotor,
                                  uint16_t motorNum)
{
#if(ESMO_COMPARE_LOG_ENABLE == 0U)
    return;
#else
    ESMO_Obj *pEsmo = (ESMO_Obj *)pMotor->esmoHandle;
    float32_t vdOut;
    float32_t vqOut;
    float32_t piUseSq;
    float32_t vdLimit;
    float32_t iqLimit;
    float32_t vsLimitSq;

    if(motorNum != esmoCompareLogMotor)
    {
        return;
    }

    if((esmoCompareRequireQepIndex != 0U) &&
       (motorNum <= MTR_2) &&
       (esmoQepIndexCalibrated[motorNum] == 0U))
    {
        return;
    }

    if((esmoCompareLogArmed != 0U) &&
       (esmoCompareLogActive == 0U) &&
       (pMotor->runMotor == MOTOR_RUN) &&
       (absF32(pMotor->rc.SetpointValue) >=
               esmoCompareTriggerCommandPu) &&
       (absF32(pMotor->esmoSpeedPu) >= esmoCompareTriggerSpeedPu))
    {
        esmoCompareLogIndex = 0;
        esmoCompareLogDecimCnt = 0;
        esmoCompareLogActive = 1U;
        esmoCompareLogDone = 0U;
        esmoCompareLogArmed = 0U;
    }

    if(esmoCompareLogActive == 0U)
    {
        return;
    }

    if((pMotor->runMotor == MOTOR_STOP) || (pMotor->tripFlagDMC != 0U))
    {
        esmoCompareLogActive = 0U;
        esmoCompareLogDone = 1U;
        return;
    }

    if(esmoCompareLogDecimCnt != 0U)
    {
        esmoCompareLogDecimCnt++;

        if(esmoCompareLogDecimCnt >= esmoCompareLogDecimation)
        {
            esmoCompareLogDecimCnt = 0U;
        }

        return;
    }

    esmoCompareLogDecimCnt++;
    if(esmoCompareLogDecimCnt >= esmoCompareLogDecimation)
    {
        esmoCompareLogDecimCnt = 0U;
    }

    esmoCompareLog[esmoCompareLogIndex].lsw = pMotor->ptrFCL->lsw;
    esmoCompareLog[esmoCompareLogIndex].flags =
            ((pMotor->runMotor == MOTOR_RUN) ? 1U : 0U) |
            ((pMotor->tripFlagDMC != 0U) ? 2U : 0U) |
            ((isSensorlessTakeoverActive(pMotor) != 0U) ? 4U : 0U) |
            (((motorNum <= MTR_2) &&
              (esmoQepIndexCalibrated[motorNum] != 0U)) ? 8U : 0U);
    esmoCompareLog[esmoCompareLogIndex].rcSetpoint_q15 =
            q15Log(pMotor->rc.SetpointValue);

    vdOut = pMotor->pi_id.out;
    vqOut = pMotor->ptrFCL->pi_iq.out;
    piUseSq = 0.0f;
    vdLimit = absF32(pMotor->pi_id.Umax);
    iqLimit = absF32(pMotor->ptrFCL->pi_iq.Umax);

    if(vdLimit > 0.000001f)
    {
        piUseSq += ((vdOut * vdOut) / (vdLimit * vdLimit));
    }

    if(iqLimit > 0.000001f)
    {
        piUseSq += ((vqOut * vqOut) / (iqLimit * iqLimit));
    }

    vsLimitSq = 1.0f;

    esmoCompareLog[esmoCompareLogIndex].esmoAngle_q15 =
            q15Log(pMotor->esmoAnglePu);
    esmoCompareLog[esmoCompareLogIndex].qepAngle_q15 =
            q15Log(pMotor->esmoQepAnglePu);
    esmoCompareLog[esmoCompareLogIndex].angleErr_q15 =
            q15Log(pMotor->esmoAngleErrPu);
    esmoCompareLog[esmoCompareLogIndex].esmoSpeed_q15 =
            q15Log(pMotor->esmoSpeedPu);
    esmoCompareLog[esmoCompareLogIndex].qepSpeed_q15 =
            q15Log((motorNum <= MTR_2) ? esmoQepSpeedPu[motorNum] : 0.0f);
    esmoCompareLog[esmoCompareLogIndex].iqRef_q15 =
            q15Log(pMotor->ptrFCL->pi_iq.ref);
    esmoCompareLog[esmoCompareLogIndex].iqFbk_q15 =
            q15Log(pMotor->ptrFCL->pi_iq.fbk);
    esmoCompareLog[esmoCompareLogIndex].idFbk_q15 =
            q15Log(pMotor->pi_id.fbk);
    esmoCompareLog[esmoCompareLogIndex].vdOut_q15 =
            q15Log(vdOut);
    esmoCompareLog[esmoCompareLogIndex].vqOut_q15 =
            q15Log(vqOut);
    esmoCompareLog[esmoCompareLogIndex].vsSq_q15 =
            q15Log(piUseSq);
    esmoCompareLog[esmoCompareLogIndex].vsLimitSq_q15 =
            q15Log(vsLimitSq);
    esmoCompareLog[esmoCompareLogIndex].eqMag_q15 =
            q15Log((pEsmo != (ESMO_Obj *)0) ? pEsmo->Eq_mag : 0.0f);
    esmoCompareLog[esmoCompareLogIndex].thetaErr_q15 =
            q15Log((pEsmo != (ESMO_Obj *)0) ? pEsmo->thetaErr : 0.0f);

    esmoCompareLogIndex++;

    if(esmoCompareLogIndex >= ESMO_COMPARE_LOG_SIZE)
    {
        esmoCompareLogActive = 0U;
        esmoCompareLogDone = 1U;
    }

    return;
#endif
}

//
//   Various Incremental Build levels
//

//****************************************************************************
// INCRBUILD 1
//****************************************************************************
//
#if(BUILDLEVEL == FCL_LEVEL1)

// =============================== FCL_LEVEL 1 =================================
// Level 1 verifies
//  - PWM Generation blocks and DACs
// =============================================================================
// build level 1 subroutine for motor_1
#pragma FUNC_ALWAYS_INLINE(buildLevel1_M1)

static inline void buildLevel1_M1(void)
{
// -------------------------------------------------------------------------
// control force angle generation based on 'runMotor'
// -------------------------------------------------------------------------
    if(motorVars[0].runMotor == MOTOR_STOP)
    {
        motorVars[0].rc.TargetValue = 0;
        motorVars[0].rc.SetpointValue = 0;
        motorVars[0].ipark.Ds = 0.0;
        motorVars[0].ipark.Qs = 0.0;
    }
    else
    {
        motorVars[0].rc.TargetValue = motorVars[0].speedRef;
        motorVars[0].ipark.Ds = VdTesting;
        motorVars[0].ipark.Qs = VqTesting;
    }

// -----------------------------------------------------------------------------
// Connect inputs of the RMP module and call the ramp control module
// -----------------------------------------------------------------------------
    fclRampControl(&motorVars[0].rc);

// -----------------------------------------------------------------------------
// Connect inputs of the RAMP GEN module and call the ramp generator module
// -----------------------------------------------------------------------------
    motorVars[0].ptrFCL->rg.Freq = motorVars[0].rc.SetpointValue;
    fclRampGen((RAMPGEN *)&motorVars[0].ptrFCL->rg);

// -----------------------------------------------------------------------------
// Connect inputs of the INV_PARK module and call the inverse park module
// -----------------------------------------------------------------------------
    motorVars[0].ipark.Sine = __sinpuf32(motorVars[0].ptrFCL->rg.Out);
    motorVars[0].ipark.Cosine = __cospuf32(motorVars[0].ptrFCL->rg.Out);
    runIPark(&motorVars[0].ipark);

// -----------------------------------------------------------------------------
// Position encoder suite module
// -----------------------------------------------------------------------------
    FCL_runQEPWrap_M1(); // to wrap up the CLA functions in library

// ----------------------------------------------------------------------------
//  Measure DC Bus voltage
// ----------------------------------------------------------------------------
    motorVars[0].FCL_params.Vdcbus = getVdc(&motorVars[0]);

// -----------------------------------------------------------------------------
// Connect inputs of the SVGEN_DQ module and call the space-vector gen. module
// -----------------------------------------------------------------------------
    motorVars[0].svgen.Ualpha = motorVars[0].ipark.Alpha;
    motorVars[0].svgen.Ubeta  = motorVars[0].ipark.Beta;
    runSVGenDQ(&motorVars[0].svgen);

// -----------------------------------------------------------------------------
// Computed Duty and Write to CMPA register
// -----------------------------------------------------------------------------
    EPWM_setCounterCompareValue(halMtr[0].pwmHandle[0], EPWM_COUNTER_COMPARE_A,
                   (uint16_t)((M1_INV_PWM_HALF_TBPRD * motorVars[0].svgen.Tc) +
                               M1_INV_PWM_HALF_TBPRD));

    EPWM_setCounterCompareValue(halMtr[0].pwmHandle[1], EPWM_COUNTER_COMPARE_A,
                   (uint16_t)((M1_INV_PWM_HALF_TBPRD * motorVars[0].svgen.Ta) +
                               M1_INV_PWM_HALF_TBPRD));

    EPWM_setCounterCompareValue(halMtr[0].pwmHandle[2], EPWM_COUNTER_COMPARE_A,
                   (uint16_t)((M1_INV_PWM_HALF_TBPRD * motorVars[0].svgen.Tb) +
                               M1_INV_PWM_HALF_TBPRD));
    return;
}

// build level 1 subroutine for motor_2
#pragma FUNC_ALWAYS_INLINE(buildLevel1_M2)

static inline void buildLevel1_M2(void)
{
// -------------------------------------------------------------------------
// control force angle generation based on 'runMotor'
// -------------------------------------------------------------------------
    if(motorVars[1].runMotor == MOTOR_STOP)
    {
        motorVars[1].rc.TargetValue = 0;
        motorVars[1].rc.SetpointValue = 0;
        motorVars[1].ipark.Ds = 0.0;
        motorVars[1].ipark.Qs = 0.0;
    }
    else
    {
        motorVars[1].ipark.Ds = VdTesting;
        motorVars[1].ipark.Qs = VqTesting;
        motorVars[1].rc.TargetValue = motorVars[1].speedRef;
    }

// -----------------------------------------------------------------------------
// Connect inputs of the RMP module and call the ramp control module
// -----------------------------------------------------------------------------
    fclRampControl(&motorVars[1].rc);

// -----------------------------------------------------------------------------
// Connect inputs of the RAMP GEN module and call the ramp generator module
// -----------------------------------------------------------------------------
    motorVars[1].ptrFCL->rg.Freq = motorVars[1].rc.SetpointValue;
    fclRampGen((RAMPGEN *)&motorVars[1].ptrFCL->rg);

// -----------------------------------------------------------------------------
// Connect inputs of the INV_PARK module and call the inverse park module
// -----------------------------------------------------------------------------
    motorVars[1].ipark.Sine = __sinpuf32(motorVars[1].ptrFCL->rg.Out);
    motorVars[1].ipark.Cosine = __cospuf32(motorVars[1].ptrFCL->rg.Out);
    runIPark(&motorVars[1].ipark);

// -----------------------------------------------------------------------------
// Position encoder suite module
// -----------------------------------------------------------------------------
    FCL_runQEPWrap_M2(); // to wrap up the CLA functions in library

// ----------------------------------------------------------------------------
//  Measure DC Bus voltage
// ----------------------------------------------------------------------------
    motorVars[1].FCL_params.Vdcbus = getVdc(&motorVars[1]);

// -----------------------------------------------------------------------------
// Connect inputs of the SVGEN_DQ module and call the space-vector gen. module
// -----------------------------------------------------------------------------
    motorVars[1].svgen.Ualpha = motorVars[1].ipark.Alpha;
    motorVars[1].svgen.Ubeta  = motorVars[1].ipark.Beta;
    runSVGenDQ(&motorVars[1].svgen);

// -----------------------------------------------------------------------------
// Computed Duty and Write to CMPA register
// -----------------------------------------------------------------------------
    EPWM_setCounterCompareValue(halMtr[1].pwmHandle[0], EPWM_COUNTER_COMPARE_A,
                    (uint16_t)((M2_INV_PWM_HALF_TBPRD * motorVars[1].svgen.Tc) +
                                M2_INV_PWM_HALF_TBPRD));

    EPWM_setCounterCompareValue(halMtr[1].pwmHandle[1], EPWM_COUNTER_COMPARE_A,
                    (uint16_t)((M2_INV_PWM_HALF_TBPRD * motorVars[1].svgen.Ta) +
                                M2_INV_PWM_HALF_TBPRD));

    EPWM_setCounterCompareValue(halMtr[1].pwmHandle[2], EPWM_COUNTER_COMPARE_A,
                    (uint16_t)((M2_INV_PWM_HALF_TBPRD * motorVars[1].svgen.Tb) +
                                M2_INV_PWM_HALF_TBPRD));
    return;
}
#endif // (BUILDLEVEL==FCL_LEVEL1)

//
//****************************************************************************
// INCRBUILD 3
//****************************************************************************
//
#if(BUILDLEVEL == FCL_LEVEL2)
// =============================== FCL_LEVEL 2 =================================
// Level 2 verifies
//   - verify inline shunt current sense schemes
//     - analog-to-digital conversion
//   - Current Limit Settings for over current protection
//   - Position sensor interface is taken care by FCL lib using QEP
//     - speed estimation
// =============================================================================
// build level 2 subroutine for motor_1
#pragma FUNC_ALWAYS_INLINE(buildLevel2_M1)

static inline void buildLevel2_M1(void)
{
    // -------------------------------------------------------------------------
    // Alignment Routine: this routine aligns the motor to zero electrical
    // angle and in case of QEP also finds the index location and initializes
    // the angle w.r.t. the index location
    // -------------------------------------------------------------------------
    if(motorVars[0].runMotor == MOTOR_STOP)
    {
        motorVars[0].ptrFCL->lsw = ENC_ALIGNMENT;
        motorVars[0].IdRef = 0;
        motorVars[0].pi_id.ref = motorVars[0].IdRef;

        FCL_resetController(&motorVars[0]);

        motorVars[0].ipark.Ds = 0.0;
        motorVars[0].ipark.Qs = 0.0;
    }
    else if(motorVars[0].ptrFCL->lsw == ENC_ALIGNMENT)
    {
        // for restarting from (runMotor = STOP)
        motorVars[0].rc.TargetValue = 0;
        motorVars[0].rc.SetpointValue = 0;

        // for QEP, spin the motor to find the index pulse
        motorVars[0].ptrFCL->lsw = ENC_WAIT_FOR_INDEX;

        motorVars[0].ipark.Ds = VdTesting;
        motorVars[0].ipark.Qs = VqTesting;
    } // end else if(lsw == ENC_ALIGNMENT)

// ----------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control module
// ----------------------------------------------------------------------------
    if(motorVars[0].ptrFCL->lsw == ENC_ALIGNMENT)
    {
        motorVars[0].rc.TargetValue = 0;
    }
    else
    {
        motorVars[0].rc.TargetValue = motorVars[0].speedRef;
    }

    fclRampControl(&motorVars[0].rc);

// ----------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator module
// ----------------------------------------------------------------------------
    motorVars[0].ptrFCL->rg.Freq = motorVars[0].rc.SetpointValue;
    fclRampGen((RAMPGEN *)&motorVars[0].ptrFCL->rg);

// ----------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5)
//  to (-1,+1). Connect inputs of the CLARKE module and call the clarke
//  transformation module
// ----------------------------------------------------------------------------

    //wait on ADC EOC
    while(ADC_getInterruptStatus(M1_IW_ADC_BASE, ADC_INT_NUMBER1) == 0);

    NOP;    //1 cycle delay for ADC PPB result

    motorVars[0].clarke.As = (float32_t)M1_IFB_V_PPB *
            motorVars[0].FCL_params.adcScale;

    motorVars[0].clarke.Bs = (float32_t)M1_IFB_W_PPB *
            motorVars[0].FCL_params.adcScale;

    runClarke(&motorVars[0].clarke);

// ----------------------------------------------------------------------------
//  Measure DC Bus voltage
// ----------------------------------------------------------------------------
    motorVars[0].FCL_params.Vdcbus = getVdc(&motorVars[0]);

// ----------------------------------------------------------------------------
// Connect inputs of the PARK module and call the park module
// ----------------------------------------------------------------------------
    motorVars[0].park.Alpha  = motorVars[0].clarke.Alpha;
    motorVars[0].park.Beta   = motorVars[0].clarke.Beta;
    motorVars[0].park.Angle  = motorVars[0].ptrFCL->rg.Out;
    motorVars[0].park.Sine   = __sinpuf32(motorVars[0].park.Angle);
    motorVars[0].park.Cosine = __cospuf32(motorVars[0].park.Angle);
    runPark(&motorVars[0].park);

// ----------------------------------------------------------------------------
// Connect inputs of the INV_PARK module and call the inverse park module
// ----------------------------------------------------------------------------
    motorVars[0].ipark.Sine = motorVars[0].park.Sine;
    motorVars[0].ipark.Cosine = motorVars[0].park.Cosine;
    runIPark(&motorVars[0].ipark);

// ----------------------------------------------------------------------------
// Position encoder suite module
// ----------------------------------------------------------------------------
    FCL_runQEPWrap_M1();

    // Position Sensing is performed in CLA
    motorVars[0].posElecTheta = motorVars[0].ptrFCL->qep.ElecTheta;
    motorVars[0].posMechTheta = motorVars[0].ptrFCL->qep.MechTheta;

// ----------------------------------------------------------------------------
// Connect inputs of the SPEED_FR module and call the speed calculation module
// ----------------------------------------------------------------------------
    motorVars[0].speed.ElecTheta = motorVars[0].posElecTheta;
    runSpeedFR(&motorVars[0].speed);

// ----------------------------------------------------------------------------
// Connect inputs of the SVGEN_DQ module and call the space-vector gen. module
// ----------------------------------------------------------------------------
    motorVars[0].svgen.Ualpha = motorVars[0].ipark.Alpha;
    motorVars[0].svgen.Ubeta  = motorVars[0].ipark.Beta;
    runSVGenDQ(&motorVars[0].svgen);

// ----------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ----------------------------------------------------------------------------
    EPWM_setCounterCompareValue(halMtr[0].pwmHandle[0], EPWM_COUNTER_COMPARE_A,
                   (uint16_t)((M1_INV_PWM_HALF_TBPRD * motorVars[0].svgen.Tc) +
                               M1_INV_PWM_HALF_TBPRD));

    EPWM_setCounterCompareValue(halMtr[0].pwmHandle[1], EPWM_COUNTER_COMPARE_A,
                   (uint16_t)((M1_INV_PWM_HALF_TBPRD * motorVars[0].svgen.Ta) +
                               M1_INV_PWM_HALF_TBPRD));

    EPWM_setCounterCompareValue(halMtr[0].pwmHandle[2], EPWM_COUNTER_COMPARE_A,
                   (uint16_t)((M1_INV_PWM_HALF_TBPRD * motorVars[0].svgen.Tb) +
                               M1_INV_PWM_HALF_TBPRD));
    return;
}

// build level 2 subroutine for motor_2
#pragma FUNC_ALWAYS_INLINE(buildLevel2_M2)

static inline void buildLevel2_M2(void)
{
    // -------------------------------------------------------------------------
    // Alignment Routine: this routine aligns the motor to zero electrical
    // angle and in case of QEP also finds the index location and initializes
    // the angle w.r.t. the index location
    // -------------------------------------------------------------------------
    if(motorVars[1].runMotor == MOTOR_STOP)
    {
        motorVars[1].ptrFCL->lsw = ENC_ALIGNMENT;
        motorVars[1].IdRef = 0;
        motorVars[1].pi_id.ref = motorVars[1].IdRef;

        FCL_resetController(&motorVars[1]);

        motorVars[1].ipark.Ds = 0.0;
        motorVars[1].ipark.Qs = 0.0;
    }
    else if(motorVars[1].ptrFCL->lsw == ENC_ALIGNMENT)
    {
        // for restarting from (runMotor = STOP)
        motorVars[1].rc.TargetValue = 0;
        motorVars[1].rc.SetpointValue = 0;

        // for QEP, spin the motor to find the index pulse
        motorVars[1].ptrFCL->lsw = ENC_WAIT_FOR_INDEX;

        motorVars[1].ipark.Ds = VdTesting;
        motorVars[1].ipark.Qs = VqTesting;
    } // end else if(lsw == ENC_ALIGNMENT)

// ----------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control module
// ----------------------------------------------------------------------------
    if(motorVars[1].ptrFCL->lsw == ENC_ALIGNMENT)
    {
        motorVars[1].rc.TargetValue = 0;
    }
    else
    {
        motorVars[1].rc.TargetValue = motorVars[1].speedRef;
    }

    fclRampControl(&motorVars[1].rc);

// ----------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator module
// ----------------------------------------------------------------------------
    motorVars[1].ptrFCL->rg.Freq = motorVars[1].rc.SetpointValue;
    fclRampGen((RAMPGEN *)&motorVars[1].ptrFCL->rg);

// ----------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5)
//  to (-1,+1). Connect inputs of the CLARKE module and call the clarke
//  transformation module
// ----------------------------------------------------------------------------

    //wait on ADC EOC
    while(ADC_getInterruptStatus(M2_IW_ADC_BASE, ADC_INT_NUMBER2) == 0);

    NOP;    //1 cycle delay for ADC PPB result

    motorVars[1].clarke.As = (float32_t)M2_IFB_V_PPB *
            motorVars[1].FCL_params.adcScale;

    motorVars[1].clarke.Bs = (float32_t)M2_IFB_W_PPB *
            motorVars[1].FCL_params.adcScale;

    runClarke(&motorVars[1].clarke);

// ----------------------------------------------------------------------------
//  Measure DC Bus voltage using SDFM Filter3
// ----------------------------------------------------------------------------
    motorVars[1].FCL_params.Vdcbus = getVdc(&motorVars[1]);

// ----------------------------------------------------------------------------
// Connect inputs of the PARK module and call the park module
// ----------------------------------------------------------------------------
    motorVars[1].park.Alpha  = motorVars[1].clarke.Alpha;
    motorVars[1].park.Beta   = motorVars[1].clarke.Beta;
    motorVars[1].park.Angle  = motorVars[1].ptrFCL->rg.Out;
    motorVars[1].park.Sine   = __sinpuf32(motorVars[1].park.Angle);
    motorVars[1].park.Cosine = __cospuf32(motorVars[1].park.Angle);
    runPark(&motorVars[1].park);

// ----------------------------------------------------------------------------
// Connect inputs of the INV_PARK module and call the inverse park module
// ----------------------------------------------------------------------------
    motorVars[1].ipark.Sine = motorVars[1].park.Sine;
    motorVars[1].ipark.Cosine = motorVars[1].park.Cosine;
    runIPark(&motorVars[1].ipark);

// ----------------------------------------------------------------------------
// Position encoder suite module
// ----------------------------------------------------------------------------
    FCL_runQEPWrap_M2();

    // Position Sensing is performed in CLA
    motorVars[1].posElecTheta = motorVars[1].ptrFCL->qep.ElecTheta;
    motorVars[1].posMechTheta = motorVars[1].ptrFCL->qep.MechTheta;

// ----------------------------------------------------------------------------
// Connect inputs of the SPEED_FR module and call the speed calculation module
// ----------------------------------------------------------------------------
    motorVars[1].speed.ElecTheta = motorVars[1].posElecTheta;
    runSpeedFR(&motorVars[1].speed);

// ----------------------------------------------------------------------------
// Connect inputs of the SVGEN_DQ module and call the space-vector gen. module
// ----------------------------------------------------------------------------
    motorVars[1].svgen.Ualpha = motorVars[1].ipark.Alpha;
    motorVars[1].svgen.Ubeta  = motorVars[1].ipark.Beta;
    runSVGenDQ(&motorVars[1].svgen);

// ----------------------------------------------------------------------------
//  Computed Duty and Write to CMPA register
// ----------------------------------------------------------------------------
    EPWM_setCounterCompareValue(halMtr[1].pwmHandle[0], EPWM_COUNTER_COMPARE_A,
                   (uint16_t)((M2_INV_PWM_HALF_TBPRD * motorVars[1].svgen.Tc) +
                               M2_INV_PWM_HALF_TBPRD));

    EPWM_setCounterCompareValue(halMtr[1].pwmHandle[1], EPWM_COUNTER_COMPARE_A,
                   (uint16_t)((M2_INV_PWM_HALF_TBPRD * motorVars[1].svgen.Ta) +
                               M2_INV_PWM_HALF_TBPRD));

    EPWM_setCounterCompareValue(halMtr[1].pwmHandle[2], EPWM_COUNTER_COMPARE_A,
                   (uint16_t)((M2_INV_PWM_HALF_TBPRD * motorVars[1].svgen.Tb) +
                               M2_INV_PWM_HALF_TBPRD));
    return;
}

#endif // (BUILDLEVEL==FCL_LEVEL2)


//
//****************************************************************************
// INCRBUILD 3
//****************************************************************************
//
#if(BUILDLEVEL == FCL_LEVEL3)
// =============================== FCL_LEVEL 3 ================================
//  Level 3 verifies the dq-axis current regulation performed by PID and speed
//  measurement modules
//  lsw = ENC_ALIGNMENT      : lock the rotor of the motor
//  lsw = ENC_WAIT_FOR_INDEX : close the current loop
//  NOTE:-
//      1. Iq loop is closed using actual QEP angle.
//         Therefore, motor speed races to high speed with lighter load. It is
//         better to ensure the motor is loaded during this test. Otherwise,
//         the motor will run at higher speeds where it can saturate.
//         It may be typically around the rated speed of the motor or higher.
//      2. clarke1.As and clarke1.Bs are not brought out from the FCL library
//         as of library release version 0x02
// ============================================================================

// build level 3 subroutine for motor_1
#pragma FUNC_ALWAYS_INLINE(buildLevel3_M1)

static inline void buildLevel3_M1(void)
{

#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrl_M1(&motorVars[0]);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrl_M1(&motorVars[0]);
#endif

// ----------------------------------------------------------------------------
// FCL_cycleCount calculations for debug
// customer can remove the below code in final implementation
// ----------------------------------------------------------------------------
    getFCLTime(MTR_1);

// ----------------------------------------------------------------------------
// Measure DC Bus voltage using SDFM Filter3
// ----------------------------------------------------------------------------
    motorVars[0].FCL_params.Vdcbus = getVdc(&motorVars[0]);

// ----------------------------------------------------------------------------
// Fast current loop controller wrapper
// ----------------------------------------------------------------------------
#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrlWrap_M1(&motorVars[0]);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrlWrap_M1(&motorVars[0]);
#endif

// ----------------------------------------------------------------------------
// Alignment Routine: this routine aligns the motor to zero electrical angle
// and in case of QEP also finds the index location and initializes the angle
// w.r.t. the index location
// ----------------------------------------------------------------------------
    if(motorVars[0].runMotor == MOTOR_STOP)
    {
        motorVars[0].ptrFCL->lsw = ENC_ALIGNMENT;
        motorVars[0].pi_id.ref = 0;
        motorVars[0].IdRef = 0;
        FCL_resetController(&motorVars[0]);
    }
    else if(motorVars[0].ptrFCL->lsw == ENC_ALIGNMENT)
    {
        // alignment current
        motorVars[0].IdRef = motorVars[0].IdRef_start;  //0.1;

        // set up an alignment and hold time for shaft to settle down
        if(motorVars[0].pi_id.ref >= motorVars[0].IdRef)
        {
            motorVars[0].alignCntr++;

            if(motorVars[0].alignCntr >= motorVars[0].alignCnt)
            {
                motorVars[0].alignCntr  = 0;

                // for QEP, spin the motor to find the index pulse
                motorVars[0].ptrFCL->lsw = ENC_WAIT_FOR_INDEX;
            }
        }

    } // end else if(lsw == ENC_ALIGNMENT)
    else if(motorVars[0].ptrFCL->lsw == ENC_CALIBRATION_DONE)
    {
        motorVars[0].IdRef = motorVars[0].IdRef_run;
    }

// ----------------------------------------------------------------------------
// Connect inputs of the RMP module and call the ramp control module
// ----------------------------------------------------------------------------
    if(motorVars[0].ptrFCL->lsw == ENC_ALIGNMENT)
    {
        motorVars[0].rc.TargetValue = 0;
        motorVars[0].rc.SetpointValue = 0;
    }
    else
    {
        motorVars[0].rc.TargetValue = motorVars[0].speedRef;
    }

    fclRampControl(&motorVars[0].rc);

// ----------------------------------------------------------------------------
// Connect inputs of the RAMP GEN module and call the ramp generator module
// ----------------------------------------------------------------------------
    motorVars[0].ptrFCL->rg.Freq = motorVars[0].rc.SetpointValue;
    fclRampGen((RAMPGEN *)&motorVars[0].ptrFCL->rg);

    motorVars[0].posElecTheta = motorVars[0].ptrFCL->qep.ElecTheta;
    motorVars[0].speed.ElecTheta = motorVars[0].posElecTheta;

    runSpeedFR(&motorVars[0].speed);

// ----------------------------------------------------------------------------
// setup iqref for FCL
// ----------------------------------------------------------------------------
    motorVars[0].ptrFCL->pi_iq.ref =
           (motorVars[0].ptrFCL->lsw == ENC_ALIGNMENT) ? 0 : motorVars[0].IqRef;

// ----------------------------------------------------------------------------
// setup idref for FCL
// ----------------------------------------------------------------------------
    motorVars[0].pi_id.ref =
           ramper(motorVars[0].IdRef, motorVars[0].pi_id.ref, 0.00001);

    return;
}

// build level 3 subroutine for motor_1
#pragma FUNC_ALWAYS_INLINE(buildLevel3_M2)

static inline void buildLevel3_M2(void)
{

#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrl_M2(&motorVars[1]);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrl_M2(&motorVars[1]);
#endif

// ----------------------------------------------------------------------------
// FCL_cycleCount calculations for debug
// customer can remove the below code in final implementation
// ----------------------------------------------------------------------------
    getFCLTime(MTR_2);

// ----------------------------------------------------------------------------
// Measure DC Bus voltage using SDFM Filter3
// ----------------------------------------------------------------------------
    motorVars[1].FCL_params.Vdcbus = getVdc(&motorVars[0]);

// ----------------------------------------------------------------------------
// Fast current loop controller wrapper
// ----------------------------------------------------------------------------
#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrlWrap_M2(&motorVars[1]);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrlWrap_M2(&motorVars[1]);
#endif

// ----------------------------------------------------------------------------
// Alignment Routine: this routine aligns the motor to zero electrical angle
// and in case of QEP also finds the index location and initializes the angle
// w.r.t. the index location
// ----------------------------------------------------------------------------
    if(motorVars[1].runMotor == MOTOR_STOP)
    {
        motorVars[1].ptrFCL->lsw = ENC_ALIGNMENT;
        motorVars[1].pi_id.ref = 0;
        motorVars[1].IdRef = 0;
        FCL_resetController(&motorVars[1]);

        motorVars[1].state |= 0x8000;
    }
    else if(motorVars[1].ptrFCL->lsw == ENC_ALIGNMENT)
    {
        // alignment current
        motorVars[1].IdRef = motorVars[1].IdRef_start;  //0.1;

        motorVars[1].state |= 0x0001;

        // set up an alignment and hold time for shaft to settle down
        if(motorVars[1].pi_id.ref >= motorVars[1].IdRef)
        {
            motorVars[1].state |= 0x0002;

            motorVars[1].alignCntr++;

            if(motorVars[1].alignCntr >= motorVars[1].alignCnt)
            {
                motorVars[1].alignCntr  = 0;

                // for QEP, spin the motor to find the index pulse
                motorVars[1].ptrFCL->lsw = ENC_WAIT_FOR_INDEX;

                motorVars[1].state |= 0x0004;
            }
        }
    } // end else if(lsw == ENC_ALIGNMENT)
    else if(motorVars[1].ptrFCL->lsw == ENC_CALIBRATION_DONE)
    {
        motorVars[1].IdRef = motorVars[1].IdRef_run;

        motorVars[1].state |= 0x0010;
    }

// ----------------------------------------------------------------------------
// Connect inputs of the RMP module and call the ramp control module
// ----------------------------------------------------------------------------
    if(motorVars[1].ptrFCL->lsw == ENC_ALIGNMENT)
    {
        motorVars[1].rc.TargetValue = 0;
        motorVars[1].rc.SetpointValue = 0;
    }
    else
    {
        motorVars[1].rc.TargetValue = motorVars[1].speedRef;

        motorVars[1].state |= 0x0020;
    }

    fclRampControl(&motorVars[1].rc);

// ----------------------------------------------------------------------------
// Connect inputs of the RAMP GEN module and call the ramp generator module
// ----------------------------------------------------------------------------
    motorVars[1].ptrFCL->rg.Freq = motorVars[1].rc.SetpointValue;
    fclRampGen((RAMPGEN *)&motorVars[1].ptrFCL->rg);

    motorVars[1].posElecTheta = motorVars[1].ptrFCL->qep.ElecTheta;
    motorVars[1].speed.ElecTheta = motorVars[1].posElecTheta;

    runSpeedFR(&motorVars[1].speed);

// ----------------------------------------------------------------------------
// setup iqref for FCL
// ----------------------------------------------------------------------------
    motorVars[1].ptrFCL->pi_iq.ref =
           (motorVars[1].ptrFCL->lsw == ENC_ALIGNMENT) ? 0 : motorVars[1].IqRef;

// ----------------------------------------------------------------------------
// setup idref for FCL
// ----------------------------------------------------------------------------
    motorVars[1].pi_id.ref =
           ramper(motorVars[1].IdRef, motorVars[1].pi_id.ref, 0.00001);

    return;
}

#endif // (BUILDLEVEL==FCL_LEVEL3)

//
//****************************************************************************
// INCRBUILD 4
//****************************************************************************
//
#if((BUILDLEVEL == FCL_LEVEL4) || (BUILDLEVEL == FCL_LEVEL6) )
// =============================== FCL_LEVEL 4 ================================
// Level 4 verifies the speed regulator performed by PID module.
// The system speed loop is closed by using the measured speed as feedback
//  lsw = ENC_ALIGNMENT      : lock the rotor of the motor
//  lsw = ENC_WAIT_FOR_INDEX : - needed only with QEP encoders until first
//                               index pulse
//                             - Loops shown for 'lsw=ENC_CALIBRATION_DONE' are
//                               closed in this stage
//  lsw = ENC_CALIBRATION_DONE      : close speed loop and current loops Id, Iq
//
//  ****************************************************************
//
//  Level 6 verifies the SFRA functions used to verify bandwidth.
//  This demo code uses Level 4 code to perform SFRA analysis on
//  a current loop inside the speed loop
//
// ============================================================================
// build level 4/6 subroutine for motor_1
#pragma FUNC_ALWAYS_INLINE(buildLevel46_M1)

static inline void buildLevel46_M1(void)
{

#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrl_M1(&motorVars[0]);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrl_M1(&motorVars[0]);
#endif

// ----------------------------------------------------------------------------
// FCL_cycleCount calculations for debug
// customer can remove the below code in final implementation
// ----------------------------------------------------------------------------
    getFCLTime(MTR_1);

// -----------------------------------------------------------------------------
// Measure DC Bus voltage using SDFM Filter3
// ----------------------------------------------------------------------------
    motorVars[0].FCL_params.Vdcbus = getVdc(&motorVars[0]);

// ----------------------------------------------------------------------------
// Fast current loop controller wrapper
// ----------------------------------------------------------------------------
#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrlWrap_M1(&motorVars[0]);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrlWrap_M1(&motorVars[0]);
#endif

    // ------------------------------------------------------------------------
    // Alignment Routine: this routine aligns the motor to zero electrical
    // angle and in case of QEP also finds the index location and initializes
    // the angle w.r.t. the index location
    // ------------------------------------------------------------------------
    if(motorVars[0].runMotor == MOTOR_STOP)
    {
        motorVars[0].ptrFCL->lsw = ENC_ALIGNMENT;
        motorVars[0].IdRef = 0;
        motorVars[0].tempIdRef = motorVars[0].IdRef;
        motorVars[0].esmoForceRunCntr = 0;
        // Restart the ramp generator from the stator zero so every
        // STOP/RUN cycle aligns the rotor at the same electrical angle as
        // a fresh power-up, instead of the angle frozen at the last stop.
        motorVars[0].ptrFCL->rg.Angle = 0.0f;
        motorVars[0].ptrFCL->rg.Out = 0.0f;
        resetSensorlessEstimator(&motorVars[0]);
        resetESMOQepReference(&motorVars[0], MTR_1);
        FCL_resetController(&motorVars[0]);
    }
    else if(motorVars[0].ptrFCL->lsw == ENC_ALIGNMENT)
    {
        // alignment current
        motorVars[0].IdRef = motorVars[0].IdRef_start;  //(0.1);

        // set up an alignment and hold time for shaft to settle down
        if(motorVars[0].tempIdRef >= motorVars[0].IdRef)
        {
            motorVars[0].alignCntr++;

            if(motorVars[0].alignCntr >= motorVars[0].alignCnt)
            {
                motorVars[0].alignCntr  = 0;
                if((motorVars[0].positionFeedback == POSITION_FEEDBACK_ESMO) &&
                   (motorVars[0].ptrFCL->ptrQEP !=
                        (volatile struct EQEP_REGS *)0))
                {
                    // Use the alignment-held position as the QEP compare
                    // frame, captured ONCE per power-up: the eQEP tracks the
                    // rotor through STOP, so later runs reuse the same zero
                    // instead of re-sampling the alignment settling residual.
                    // rg.Out is the electrical angle the rotor is locked at,
                    // latched as the offset back into the stator frame.
                    esmoQepIndexCalibrated[MTR_1] = 1U;
                    esmoQepIndexCount[MTR_1] = 0U;
                    if(esmoQepFrameCaptured[MTR_1] == 0U)
                    {
                        esmoQepFrameOffsetPu[MTR_1] =
                                normalizePuLocal(motorVars[0].ptrFCL->rg.Out);
                        motorVars[0].ptrFCL->ptrQEP->QPOSCNT = 0U;
                        // Discard any stale index latch so the index-based
                        // frame correction consumes an index from THIS run.
                        motorVars[0].ptrFCL->ptrQEP->QCLR.all =
                                ESMO_QEP_IEL_FLAG;
                        esmoQepIndexSeen[MTR_1] = 0U;
                        esmoQepFrameCaptured[MTR_1] = 1U;
                    }
                }

                // for QEP, spin the motor to find the index pulse
                motorVars[0].ptrFCL->lsw = ENC_WAIT_FOR_INDEX;
            }
        }
    } // end else if(lsw == ENC_ALIGNMENT)
    else if(motorVars[0].ptrFCL->lsw == ENC_CALIBRATION_DONE)
    {
        motorVars[0].IdRef = motorVars[0].IdRef_run;
    }

// -----------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control module
// -----------------------------------------------------------------------------
    if(motorVars[0].ptrFCL->lsw == ENC_ALIGNMENT)
    {
        motorVars[0].rc.TargetValue = 0;
        motorVars[0].rc.SetpointValue = 0;
    }
    else if((motorVars[0].ptrFCL->lsw == ENC_WAIT_FOR_INDEX) ||
            (isSensorlessTakeoverActive(&motorVars[0]) != 0U))
    {
        motorVars[0].rc.TargetValue =
                (isSensorlessControl(&motorVars[0]) ?
                 motorVars[0].esmoForceSpeed : motorVars[0].lsw1Speed) *
                (motorVars[0].speedRef > 0 ? 1 : -1);
    }
    else
    {
        motorVars[0].rc.TargetValue = motorVars[0].speedRef;
    }

    fclRampControl(&motorVars[0].rc);

// -----------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator module
// -----------------------------------------------------------------------------
    motorVars[0].ptrFCL->rg.Freq = motorVars[0].rc.SetpointValue;
    fclRampGen((RAMPGEN *)&motorVars[0].ptrFCL->rg);

    if(isSensorlessControl(&motorVars[0]) != 0U)
    {
        if(motorVars[0].ptrFCL->lsw == ENC_WAIT_FOR_INDEX)
        {
            if(motorVars[0].esmoForceRunCntr >=
                    motorVars[0].esmoForceRunCntMax)
            {
                if((absF32(motorVars[0].rc.SetpointValue) >=
                        M1_ESMO_TAKEOVER_MIN_SETPOINT) &&
                   (absF32(motorVars[0].esmoSpeedPu) >=
                        M1_ESMO_TAKEOVER_MIN_SPEED) &&
                   (isSensorlessReadyForClosedLoop(&motorVars[0]) != 0U))
                {
                    motorVars[0].esmoForceRunCntr = 0;
                    motorVars[0].esmoTakeoverCntr = 0;
                    if(esmoCompareLogMotor == MTR_1)
                    {
                        clearESMOHandoffLatch();
                        captureESMOHandoffLatch(&motorVars[0], MTR_1,
                                                ESMO_HANDOFF_LATCH_PRE_SYNC);
                    }
#if(ESMO_DEBUG_QEP_HANDOFF_SYNC != 0U)
                    if((esmoQepIndexCalibrated[MTR_1] != 0U) &&
                       (esmoQepRefValid[MTR_1] != 0U))
                    {
                        syncSensorlessEstimatorAngle(&motorVars[0],
                                                     motorVars[0].esmoQepAnglePu);
                        motorVars[0].esmoTakeoverCntr =
                                motorVars[0].esmoTakeoverCntMax;
                    }
                    else
                    {
                        syncSensorlessEstimatorAngle(&motorVars[0],
                                                     motorVars[0].ptrFCL->rg.Out);
                    }
#else
                    syncSensorlessEstimatorAngle(&motorVars[0],
                                                 motorVars[0].ptrFCL->rg.Out);
#endif
                    captureESMOHandoffLatch(&motorVars[0], MTR_1,
                                            ESMO_HANDOFF_LATCH_POST_SYNC);
                    seedSensorlessSpeedController(&motorVars[0]);
                    motorVars[0].ptrFCL->lsw = ENC_CALIBRATION_DONE;
#if(ESMO_DEBUG_QEP_HANDOFF_SYNC != 0U)
                    if((esmoQepIndexCalibrated[MTR_1] != 0U) &&
                       (esmoQepRefValid[MTR_1] != 0U))
                    {
                        captureESMOHandoffLatch(&motorVars[0], MTR_1,
                                                ESMO_HANDOFF_LATCH_TAKEOVER_DONE);
                    }
#endif
                }
            }
            else
            {
                motorVars[0].esmoForceRunCntr++;
            }
        }
        else
        {
            motorVars[0].esmoForceRunCntr = 0;
        }

        if(motorVars[0].ptrFCL->lsw != ENC_CALIBRATION_DONE)
        {
            forceSensorlessAngle(&motorVars[0], motorVars[0].ptrFCL->rg.Out);
        }
    }

    runSensorlessEstimator(&motorVars[0]);

    if((isSensorlessControl(&motorVars[0]) != 0U) &&
       (motorVars[0].ptrFCL->lsw != ENC_CALIBRATION_DONE))
    {
        forceSensorlessAngle(&motorVars[0], motorVars[0].ptrFCL->rg.Out);
    }
    else if(isSensorlessTakeoverActive(&motorVars[0]) != 0U)
    {
        blendSensorlessAngle(&motorVars[0], motorVars[0].ptrFCL->rg.Out);
        if(motorVars[0].esmoTakeoverCntr >= motorVars[0].esmoTakeoverCntMax)
        {
            captureESMOHandoffLatch(&motorVars[0], MTR_1,
                                    ESMO_HANDOFF_LATCH_TAKEOVER_DONE);
        }
    }

// -----------------------------------------------------------------------------
//  Connect inputs of the SPEED_FR module and call the speed calculation module
// -----------------------------------------------------------------------------
    if(isSensorlessControl(&motorVars[0]) != 0U)
    {
        motorVars[0].posElecTheta = motorVars[0].esmoAnglePu;
        motorVars[0].posMechTheta = 0.0f;
        motorVars[0].speed.Speed =
                getSensorlessSpeedLoopFbk(&motorVars[0], MTR_1);
        motorVars[0].speed.SpeedRpm =
                motorVars[0].speed.BaseRpm * motorVars[0].speed.Speed;
    }
    else
    {
        motorVars[0].posElecTheta = motorVars[0].ptrFCL->qep.ElecTheta;
        motorVars[0].posMechTheta = motorVars[0].ptrFCL->qep.MechTheta;
        motorVars[0].speed.ElecTheta = motorVars[0].posElecTheta;
        runSpeedFR(&motorVars[0].speed);
    }

    updateESMOQepReference(&motorVars[0], MTR_1);

#if((BUILDLEVEL == FCL_LEVEL6) && (SFRA_MOTOR == MOTOR_1))
// -----------------------------------------------------------------------------
//    SFRA collect routine, only to be called after SFRA inject has occurred 1st
// -----------------------------------------------------------------------------
    if(sfraCollectStart)
    {
        collectSFRA(&motorVars[0]);    // Collect noise feedback from loop
    }

// -----------------------------------------------------------------------------
//  SFRA injection
// -----------------------------------------------------------------------------
    injectSFRA();               // create SFRA Noise per 'sfraTestLoop'

    sfraCollectStart = 1;       // enable SFRA data collection
#endif

// -----------------------------------------------------------------------------
//    Connect inputs of the PI module and call the PID speed controller module
// -----------------------------------------------------------------------------
    motorVars[0].speedLoopCount++;

    if(motorVars[0].speedLoopCount >= motorVars[0].speedLoopPrescaler)
    {   // �ٶȻ��ķ�Ƶ���ٶȻ��ĸ���Ƶ��ͨ���ȵ�������

#if((BUILDLEVEL == FCL_LEVEL6) && (SFRA_MOTOR == MOTOR_1))
        // SFRA Noise injection in speed loop
        motorVars[0].pid_spd.term.Ref =
                motorVars[0].rc.SetpointValue + sfraNoiseW;
#else       // if(BUILDLEVEL == FCL_LEVEL4)
        motorVars[0].pid_spd.term.Ref =
                motorVars[0].rc.SetpointValue;  //speedRef;// �趨�ٶ�
#endif

        motorVars[0].pid_spd.term.Fbk = motorVars[0].speed.Speed;// �����ٶ�
        runPID(&motorVars[0].pid_spd);// ִ���ٶ� PID

        limitSensorlessTakeoverSpeedPid(&motorVars[0]);
        motorVars[0].speedLoopCount = 0;

        // ---------------------------------------------------------------------
        // ������Эͬ PI ���������� (���룺���1�ٶ� - ���2�ٶ�)
        // ---------------------------------------------------------------------
        syncSpdErr = motorVars[0].speed.Speed - motorVars[1].speed.Speed;

        if((flagSyncPI == true) &&
           (motorVars[0].runMotor == MOTOR_RUN) &&
           (motorVars[1].runMotor == MOTOR_RUN) &&
           (motorVars[0].tripFlagDMC == 0U) &&
           (motorVars[1].tripFlagDMC == 0U) &&
           (motorVars[0].ptrFCL->lsw == ENC_CALIBRATION_DONE) &&
           (motorVars[1].ptrFCL->lsw == ENC_CALIBRATION_DONE))
        {
            pid_sync_spd.term.Ref = motorVars[0].speed.Speed;
            pid_sync_spd.term.Fbk = motorVars[1].speed.Speed;
            runPID(&pid_sync_spd);
            syncPI_out = pid_sync_spd.term.Out;
            syncPIActive = 1U;
        }
        else
        {
            pid_sync_spd.data.d1 = 0.0;
            pid_sync_spd.data.d2 = 0.0;
            pid_sync_spd.data.i1 = 0.0;
            pid_sync_spd.data.ud = 0.0;
            pid_sync_spd.data.ui = 0.0;
            pid_sync_spd.data.up = 0.0;
            pid_sync_spd.term.Out = 0.0;
            syncPI_out = 0.0;
            syncPIActive = 0U;
        }
    }

    if((motorVars[0].ptrFCL->lsw == ENC_ALIGNMENT) ||
            (motorVars[0].ptrFCL->lsw == ENC_WAIT_FOR_INDEX))
    {
        motorVars[0].pid_spd.data.d1 = 0;
        motorVars[0].pid_spd.data.d2 = 0;
        motorVars[0].pid_spd.data.i1 = 0;
        motorVars[0].pid_spd.data.ud = 0;
        motorVars[0].pid_spd.data.ui = motorVars[0].pid_spd.data.i1;
        motorVars[0].pid_spd.data.up = 0;
        motorVars[0].pid_spd.term.Out = 0;
    }

// -----------------------------------------------------------------------------
//    setup iqref and idref for FCL
// -----------------------------------------------------------------------------
#if((BUILDLEVEL == FCL_LEVEL6) && (SFRA_MOTOR == MOTOR_1))
    // SFRA Noise injection in Q axis
    motorVars[0].ptrFCL->pi_iq.ref =
            (motorVars[0].ptrFCL->lsw == ENC_ALIGNMENT) ? 0 :
                    (motorVars[0].ptrFCL->lsw == ENC_WAIT_FOR_INDEX) ?
                            getSensorlessStartupIqRef(&motorVars[0]) :
                            limitSensorlessTakeoverIqRef(&motorVars[0],
                                    (motorVars[0].pid_spd.term.Out +
                                     sfraNoiseQ));

    // SFRA Noise injection in D axis
    motorVars[0].tempIdRef =
            ramper(motorVars[0].IdRef, motorVars[0].tempIdRef, 0.00001);

    motorVars[0].pi_id.ref = motorVars[0].tempIdRef + sfraNoiseD;
#else   // if(BUILDLEVEL == FCL_LEVEL4)
    // setup iqref (����Эͬ���Ʋ���)
    motorVars[0].ptrFCL->pi_iq.ref =
            (motorVars[0].ptrFCL->lsw == ENC_ALIGNMENT) ? 0 :
                    (motorVars[0].ptrFCL->lsw == ENC_WAIT_FOR_INDEX) ?
                            getSensorlessStartupIqRef(&motorVars[0]) :
                            limitSensorlessTakeoverIqRef(&motorVars[0],
                                    (motorVars[0].pid_spd.term.Out -
                                     syncPI_out));

    // setup idref
    motorVars[0].tempIdRef = ramper(motorVars[0].IdRef,
                                    motorVars[0].tempIdRef, 0.00001);
    motorVars[0].pi_id.ref = motorVars[0].tempIdRef;
#endif

    // Field weakening: overrides pi_id.ref / pi_iq.ref with the negative-id
    // split only when enabled AND in steady closed loop (no-op otherwise).
    runFieldWeakening(&motorVars[0], MTR_1);
    updateESMOVoltageUseMonitor(&motorVars[0], MTR_1);

   return;
}

// build level 4/6 subroutine for motor_2
#pragma FUNC_ALWAYS_INLINE(buildLevel46_M2)

static inline void buildLevel46_M2(void)
{

#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrl_M2(&motorVars[1]);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrl_M2(&motorVars[1]);
#endif

// ----------------------------------------------------------------------------
// FCL_cycleCount calculations for debug
// customer can remove the below code in final implementation
// ----------------------------------------------------------------------------
    getFCLTime(MTR_2);

// -----------------------------------------------------------------------------
// Measure DC Bus voltage using SDFM Filter3
// ----------------------------------------------------------------------------
    motorVars[1].FCL_params.Vdcbus = getVdc(&motorVars[1]);

// ----------------------------------------------------------------------------
// Fast current loop controller wrapper
// ----------------------------------------------------------------------------
#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrlWrap_M2(&motorVars[1]);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrlWrap_M2(&motorVars[1]);
#endif

    // ------------------------------------------------------------------------
    // Alignment Routine: this routine aligns the motor to zero electrical
    // angle and in case of QEP also finds the index location and initializes
    // the angle w.r.t. the index location
    // ------------------------------------------------------------------------
    if(motorVars[1].runMotor == MOTOR_STOP)
    {
        motorVars[1].ptrFCL->lsw = ENC_ALIGNMENT;
        motorVars[1].IdRef = 0;
        motorVars[1].tempIdRef = motorVars[1].IdRef;
        motorVars[1].esmoForceRunCntr = 0;
        // Restart the ramp generator from the stator zero so every
        // STOP/RUN cycle aligns the rotor at the same electrical angle as
        // a fresh power-up, instead of the angle frozen at the last stop.
        motorVars[1].ptrFCL->rg.Angle = 0.0f;
        motorVars[1].ptrFCL->rg.Out = 0.0f;
        resetSensorlessEstimator(&motorVars[1]);
        resetESMOQepReference(&motorVars[1], MTR_2);
        FCL_resetController(&motorVars[1]);
    }
    else if(motorVars[1].ptrFCL->lsw == ENC_ALIGNMENT)
    {
        // alignment current
        motorVars[1].IdRef = motorVars[1].IdRef_start;  //(0.1);

        // set up an alignment and hold time for shaft to settle down
        if(motorVars[1].tempIdRef >= motorVars[1].IdRef)
        {
            motorVars[1].alignCntr++;

            if(motorVars[1].alignCntr >= motorVars[1].alignCnt)
            {
                motorVars[1].alignCntr  = 0;
                if((motorVars[1].positionFeedback == POSITION_FEEDBACK_ESMO) &&
                   (motorVars[1].ptrFCL->ptrQEP !=
                        (volatile struct EQEP_REGS *)0))
                {
                    // Same one-shot QEP compare frame latch as motor 1.
                    esmoQepIndexCalibrated[MTR_2] = 1U;
                    esmoQepIndexCount[MTR_2] = 0U;
                    if(esmoQepFrameCaptured[MTR_2] == 0U)
                    {
                        esmoQepFrameOffsetPu[MTR_2] =
                                normalizePuLocal(motorVars[1].ptrFCL->rg.Out);
                        motorVars[1].ptrFCL->ptrQEP->QPOSCNT = 0U;
                        // Discard any stale index latch so the index-based
                        // frame correction consumes an index from THIS run.
                        motorVars[1].ptrFCL->ptrQEP->QCLR.all =
                                ESMO_QEP_IEL_FLAG;
                        esmoQepIndexSeen[MTR_2] = 0U;
                        esmoQepFrameCaptured[MTR_2] = 1U;
                    }
                }

                // for QEP, spin the motor to find the index pulse
                motorVars[1].ptrFCL->lsw = ENC_WAIT_FOR_INDEX;
            }
        }
    } // end else if(lsw == ENC_ALIGNMENT)
    else if(motorVars[1].ptrFCL->lsw == ENC_CALIBRATION_DONE)
    {
        motorVars[1].IdRef = motorVars[1].IdRef_run;
    }

// -----------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control module
// -----------------------------------------------------------------------------
    if(motorVars[1].ptrFCL->lsw == ENC_ALIGNMENT)
    {
        motorVars[1].rc.TargetValue = 0;
        motorVars[1].rc.SetpointValue = 0;
    }
    else if((motorVars[1].ptrFCL->lsw == ENC_WAIT_FOR_INDEX) ||
            (isSensorlessTakeoverActive(&motorVars[1]) != 0U))
    {
        motorVars[1].rc.TargetValue =
                (isSensorlessControl(&motorVars[1]) ?
                 motorVars[1].esmoForceSpeed : motorVars[1].lsw1Speed) *
                (motorVars[1].speedRef > 0 ? 1 : -1);
    }
    else
    {
        motorVars[1].rc.TargetValue = motorVars[1].speedRef;
    }

    fclRampControl(&motorVars[1].rc);

// -----------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator module
// -----------------------------------------------------------------------------
    motorVars[1].ptrFCL->rg.Freq = motorVars[1].rc.SetpointValue;
    fclRampGen((RAMPGEN *)&motorVars[1].ptrFCL->rg);

    if(isSensorlessControl(&motorVars[1]) != 0U)
    {
        if(motorVars[1].ptrFCL->lsw == ENC_WAIT_FOR_INDEX)
        {
            if(motorVars[1].esmoForceRunCntr >=
                    motorVars[1].esmoForceRunCntMax)
            {
                if((absF32(motorVars[1].rc.SetpointValue) >=
                        M2_ESMO_TAKEOVER_MIN_SETPOINT) &&
                   (absF32(motorVars[1].esmoSpeedPu) >=
                        M2_ESMO_TAKEOVER_MIN_SPEED) &&
                   (isSensorlessReadyForClosedLoop(&motorVars[1]) != 0U))
                {
                    motorVars[1].esmoForceRunCntr = 0;
                    motorVars[1].esmoTakeoverCntr = 0;
#if(ESMO_COMPARE_LOG_M2_ENABLE != 0U)
                    if(esmoCompareLogMotor == MTR_2)
                    {
                        clearESMOHandoffLatch();
                        captureESMOHandoffLatch(&motorVars[1], MTR_2,
                                                ESMO_HANDOFF_LATCH_PRE_SYNC);
                    }
#endif
                    syncSensorlessEstimatorAngle(&motorVars[1],
                                                 motorVars[1].ptrFCL->rg.Out);
#if(ESMO_COMPARE_LOG_M2_ENABLE != 0U)
                    captureESMOHandoffLatch(&motorVars[1], MTR_2,
                                            ESMO_HANDOFF_LATCH_POST_SYNC);
#endif
                    seedSensorlessSpeedController(&motorVars[1]);
                    motorVars[1].ptrFCL->lsw = ENC_CALIBRATION_DONE;
                }
            }
            else
            {
                motorVars[1].esmoForceRunCntr++;
            }
        }
        else
        {
            motorVars[1].esmoForceRunCntr = 0;
        }

        if(motorVars[1].ptrFCL->lsw != ENC_CALIBRATION_DONE)
        {
            forceSensorlessAngle(&motorVars[1], motorVars[1].ptrFCL->rg.Out);
        }
    }

    runSensorlessEstimator(&motorVars[1]);

    if((isSensorlessControl(&motorVars[1]) != 0U) &&
       (motorVars[1].ptrFCL->lsw != ENC_CALIBRATION_DONE))
    {
        forceSensorlessAngle(&motorVars[1], motorVars[1].ptrFCL->rg.Out);
    }
    else if(isSensorlessTakeoverActive(&motorVars[1]) != 0U)
    {
        blendSensorlessAngle(&motorVars[1], motorVars[1].ptrFCL->rg.Out);
        if(motorVars[1].esmoTakeoverCntr >= motorVars[1].esmoTakeoverCntMax)
        {
#if(ESMO_COMPARE_LOG_M2_ENABLE != 0U)
            captureESMOHandoffLatch(&motorVars[1], MTR_2,
                                    ESMO_HANDOFF_LATCH_TAKEOVER_DONE);
#endif
        }
    }

// -----------------------------------------------------------------------------
//  Connect inputs of the SPEED_FR module and call the speed calculation module
// -----------------------------------------------------------------------------
    if(isSensorlessControl(&motorVars[1]) != 0U)
    {
        motorVars[1].posElecTheta = motorVars[1].esmoAnglePu;
        motorVars[1].posMechTheta = 0.0f;
        motorVars[1].speed.Speed =
                getSensorlessSpeedLoopFbk(&motorVars[1], MTR_2);
        motorVars[1].speed.SpeedRpm =
                motorVars[1].speed.BaseRpm * motorVars[1].speed.Speed;
    }
    else
    {
        motorVars[1].posElecTheta = motorVars[1].ptrFCL->qep.ElecTheta;
        motorVars[1].posMechTheta = motorVars[1].ptrFCL->qep.MechTheta;
        motorVars[1].speed.ElecTheta = motorVars[1].posElecTheta;
        runSpeedFR(&motorVars[1].speed);
    }

    updateESMOQepReference(&motorVars[1], MTR_2);

#if((BUILDLEVEL == FCL_LEVEL6)  && (SFRA_MOTOR == MOTOR_2))
// -----------------------------------------------------------------------------
//    SFRA collect routine, only to be called after SFRA inject has occurred 1st
// -----------------------------------------------------------------------------
    if(sfraCollectStart)
    {
        collectSFRA(&motorVars[1]);    // Collect noise feedback from loop
    }

// -----------------------------------------------------------------------------
//  SFRA injection
// -----------------------------------------------------------------------------
    injectSFRA();               // create SFRA Noise per 'sfraTestLoop'
    sfraCollectStart = 1;       // enable SFRA data collection
#endif

// -----------------------------------------------------------------------------
//    Connect inputs of the PI module and call the PID speed controller module
// -----------------------------------------------------------------------------
    motorVars[1].speedLoopCount++;

    if(motorVars[1].speedLoopCount >= motorVars[1].speedLoopPrescaler)
    {

#if((BUILDLEVEL == FCL_LEVEL6) && (SFRA_MOTOR == MOTOR_2))
        // SFRA Noise injection in speed loop
        motorVars[1].pid_spd.term.Ref =
                motorVars[1].rc.SetpointValue + sfraNoiseW;
#else   // #if(BUILDLEVEL == FCL_LEVEL4)
        motorVars[1].pid_spd.term.Ref =
                motorVars[1].rc.SetpointValue;  //speedRef;
#endif

        motorVars[1].pid_spd.term.Fbk = motorVars[1].speed.Speed;
        runPID(&motorVars[1].pid_spd);

        limitSensorlessTakeoverSpeedPid(&motorVars[1]);
        motorVars[1].speedLoopCount = 0;
    }

    if((motorVars[1].ptrFCL->lsw == ENC_ALIGNMENT) ||
            (motorVars[1].ptrFCL->lsw == ENC_WAIT_FOR_INDEX))
    {
        motorVars[1].pid_spd.data.d1 = 0;
        motorVars[1].pid_spd.data.d2 = 0;
        motorVars[1].pid_spd.data.i1 = 0;
        motorVars[1].pid_spd.data.ud = 0;
        motorVars[1].pid_spd.data.ui = motorVars[1].pid_spd.data.i1;
        motorVars[1].pid_spd.data.up = 0;
        motorVars[1].pid_spd.term.Out = 0;
    }

// -----------------------------------------------------------------------------
//    setup iqref and idref for FCL
// -----------------------------------------------------------------------------
#if((BUILDLEVEL == FCL_LEVEL6) && (SFRA_MOTOR == MOTOR_2))
    // SFRA Noise injection in Q axis
    motorVars[1].ptrFCL->pi_iq.ref =
            (motorVars[1].ptrFCL->lsw == ENC_ALIGNMENT) ? 0 :
                    (motorVars[1].ptrFCL->lsw == ENC_WAIT_FOR_INDEX) ?
                            getSensorlessStartupIqRef(&motorVars[1]) :
                            limitSensorlessTakeoverIqRef(&motorVars[1],
                                    (motorVars[1].pid_spd.term.Out +
                                     sfraNoiseQ));

    // SFRA Noise injection in D axis
    motorVars[1].tempIdRef =
            ramper(motorVars[1].IdRef, motorVars[1].tempIdRef, 0.00001);

    motorVars[1].pi_id.ref = motorVars[1].tempIdRef + sfraNoiseD;
#else   // #if(BUILDLEVEL == FCL_LEVEL4)
    // setup iqref (��ȥЭͬ���Ʋ���)
    motorVars[1].ptrFCL->pi_iq.ref =
            (motorVars[1].ptrFCL->lsw == ENC_ALIGNMENT) ? 0 :
                    (motorVars[1].ptrFCL->lsw == ENC_WAIT_FOR_INDEX) ?
                            getSensorlessStartupIqRef(&motorVars[1]) :
                            limitSensorlessTakeoverIqRef(&motorVars[1],
                                    (motorVars[1].pid_spd.term.Out +
                                     syncPI_out));

    // setup idref
    motorVars[1].tempIdRef = ramper(motorVars[1].IdRef,
                                    motorVars[1].tempIdRef, 0.00001);
    motorVars[1].pi_id.ref = motorVars[1].tempIdRef;
#endif

    // Field weakening: see buildLevel46_M1.
    runFieldWeakening(&motorVars[1], MTR_2);
    updateESMOVoltageUseMonitor(&motorVars[1], MTR_2);

    return;
 }
#endif // ( (BUILDLEVEL==FCL_LEVEL4) || (BUILDLEVEL == FCL_LEVEL6) )

//
//****************************************************************************
// INCRBUILD 5
//****************************************************************************
//
#if(BUILDLEVEL == FCL_LEVEL5)
// =============================== FCL_LEVEL 5 =================================
//  Level 5 verifies the position control
//  Position references generated locally from a posArray
//  lsw = ENC_ALIGNMENT      : lock the rotor of the motor
//  lsw = ENC_WAIT_FOR_INDEX : - needed only with QEP encoders until first
//                               index pulse
//                             - Loops shown for 'lsw=ENC_CALIBRATION_DONE' are
//                               closed in this stage
//  lsw = ENC_CALIBRATION_DONE : close all loops, position/speed/currents(Id/Iq)
//
//    NOTE:-
//       clarke1.As and clarke1.Bs are not brought out from the FCL library
//       as of library release version 0x02
//
// =============================================================================
// build level 5 subroutine for motor_1
#pragma FUNC_ALWAYS_INLINE(buildLevel5_M1)

static inline void buildLevel5_M1(void)
{

#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrl_M1(&motorVars[0]);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrl_M1(&motorVars[0]);
#endif

// -----------------------------------------------------------------------------
//    FCL_cycleCount calculations for debug
//    customer can remove the below code in final implementation
// -----------------------------------------------------------------------------
    getFCLTime(MTR_1);

// -----------------------------------------------------------------------------
//  Measure DC Bus voltage using SDFM Filter3
// -----------------------------------------------------------------------------
    motorVars[0].FCL_params.Vdcbus = getVdc(&motorVars[0]);

// -----------------------------------------------------------------------------
// Fast current loop controller wrapper
// -----------------------------------------------------------------------------
#if(FCL_CNTLR ==  PI_CNTLR)
   FCL_runPICtrlWrap_M1(&motorVars[0]);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
   FCL_runComplexCtrlWrap_M1(&motorVars[0]);
#endif

// -----------------------------------------------------------------------------
//  Alignment Routine: this routine aligns the motor to zero electrical angle
//  and in case of QEP also finds the index location and initializes the angle
//  w.r.t. the index location
// -----------------------------------------------------------------------------
    if(motorVars[0].runMotor == MOTOR_STOP)
    {
        motorVars[0].ptrFCL->lsw = ENC_ALIGNMENT;
        motorVars[0].lsw2EntryFlag = 0;
        motorVars[0].alignCntr = 0;
        motorVars[0].posCntr = 0;
        motorVars[0].posPtr = 0;
        motorVars[0].IdRef = 0;
        motorVars[0].pi_id.ref = motorVars[0].IdRef;
        FCL_resetController(&motorVars[0]);
    }
    else if(motorVars[0].ptrFCL->lsw == ENC_ALIGNMENT)
    {
        // alignment curretnt
        motorVars[0].IdRef = motorVars[0].IdRef_start;  //(0.1);

        // for restarting from (runMotor = STOP)
        motorVars[0].rc.TargetValue = 0;
        motorVars[0].rc.SetpointValue = 0;

        // set up an alignment and hold time for shaft to settle down
        if(motorVars[0].pi_id.ref >= motorVars[0].IdRef)
        {
            motorVars[0].alignCntr++;

            if(motorVars[0].alignCntr >= motorVars[0].alignCnt)
            {
                motorVars[0].alignCntr  = 0;

                // for QEP, spin the motor to find the index pulse
                motorVars[0].ptrFCL->lsw = ENC_WAIT_FOR_INDEX;
            }
        }
    } // end else if(lsw == ENC_ALIGNMENT)
    else if(motorVars[0].ptrFCL->lsw == ENC_CALIBRATION_DONE)
    {
        motorVars[0].IdRef = motorVars[0].IdRef_run;
    }

// -----------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator module
// -----------------------------------------------------------------------------
    motorVars[0].ptrFCL->rg.Freq = motorVars[0].speedRef * 0.1;
    fclRampGen((RAMPGEN *)&motorVars[0].ptrFCL->rg);

// -----------------------------------------------------------------------------
//   Connect inputs of the SPEED_FR module and call the speed calculation module
// -----------------------------------------------------------------------------
    motorVars[0].posElecTheta = motorVars[0].ptrFCL->qep.ElecTheta;
    motorVars[0].posMechTheta = motorVars[0].ptrFCL->qep.MechTheta;
    motorVars[0].speed.ElecTheta = motorVars[0].posElecTheta;
    runSpeedFR(&motorVars[0].speed);

// -----------------------------------------------------------------------------
//    Connect inputs of the PID module and call the PID speed controller module
// -----------------------------------------------------------------------------
    motorVars[0].speedLoopCount++;

    if(motorVars[0].speedLoopCount >= motorVars[0].speedLoopPrescaler)
    {
        if(motorVars[0].ptrFCL->lsw == ENC_CALIBRATION_DONE)
        {
            if(!motorVars[0].lsw2EntryFlag)
            {
                motorVars[0].lsw2EntryFlag = 1;
                motorVars[0].rc.TargetValue = motorVars[0].posMechTheta;
                motorVars[0].pi_pos.Fbk = motorVars[0].rc.TargetValue;
                motorVars[0].pi_pos.Ref = motorVars[0].pi_pos.Fbk;
            }
            else
            {
                // ========== reference position setting =========
                // choose between 1 of 2 position commands
                // The user can choose between a position reference table
                // used within refPosGen() or feed it in from rg1.Out
                // Position command read from a table
                motorVars[0].rc.TargetValue =
                        refPosGen(motorVars[0].rc.TargetValue, &motorVars[0]);

                motorVars[0].rc.SetpointValue = motorVars[0].rc.TargetValue -
                             (float32_t)((int32_t)motorVars[0].rc.TargetValue);

                // Rolling in angle within 0 to 1pu
                if(motorVars[0].rc.SetpointValue < 0)
                {
                    motorVars[0].rc.SetpointValue += 1.0;
                }

                motorVars[0].pi_pos.Ref = motorVars[0].rc.SetpointValue;
                motorVars[0].pi_pos.Fbk = motorVars[0].posMechTheta;
            }

            runPIPos(&motorVars[0].pi_pos);

            // speed PI regulator
            motorVars[0].pid_spd.term.Ref = motorVars[0].pi_pos.Out;
            motorVars[0].pid_spd.term.Fbk = motorVars[0].speed.Speed;
            runPID(&motorVars[0].pid_spd);
            limitSensorlessTakeoverSpeedPid(&motorVars[0]);
        }

        motorVars[0].speedLoopCount = 0;
    }

    if(motorVars[0].ptrFCL->lsw == ENC_ALIGNMENT)
    {
        motorVars[0].rc.SetpointValue = 0;  // position = 0 deg
        motorVars[0].pid_spd.data.d1 = 0;
        motorVars[0].pid_spd.data.d2 = 0;
        motorVars[0].pid_spd.data.i1 = 0;
        motorVars[0].pid_spd.data.ud = 0;
        motorVars[0].pid_spd.data.ui = 0;
        motorVars[0].pid_spd.data.up = 0;

        motorVars[0].pi_pos.ui = 0;
        motorVars[0].pi_pos.i1 = 0;

        motorVars[0].ptrFCL->rg.Out = 0;
        motorVars[0].lsw2EntryFlag = 0;
    }

// -----------------------------------------------------------------------------
//  Setup iqref for FCL
// -----------------------------------------------------------------------------
    motorVars[0].ptrFCL->pi_iq.ref =
            (motorVars[0].ptrFCL->lsw == ENC_ALIGNMENT) ? 0 :
                    (motorVars[0].ptrFCL->lsw == ENC_WAIT_FOR_INDEX) ?
                            getSensorlessStartupIqRef(&motorVars[0]) :
                            limitSensorlessTakeoverIqRef(&motorVars[0],
                                    motorVars[0].pid_spd.term.Out);

// -----------------------------------------------------------------------------
//  Setup idref for FCL
// -----------------------------------------------------------------------------
    motorVars[0].pi_id.ref =
            ramper(motorVars[0].IdRef, motorVars[0].pi_id.ref, 0.00001);

    return;
}

// build level 5 subroutine for motor_2
#pragma FUNC_ALWAYS_INLINE(buildLevel5_M2)

static inline void buildLevel5_M2(void)
{

#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrl_M2(&motorVars[1]);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrl_M2(&motorVars[1]);
#endif

// -----------------------------------------------------------------------------
//    FCL_cycleCount calculations for debug
//    customer can remove the below code in final implementation
// -----------------------------------------------------------------------------
    getFCLTime(MTR_2);

// -----------------------------------------------------------------------------
//  Measure DC Bus voltage using SDFM Filter3
// -----------------------------------------------------------------------------
    motorVars[1].FCL_params.Vdcbus = getVdc(&motorVars[1]);

// -----------------------------------------------------------------------------
// Fast current loop controller wrapper
// -----------------------------------------------------------------------------
#if(FCL_CNTLR ==  PI_CNTLR)
    FCL_runPICtrlWrap_M2(&motorVars[1]);
#endif

#if(FCL_CNTLR ==  CMPLX_CNTLR)
    FCL_runComplexCtrlWrap_M2(&motorVars[1]);
#endif

// -----------------------------------------------------------------------------
//  Alignment Routine: this routine aligns the motor to zero electrical angle
//  and in case of QEP also finds the index location and initializes the angle
//  w.r.t. the index location
// -----------------------------------------------------------------------------
    if(motorVars[1].runMotor == MOTOR_STOP)
    {
        motorVars[1].ptrFCL->lsw = ENC_ALIGNMENT;
        motorVars[1].lsw2EntryFlag = 0;
        motorVars[1].alignCntr = 0;
        motorVars[1].posCntr = 0;
        motorVars[1].posPtr = 0;
        motorVars[1].IdRef = 0;
        motorVars[1].pi_id.ref = motorVars[1].IdRef;
        FCL_resetController(&motorVars[1]);
    }
    else if(motorVars[1].ptrFCL->lsw == ENC_ALIGNMENT)
    {
        // alignment curretnt
        motorVars[1].IdRef = motorVars[1].IdRef_start;  //(0.1);

        // for restarting from (runMotor = STOP)
        motorVars[1].rc.TargetValue = 0;
        motorVars[1].rc.SetpointValue = 0;

        // set up an alignment and hold time for shaft to settle down
        if(motorVars[1].pi_id.ref >= motorVars[1].IdRef)
        {
            motorVars[1].alignCntr++;

            if(motorVars[1].alignCntr >= motorVars[1].alignCnt)
            {
                motorVars[1].alignCntr  = 0;

                // for QEP, spin the motor to find the index pulse
                motorVars[1].ptrFCL->lsw = ENC_WAIT_FOR_INDEX;
            }
        }
    } // end else if(lsw == ENC_ALIGNMENT)
    else if(motorVars[1].ptrFCL->lsw == ENC_CALIBRATION_DONE)
    {
        motorVars[1].IdRef = motorVars[1].IdRef_run;
    }

// -----------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator module
// -----------------------------------------------------------------------------
    motorVars[1].ptrFCL->rg.Freq = motorVars[1].speedRef * 0.1;
    fclRampGen((RAMPGEN *)&motorVars[1].ptrFCL->rg);

// -----------------------------------------------------------------------------
//  Connect inputs of the SPEED_FR module and call the speed calculation module
// -----------------------------------------------------------------------------
    motorVars[1].posElecTheta = motorVars[1].ptrFCL->qep.ElecTheta;
    motorVars[1].posMechTheta = motorVars[1].ptrFCL->qep.MechTheta;
    motorVars[1].speed.ElecTheta = motorVars[1].posElecTheta;
    runSpeedFR(&motorVars[1].speed);

// -----------------------------------------------------------------------------
//    Connect inputs of the PID module and call the PID speed controller module
// -----------------------------------------------------------------------------
    motorVars[1].speedLoopCount++;

    if(motorVars[1].speedLoopCount >= motorVars[1].speedLoopPrescaler)
    {
        if(motorVars[1].ptrFCL->lsw == ENC_CALIBRATION_DONE)
        {
            if(!motorVars[1].lsw2EntryFlag)
            {
                motorVars[1].lsw2EntryFlag = 1;
                motorVars[1].rc.TargetValue = motorVars[1].posMechTheta;
                motorVars[1].pi_pos.Fbk = motorVars[1].rc.TargetValue;
                motorVars[1].pi_pos.Ref = motorVars[1].pi_pos.Fbk;
            }
            else
            {
                // ========== reference position setting =========
#if(BUILDLEVEL == FCL_LEVEL5)
                // choose between 1 of 2 position commands
                // The user can choose between a position reference table
                // used within refPosGen() or feed it in from rg1.Out
                // Position command read from a table
                motorVars[1].rc.TargetValue =
                        refPosGen(motorVars[1].rc.TargetValue, &motorVars[1]);

#endif

                motorVars[1].rc.SetpointValue = motorVars[1].rc.TargetValue -
                             (float32_t)((int32_t)motorVars[1].rc.TargetValue);

                // Rolling in angle within 0 to 1pu
                if(motorVars[1].rc.SetpointValue < 0)
                {
                    motorVars[1].rc.SetpointValue += 1.0;
                }

                motorVars[1].pi_pos.Ref = motorVars[1].rc.SetpointValue;
                motorVars[1].pi_pos.Fbk = motorVars[1].posMechTheta;
            }

            runPIPos(&motorVars[1].pi_pos);

            // speed PI regulator
            motorVars[1].pid_spd.term.Ref = motorVars[1].pi_pos.Out;
            motorVars[1].pid_spd.term.Fbk = motorVars[1].speed.Speed;
            runPID(&motorVars[1].pid_spd);
            limitSensorlessTakeoverSpeedPid(&motorVars[1]);
        }

        motorVars[1].speedLoopCount = 0;
    }

    if(motorVars[1].ptrFCL->lsw == ENC_ALIGNMENT)
    {
        motorVars[1].rc.SetpointValue = 0;  // position = 0 deg
        motorVars[1].pid_spd.data.d1 = 0;
        motorVars[1].pid_spd.data.d2 = 0;
        motorVars[1].pid_spd.data.i1 = 0;
        motorVars[1].pid_spd.data.ud = 0;
        motorVars[1].pid_spd.data.ui = 0;
        motorVars[1].pid_spd.data.up = 0;
        motorVars[1].pi_pos.ui = 0;
        motorVars[1].pi_pos.i1 = 0;
        motorVars[1].ptrFCL->rg.Out = 0;
        motorVars[1].lsw2EntryFlag = 0;
    }

// -----------------------------------------------------------------------------
//  Setup iqref for FCL
// -----------------------------------------------------------------------------
    motorVars[1].ptrFCL->pi_iq.ref =
            (motorVars[1].ptrFCL->lsw == ENC_ALIGNMENT) ? 0 :
                    (motorVars[1].ptrFCL->lsw == ENC_WAIT_FOR_INDEX) ?
                            getSensorlessStartupIqRef(&motorVars[1]) :
                            limitSensorlessTakeoverIqRef(&motorVars[1],
                                    motorVars[1].pid_spd.term.Out);

// -----------------------------------------------------------------------------
//  Setup idref for FCL
// -----------------------------------------------------------------------------
    motorVars[1].pi_id.ref =
            ramper(motorVars[1].IdRef, motorVars[1].pi_id.ref, 0.00001);

    return;
}
#endif // (BUILDLEVEL==FCL_LEVEL5)

// ****************************************************************************
// ****************************************************************************
// Motor Control ISR
// ****************************************************************************
// ****************************************************************************

#pragma CODE_ALIGN(motor1ControlISR, 2)

__interrupt void motor1ControlISR(void)
{

#if(BUILDLEVEL == FCL_LEVEL1)
    buildLevel1_M1();

// -----------------------------------------------------------------------------
// Connect inputs of the DATALOG module
// -----------------------------------------------------------------------------
    dlogCh1 = motorVars[0].ptrFCL->rg.Out;
    dlogCh2 = motorVars[0].svgen.Ta;
    dlogCh3 = motorVars[0].svgen.Tb;
    dlogCh4 = motorVars[0].svgen.Tc;

#ifdef DACOUT_EN
//------------------------------------------------------------------------------
// Variable display on DACs
//------------------------------------------------------------------------------
    DAC_setShadowValue(hal.dacHandle[0],
                       DAC_MACRO_PU(motorVars[0].svgen.Ta));
    DAC_setShadowValue(hal.dacHandle[1],
                       DAC_MACRO_PU(motorVars[0].svgen.Tb));
#endif   // DACOUT_EN

#elif(BUILDLEVEL == FCL_LEVEL2)
    buildLevel2_M1();

// ----------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ----------------------------------------------------------------------------
    dlogCh1 = motorVars[0].ptrFCL->rg.Out;
    dlogCh2 = motorVars[0].speed.ElecTheta;
    dlogCh3 = motorVars[0].clarke.As;
    dlogCh4 = motorVars[0].clarke.Bs;

#ifdef DACOUT_EN
//-----------------------------------------------------------------------------
// Variable display on DACs
//-----------------------------------------------------------------------------
    DAC_setShadowValue(hal.dacHandle[0],
                       DAC_MACRO_PU(motorVars[0].ptrFCL->rg.Out));
    DAC_setShadowValue(hal.dacHandle[1],
                       DAC_MACRO_PU(motorVars[0].posElecTheta));
#endif   // DACOUT_EN

#elif(BUILDLEVEL == FCL_LEVEL3)
    buildLevel3_M1();

// ----------------------------------------------------------------------------
// Connect inputs of the DATALOG module
// ----------------------------------------------------------------------------
    dlogCh1 = motorVars[0].posElecTheta;
    dlogCh2 = motorVars[0].ptrFCL->rg.Out;
    dlogCh3 = motorVars[0].ptrFCL->pi_iq.ref;
    dlogCh4 = motorVars[0].ptrFCL->pi_iq.fbk;

#ifdef DACOUT_EN
//-----------------------------------------------------------------------------
// Variable display on DACs
//-----------------------------------------------------------------------------
    DAC_setShadowValue(hal.dacHandle[0],
                       DAC_MACRO_PU(motorVars[0].ptrFCL->pi_iq.ref));
    DAC_setShadowValue(hal.dacHandle[1],
                       DAC_MACRO_PU(motorVars[0].ptrFCL->pi_iq.fbk));
#endif   // DACOUT_EN

#elif(BUILDLEVEL == FCL_LEVEL4)
    buildLevel46_M1();
#if(ESMO_STARTUP_LOG_ENABLE != 0U)
    captureESMOStartupLog(&motorVars[0], MTR_1);
#endif
#if(ESMO_COMPARE_LOG_ENABLE != 0U)
    captureESMOCompareLog(&motorVars[0], MTR_1);
#endif

    //�����������
    int16_t ppbA = (int16_t)(*(volatile uint16_t *)(motorVars[0].curA_PPBRESULT));
    int16_t ppbB = (int16_t)(*(volatile uint16_t *)(motorVars[0].curB_PPBRESULT));
    int16_t ppbC = (int16_t)(*(volatile uint16_t *)(motorVars[0].curC_PPBRESULT));

    motorVars[0].currentAs = (float32_t)ppbA * motorVars[0].currentScale;
    motorVars[0].currentBs = (float32_t)ppbB * motorVars[0].currentScale;
    motorVars[0].currentCs = (float32_t)ppbC * motorVars[0].currentScale;
// -----------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// -----------------------------------------------------------------------------
    dlogCh1 = motorVars[0].posElecTheta;
    dlogCh2 = motorVars[0].speed.Speed;
    dlogCh3 = motorVars[0].pi_id.fbk;
    dlogCh4 = motorVars[0].ptrFCL->pi_iq.fbk;

#ifdef DACOUT_EN
//------------------------------------------------------------------------------
// Variable display on DACs
//------------------------------------------------------------------------------
   DAC_setShadowValue(hal.dacHandle[0],
                      DAC_MACRO_PU(motorVars[0].ptrFCL->pi_iq.fbk));
   DAC_setShadowValue(hal.dacHandle[1],
                      DAC_MACRO_PU(motorVars[0].pi_id.fbk));
#endif   // DACOUT_EN

#elif(BUILDLEVEL == FCL_LEVEL5)
    buildLevel5_M1();

// -----------------------------------------------------------------------------
//  Connect inputs of the DATALOG module
// -----------------------------------------------------------------------------
    dlogCh1 = motorVars[0].pi_pos.Ref;
    dlogCh2 = motorVars[0].pi_pos.Fbk;
    dlogCh3 = motorVars[0].pi_id.fbk;
    dlogCh4 = motorVars[0].ptrFCL->pi_iq.fbk;

#ifdef DACOUT_EN
//------------------------------------------------------------------------------
// Variable display on DACs B and C
//------------------------------------------------------------------------------
    DAC_setShadowValue(hal.dacHandle[0],
                       DAC_MACRO_PU(motorVars[0].pi_pos.Fbk));
    DAC_setShadowValue(hal.dacHandle[1],
                       DAC_MACRO_PU(motorVars[1].pi_pos.Fbk));
#endif   // DACOUT_EN

#elif(BUILDLEVEL == FCL_LEVEL6)
    buildLevel46_M1();
#if(ESMO_STARTUP_LOG_ENABLE != 0U)
    captureESMOStartupLog(&motorVars[0], MTR_1);
#endif
#if(ESMO_COMPARE_LOG_ENABLE != 0U)
    captureESMOCompareLog(&motorVars[0], MTR_1);
#endif

// -----------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// -----------------------------------------------------------------------------
    dlogCh1 = motorVars[0].posElecTheta;
    dlogCh2 = motorVars[0].speed.Speed;
    dlogCh3 = motorVars[0].pi_id.fbk;
    dlogCh4 = motorVars[0].ptrFCL->pi_iq.fbk;

#ifdef DACOUT_EN
//------------------------------------------------------------------------------
// Variable display on DACs
//------------------------------------------------------------------------------
       DAC_setShadowValue(hal.dacHandle[0],
                          DAC_MACRO_PU(motorVars[0].ptrFCL->pi_iq.fbk));
       DAC_setShadowValue(hal.dacHandle[1],
                          DAC_MACRO_PU(motorVars[0].pi_id.fbk));
#endif   // DACOUT_EN

#endif


// ----------------------------------------------------------------------------
//    Call the DATALOG update function.
// ----------------------------------------------------------------------------
    DLOG_4CH_F_FUNC(&dlog_4ch1);

    // Acknowledges an interrupt
    HAL_ackInt_M1(halMtrHandle[MTR_1]);

    motorVars[0].isrTicker++;

} // motor1ControlISR Ends Here


#pragma CODE_ALIGN(motor2ControlISR, 2)

//  motor2ControlISR()
__interrupt void motor2ControlISR(void)
{

#if(BUILDLEVEL == FCL_LEVEL1)
    buildLevel1_M2();

#elif(BUILDLEVEL == FCL_LEVEL2)
    buildLevel2_M2();

#elif(BUILDLEVEL == FCL_LEVEL3)
    buildLevel3_M2();

#elif(BUILDLEVEL == FCL_LEVEL4)
    buildLevel46_M2();
#if(ESMO_STARTUP_LOG_ENABLE != 0U)
    captureESMOStartupLog(&motorVars[1], MTR_2);
#endif
#if((ESMO_COMPARE_LOG_ENABLE != 0U) && (ESMO_COMPARE_LOG_M2_ENABLE != 0U))
    captureESMOCompareLog(&motorVars[1], MTR_2);
#endif

#elif(BUILDLEVEL == FCL_LEVEL5)
    buildLevel5_M2();

#elif(BUILDLEVEL == FCL_LEVEL6)
    buildLevel46_M2();
#if(ESMO_STARTUP_LOG_ENABLE != 0U)
    captureESMOStartupLog(&motorVars[1], MTR_2);
#endif
#if((ESMO_COMPARE_LOG_ENABLE != 0U) && (ESMO_COMPARE_LOG_M2_ENABLE != 0U))
    captureESMOCompareLog(&motorVars[1], MTR_2);
#endif
#endif


    // Acknowledges an interrupt
    HAL_ackInt_M2(halMtrHandle[MTR_2]);

    motorVars[1].isrTicker++;
} // motor1ControlISR Ends Here

//
// run the motor control
//
void runMotorControl(MOTOR_Vars_t *pMotor, HAL_MTR_Handle mtrHandle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)mtrHandle;
    uint16_t enableDriveGPIO = EN_GATE_M1;

    if(flagSyncRun == true)
    {
        if((motorVars[0].tripFlagDMC == 0) && (motorVars[1].tripFlagDMC == 0))
        {

#if(BUILDLEVEL != FCL_LEVEL5)
            motorVars[0].speedRef = speedRef;
            motorVars[1].speedRef = speedRef;
#endif

#if(BUILDLEVEL == FCL_LEVEL3)
            motorVars[0].IdRef_run = IdRef;
            motorVars[1].IdRef_run = IdRef;

            motorVars[0].IqRef = IqRef;
            motorVars[1].IqRef = IqRef;
#endif

            if(runMotor == MOTOR_RUN)
            {
                motorVars[0].runMotor = MOTOR_RUN;
                motorVars[1].runMotor = MOTOR_RUN;
            }
            else
            {
                motorVars[0].runMotor = MOTOR_STOP;
                motorVars[1].runMotor = MOTOR_STOP;
            }
        }
        else
        {
            motorVars[0].runMotor = MOTOR_STOP;
            motorVars[1].runMotor = MOTOR_STOP;
            motorVars[0].speedRef = 0.0;
            motorVars[1].speedRef = 0.0;
        }

    }

    // *******************************************************
    // Current limit setting / tuning in Debug environment
    // *******************************************************
    if(pMotor == &motorVars[0])
    {
        pMotor->curThreshHi = 2048 +
                (int16_t)M1_CURRENT_SCALE(pMotor->curLimit);
        pMotor->curThreshLo = 2048 -
                (int16_t)M1_CURRENT_SCALE(pMotor->curLimit);

        enableDriveGPIO = EN_GATE_M1;
    }
    else if(pMotor == &motorVars[1])
    {
        pMotor->curThreshHi = 2048 +
                (int16_t)M2_CURRENT_SCALE(pMotor->curLimit);
        pMotor->curThreshLo = 2048 -
                (int16_t)M2_CURRENT_SCALE(pMotor->curLimit);

        enableDriveGPIO = EN_GATE_M2;
    }

    HAL_setupCMPSS_DACValue(mtrHandle,
                            pMotor->curThreshHi, pMotor->curThreshLo);

    // Check for PWM trip due to over current
    if((EPWM_getTripZoneFlagStatus(obj->pwmHandle[0]) & EPWM_TZ_FLAG_OST) ||
       (EPWM_getTripZoneFlagStatus(obj->pwmHandle[1]) & EPWM_TZ_FLAG_OST) ||
       (EPWM_getTripZoneFlagStatus(obj->pwmHandle[2]) & EPWM_TZ_FLAG_OST))
    {
        // if any EPwm's OST is set, force OST on all three to DISABLE inverter
        EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
        EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
        EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

        pMotor->tripFlagDMC = 1;      // Trip on DMC (halt and IPM fault trip )
        pMotor->runMotor = MOTOR_STOP;

        GPIO_writePin(enableDriveGPIO, 1);
    }

    // If clear cmd received, reset PWM trip
    if(pMotor->clearTripFlagDMC)
    {
        // clear the ocp
        pMotor->tripFlagDMC = 0;
        pMotor->clearTripFlagDMC = 0;

        // clear EPWM trip flags
        DEVICE_DELAY_US(1L);

        // clear OST & DCAEVT1 flags
        EPWM_clearTripZoneFlag(obj->pwmHandle[0],
                               (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1));

        EPWM_clearTripZoneFlag(obj->pwmHandle[1],
                               (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1));

        EPWM_clearTripZoneFlag(obj->pwmHandle[2],
                               (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1));

        // clear HLATCH - (not in TRIP gen path)
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[2]);

        // clear LLATCH - (not in TRIP gen path)
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);
    }

    if(pMotor->runMotor == MOTOR_RUN)
    {
        GPIO_writePin(enableDriveGPIO, 0);
    }
    else
    {
        GPIO_writePin(enableDriveGPIO, 1);
    }

    return;
}

//
// POSITION LOOP UTILITY FUNCTIONS
//

// slew programmable ramper
float32_t ramper(float32_t in, float32_t out, float32_t rampDelta)
{
    float32_t err;

    err = in - out;

    if(err > rampDelta)
    {
        return(out + rampDelta);
    }
    else if(err < -rampDelta)
    {
        return(out - rampDelta);
    }
    else
    {
        return(in);
    }
}

//
// Reference Position Generator for position loop
//
float32_t refPosGen(float32_t out, MOTOR_Vars_t *pMotor)
{
    float32_t in = posArray[pMotor->posPtr];

    out = ramper(in, out, pMotor->posSlewRate);

    if(in == out)
    {
        pMotor->posCntr++;

        if(pMotor->posCntr > pMotor->posCntrMax)
        {
            pMotor->posCntr = 0;

            pMotor->posPtr++;

            if(pMotor->posPtr >= pMotor->posPtrMax)
            {
                pMotor->posPtr = 0;
            }
        }
    }

    return(out);
}

//*****************************************************************************
//*****************************************************************************
// Build level 6 : SFRA support functions
//*****************************************************************************
//*****************************************************************************
#if(BUILDLEVEL == FCL_LEVEL6)
// *************************************************************************
// Using SFRA tool :
// =================
//      - INJECT noise
//      - RUN the controller
//      - CAPTURE or COLLECT the controller output
// From a controller analysis standpoint, this sequence will reveal the
// output of controller for a given input, and therefore, good for analysis
// *************************************************************************
void injectSFRA(void)
{
    if(sfraTestLoop == SFRA_TEST_D_AXIS)
    {
        sfraNoiseD = SFRA_F32_inject(0.0);
    }
    else if(sfraTestLoop == SFRA_TEST_Q_AXIS)
    {
        sfraNoiseQ = SFRA_F32_inject(0.0);
    }
    else if(sfraTestLoop == SFRA_TEST_SPEEDLOOP)
    {
        sfraNoiseW = SFRA_F32_inject(0.0);
    }

    return;
}

// ****************************************************************************
void collectSFRA(MOTOR_Vars_t *pMotor)
{
    if(sfraTestLoop == SFRA_TEST_D_AXIS)
    {
        SFRA_F32_collect(&pMotor->pi_id.out,
                         &pMotor->pi_id.fbk);
    }
    else if(sfraTestLoop == SFRA_TEST_Q_AXIS)
    {
        SFRA_F32_collect(&pMotor->ptrFCL->pi_iq.out,
                         &pMotor->ptrFCL->pi_iq.fbk);
    }
    else if(sfraTestLoop == SFRA_TEST_SPEEDLOOP)
    {
        SFRA_F32_collect(&pMotor->pid_spd.term.Out,
                         &pMotor->pid_spd.term.Fbk);
    }

    return;
}
#endif

//
// End of Code
//
