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

// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ë«ï¿½ï¿½ï¿½ï¿½Ù¶ï¿½Í¬ï¿½ï¿½Ð­Í¬ PI ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
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

// Keep the M1-only logger disarmed during normal dual-motor operation. The
// 16Hz identification setting gives 6s across 96 entries, long enough to
// retain the measured 0.5pu-to-0.6pu ramp (about 2.5s) and both steady states.
#define ESMO_COMPARE_LOG_ENABLE      1U
#define ESMO_COMPARE_LOG_SIZE        96U
#define ESMO_COMPARE_LOG_RATE_HZ     16U
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
volatile float32_t esmoCompareTriggerCommandPu = 0.0f;
volatile float32_t esmoCompareTriggerSpeedPu = 0.0f;
volatile uint32_t esmoCompareLogCallCount = 0U;
volatile uint16_t esmoCompareLogGateMask = 0U;

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
// STOP/RUN cycles, so the alignment ãmxòÚ$z{-®éÜj×¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢ Ð¢òò¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢ Ð Ð¢7&vÖ4ôDUôÄ”tâ†Ö÷F÷#6öçG&öÄ•5"Â"Ð Ð¥õö–çFW''WBfö–BÖ÷F÷#6öçG&öÄ•5"‡fö–BÐ§°Ð Ð¢6–b„%T”ÄDÄUdTÂÓÒd4ÅôÄUdTÃÐ¢'V–ÆDÆWfVÃôÓ‚“°Ð Ð¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢òò6öææV7B–çWG2öbF†RDDÄôrÖöGVÆPÐ¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢FÆöt6ƒÒÖ÷F÷%f'5³ÒçG$d4ÂÓç&rä÷WC°Ð¢FÆöt6ƒ"ÒÖ÷F÷%f'5³Òç7fvVâåF°Ð¢FÆöt6ƒ2ÒÖ÷F÷%f'5³Òç7fvVâåF#°Ð¢FÆöt6ƒBÒÖ÷F÷%f'5³Òç7fvVâåF3°Ð Ð¢6–fFVbD4õUEôTàÐ¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢òòf&–&ÆRF—7Æ’öâD70Ð¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢D5÷6WE6†F÷ufÇVR††ÂæF4†æFÆU³ÒÀÐ¢D5ôÔ5$õõR†Ö÷F÷%f'5³Òç7fvVâåF’“°Ð¢D5÷6WE6†F÷ufÇVR††ÂæF4†æFÆU³ÒÀÐ¢D5ôÔ5$õõR†Ö÷F÷%f'5³Òç7fvVâåF"’“°Ð¢6VæF–bòòD4õUEôTàÐ Ð¢6VÆ–b„%T”ÄDÄUdTÂÓÒd4ÅôÄUdTÃ"Ð¢'V–ÆDÆWfVÃ%ôÓ‚“°Ð Ð¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢òò6öææV7B–çWG2öbF†RDDÄôrÖöGVÆPÐ¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢FÆöt6ƒÒÖ÷F÷%f'5³ÒçG$d4ÂÓç&rä÷WC°Ð¢FÆöt6ƒ"ÒÖ÷F÷%f'5³Òç7VVBäVÆV5F†WF°Ð¢FÆöt6ƒ2ÒÖ÷F÷%f'5³Òæ6Æ&¶Rä3°Ð¢FÆöt6ƒBÒÖ÷F÷%f'5³Òæ6Æ&¶Rä'3°Ð Ð¢6–fFVbD4õUEôTàÐ¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢òòf&–&ÆRF—7Æ’öâD70Ð¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢D5÷6WE6†F÷ufÇVR††ÂæF4†æFÆU³ÒÀÐ¢D5ôÔ5$õõR†Ö÷F÷%f'5³ÒçG$d4ÂÓç&rä÷WB’“°Ð¢D5÷6WE6†F÷ufÇVR††ÂæF4†æFÆU³ÒÀÐ¢D5ôÔ5$õõR†Ö÷F÷%f'5³Òç÷4VÆV5F†WF’“°Ð¢6VæF–bòòD4õUEôTàÐ Ð¢6VÆ–b„%T”ÄDÄUdTÂÓÒd4ÅôÄUdTÃ2Ð¢'V–ÆDÆWfVÃ5ôÓ‚“°Ð Ð¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢òò6öææV7B–çWG2öbF†RDDÄôrÖöGVÆPÐ¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢FÆöt6ƒÒÖ÷F÷%f'5³Òç÷4VÆV5F†WF°Ð¢FÆöt6ƒ"ÒÖ÷F÷%f'5³ÒçG$d4ÂÓç&rä÷WC°Ð¢FÆöt6ƒ2ÒÖ÷F÷%f'5³ÒçG$d4ÂÓç•ö—ç&Vc°Ð¢FÆöt6ƒBÒÖ÷F÷%f'5³ÒçG$d4ÂÓç•ö—æf&³°Ð Ð¢6–fFVbD4õUEôTàÐ¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢òòf&–&ÆRF—7Æ’öâD70Ð¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢D5÷6WE6†F÷ufÇVR††ÂæF4†æFÆU³ÒÀÐ¢D5ôÔ5$õõR†Ö÷F÷%f'5³ÒçG$d4ÂÓç•ö—ç&Vb’“°Ð¢D5÷6WE6†F÷ufÇVR††ÂæF4†æFÆU³ÒÀÐ¢D5ôÔ5$õõR†Ö÷F÷%f'5³ÒçG$d4ÂÓç•ö—æf&²’“°Ð¢6VæF–bòòD4õUEôTàÐ Ð¢6VÆ–b„%T”ÄDÄUdTÂÓÒd4ÅôÄUdTÃBÐ¢'V–ÆDÆWfVÃCeôÓ‚“°Ð¢6–b„U4Ôõõ5D%EUôÄôuôTä$ÄRÒRÐ¢6GW&TU4Ôõ7F'GWÆör‚fÖ÷F÷%f'5³ÒÂÕE%ó“°Ð¢6VæF–`Ð¢6–b„U4Ôõô4ôÕ$UôÄôuôTä$ÄRÒRÐ¢6GW&TU4Ôô6ö×&TÆör‚fÖ÷F÷%f'5³ÒÂÕE%ó“°Ð¢6VæF–`Ð Ð¢òþûûÞûûÞûûÞûûÞûûÞûûÞûûÞûûÞûûÞûûÞûûÐÐ¢–çCe÷B$Ò†–çCe÷B’‚¢‡föÆF–ÆRV–çCe÷B¢’†Ö÷F÷%f'5³Òæ7W$õ%$U5TÅB’“°Ð¢–çCe÷B$"Ò†–çCe÷B’‚¢‡föÆF–ÆRV–çCe÷B¢’†Ö÷F÷%f'5³Òæ7W$%õ%$U5TÅB’“°Ð¢–çCe÷B$2Ò†–çCe÷B’‚¢‡föÆF–ÆRV–çCe÷B¢’†Ö÷F÷%f'5³Òæ7W$5õ%$U5TÅB’“°Ð Ð¢Ö÷F÷%f'5³Òæ7W'&VçD2Ò†fÆöC3%÷B—$¢Ö÷F÷%f'5³Òæ7W'&VçE66ÆS°Ð¢Ö÷F÷%f'5³Òæ7W'&VçD'2Ò†fÆöC3%÷B—$"¢Ö÷F÷%f'5³Òæ7W'&VçE66ÆS°Ð¢Ö÷F÷%f'5³Òæ7W'&VçD72Ò†fÆöC3%÷B—$2¢Ö÷F÷%f'5³Òæ7W'&VçE66ÆS°Ð¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢òò6öææV7B–çWG2öbF†RDDÄôrÖöGVÆPÐ¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢FÆöt6ƒÒÖ÷F÷%f'5³Òç÷4VÆV5F†WF°Ð¢FÆöt6ƒ"ÒÖ÷F÷%f'5³Òç7VVBå7VVC°Ð¢FÆöt6ƒ2ÒÖ÷F÷%f'5³Òç•ö–Bæf&³°Ð¢FÆöt6ƒBÒÖ÷F÷%f'5³ÒçG$d4ÂÓç•ö—æf&³°Ð Ð¢6–fFVbD4õUEôTàÐ¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢òòf&–&ÆRF—7Æ’öâD70Ð¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢D5÷6WE6†F÷ufÇVR††ÂæF4†æFÆU³ÒÀÐ¢D5ôÔ5$õõR†Ö÷F÷%f'5³ÒçG$d4ÂÓç•ö—æf&²’“°Ð¢D5÷6WE6†F÷ufÇVR††ÂæF4†æFÆU³ÒÀÐ¢D5ôÔ5$õõR†Ö÷F÷%f'5³Òç•ö–Bæf&²’“°Ð¢6VæF–bòòD4õUEôTàÐ Ð¢6VÆ–b„%T”ÄDÄUdTÂÓÒd4ÅôÄUdTÃRÐ¢'V–ÆDÆWfVÃUôÓ‚“°Ð Ð¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢òò6öææV7B–çWG2öbF†RDDÄôrÖöGVÆPÐ¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢FÆöt6ƒÒÖ÷F÷%f'5³Òç•÷÷2å&Vc°Ð¢FÆöt6ƒ"ÒÖ÷F÷%f'5³Òç•÷÷2äf&³°Ð¢FÆöt6ƒ2ÒÖ÷F÷%f'5³Òç•ö–Bæf&³°Ð¢FÆöt6ƒBÒÖ÷F÷%f'5³ÒçG$d4ÂÓç•ö—æf&³°Ð Ð¢6–fFVbD4õUEôTàÐ¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢òòf&–&ÆRF—7Æ’öâD72"æB0Ð¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢D5÷6WE6†F÷ufÇVR††ÂæF4†æFÆU³ÒÀÐ¢D5ôÔ5$õõR†Ö÷F÷%f'5³Òç•÷÷2äf&²’“°Ð¢D5÷6WE6†F÷ufÇVR††ÂæF4†æFÆU³ÒÀÐ¢D5ôÔ5$õõR†Ö÷F÷%f'5³Òç•÷÷2äf&²’“°Ð¢6VæF–bòòD4õUEôTàÐ Ð¢6VÆ–b„%T”ÄDÄUdTÂÓÒd4ÅôÄUdTÃbÐ¢'V–ÆDÆWfVÃCeôÓ‚“°Ð¢6–b„U4Ôõõ5D%EUôÄôuôTä$ÄRÒRÐ¢6GW&TU4Ôõ7F'GWÆör‚fÖ÷F÷%f'5³ÒÂÕE%ó“°Ð¢6VæF–`Ð¢6–b„U4Ôõô4ôÕ$UôÄôuôTä$ÄRÒRÐ¢6GW&TU4Ôô6ö×&TÆör‚fÖ÷F÷%f'5³ÒÂÕE%ó“°Ð¢6VæF–`Ð Ð¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢òò6öææV7B–çWG2öbF†RDDÄôrÖöGVÆPÐ¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢FÆöt6ƒÒÖ÷F÷%f'5³Òç÷4VÆV5F†WF°Ð¢FÆöt6ƒ"ÒÖ÷F÷%f'5³Òç7VVBå7VVC°Ð¢FÆöt6ƒ2ÒÖ÷F÷%f'5³Òç•ö–Bæf&³°Ð¢FÆöt6ƒBÒÖ÷F÷%f'5³ÒçG$d4ÂÓç•ö—æf&³°Ð Ð¢6–fFVbD4õUEôTàÐ¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢òòf&–&ÆRF—7Æ’öâD70Ð¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢D5÷6WE6†F÷ufÇVR††ÂæF4†æFÆU³ÒÀÐ¢D5ôÔ5$õõR†Ö÷F÷%f'5³ÒçG$d4ÂÓç•ö—æf&²’“°Ð¢D5÷6WE6†F÷ufÇVR††ÂæF4†æFÆU³ÒÀÐ¢D5ôÔ5$õõR†Ö÷F÷%f'5³Òç•ö–Bæf&²’“°Ð¢6VæF–bòòD4õUEôTàÐ Ð¢6VæF–`Ð Ð Ð¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢òò6ÆÂF†RDDÄôrWFFRgVæ7F–öâàÐ¢òòÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÒÐÐ¢DÄôuóD4…ôeôeTä2‚fFÆöuóF6ƒ“°Ð Ð¢òò6¶æ÷vÆVFvW2â–çFW''W@Ð¢„Åö6´–çEôÓ††Ä×G$†æFÆU´ÕE%óÒ“°Ð Ð¢Ö÷F÷%f'5³Òæ—7%F–6¶W"²³°Ð Ð§ÒòòÖ÷F÷#6öçG&öÄ•5"VæG2†W&PÐ Ð Ð¢7&vÖ4ôDUôÄ”tâ†Ö÷F÷#$6öçG&öÄ•5"Â"Ð Ð¢òòÖ÷F÷#$6öçG&öÄ•5"‚Ð¥õö–çFW''WBfö–BÖ÷F÷#$6öçG&öÄ•5"‡fö–BÐ§°Ð Ð¢6–b„%T”ÄDÄUdTÂÓÒd4ÅôÄUdTÃÐ¢'V–ÆDÆWfVÃôÓ"‚“°Ð Ð¢6VÆ–b„%T”ÄDÄUdTÂÓÒd4ÅôÄUdTÃ"Ð¢'V–ÆDÆWfVÃ%ôÓ"‚“°Ð Ð¢6VÆ–b„%T”ÄDÄUdTÂÓÒd4ÅôÄUdTÃ2Ð¢'V–ÆDÆWfVÃ5ôÓ"‚“°Ð Ð¢6VÆ–b„%T”ÄDÄUdTÂÓÒd4ÅôÄUdTÃBÐ¢'V–ÆDÆWfVÃCeôÓ"‚“°Ð¢6–b„U4Ôõõ5D%EUôÄôuôTä$ÄRÒRÐ¢6GW&TU4Ôõ7F'GWÆör‚fÖ÷F÷%f'5³ÒÂÕE%ó"“°Ð¢6VæF–`Ð¢6–b‚„U4Ôõô4ôÕ$UôÄôuôTä$ÄRÒR’bb„U4Ôõô4ôÕ$UôÄôuôÓ%ôTä$ÄRÒR’Ð¢6GW&TU4Ôô6ö×&TÆör‚fÖ÷F÷%f'5³ÒÂÕE%ó"“°Ð¢6VæF–`Ð Ð¢6VÆ–b„%T”ÄDÄUdTÂÓÒd4ÅôÄUdTÃRÐ¢'V–ÆDÆWfVÃUôÓ"‚“°Ð Ð¢6VÆ–b„%T”ÄDÄUdTÂÓÒd4ÅôÄUdTÃbÐ¢'V–ÆDÆWfVÃCeôÓ"‚“°Ð¢6–b„U4Ôõõ5D%EUôÄôuôTä$ÄRÒRÐ¢6GW&TU4Ôõ7F'GWÆör‚fÖ÷F÷%f'5³ÒÂÕE%ó"“°Ð¢6VæF–`Ð¢6–b‚„U4Ôõô4ôÕ$UôÄôuôTä$ÄRÒR’bb„U4Ôõô4ôÕ$UôÄôuôÓ%ôTä$ÄRÒR’Ð¢6GW&TU4Ôô6ö×&TÆör‚fÖ÷F÷%f'5³ÒÂÕE%ó"“°Ð¢6VæF–`Ð¢6VæF–`Ð Ð Ð¢òò6¶æ÷vÆVFvW2â–çFW''W@Ð¢„Åö6´–çEôÓ"††Ä×G$†æFÆU´ÕE%ó%Ò“°Ð Ð¢Ö÷F÷%f'5³Òæ—7%F–6¶W"²³°Ð§ÒòòÖ÷F÷#6öçG&öÄ•5"VæG2†W&PÐ Ð¢òðÐ¢òò'VâF†RÖ÷F÷"6öçG&öÀÐ¢òðÐ§fö–B'VäÖ÷F÷$6öçG&öÂ„ÔõDõ%õf'5÷B§Ö÷F÷"Â„ÅôÕE%ô†æFÆR×G$†æFÆRÐ§°Ð¢„ÅôÕE%ôö&¢¦ö&¢Ò„„ÅôÕE%ôö&¢¢–×G$†æFÆS°Ð¢V–çCe÷BVæ&ÆTG&—fTu”òÒTåôtDUôÓ°Ð Ð¢–b†fÆu7–æ5'VâÓÒG'VRÐ¢°Ð¢–b‚†Ö÷F÷%f'5³ÒçG&—fÆtDÔ2ÓÒ’bb†Ö÷F÷%f'5³ÒçG&—fÆtDÔ2ÓÒ’Ð¢°Ð Ð¢6–b„%T”ÄDÄUdTÂÒd4ÅôÄUdTÃRÐ¢Ö÷F÷%f'5³Òç7VVE&VbÒ7VVE&Vc°Ð¢Ö÷F÷%f'5³Òç7VVE&VbÒ7VVE&Vc°Ð¢6VæF–`Ð Ð¢6–b„%T”ÄDÄUdTÂÓÒd4ÅôÄUdTÃ2Ð¢Ö÷F÷%f'5³Òä–E&Ve÷'VâÒ–E&Vc°Ð¢Ö÷F÷%f'5³Òä–E&Ve÷'VâÒ–E&Vc°Ð Ð¢Ö÷F÷%f'5³Òä—&VbÒ—&Vc°Ð¢Ö÷F÷%f'5³Òä—&VbÒ—&Vc°Ð¢6VæF–`Ð Ð¢–b‡'VäÖ÷F÷"ÓÒÔõDõ%õ%TâÐ¢°Ð¢Ö÷F÷%f'5³Òç'VäÖ÷F÷"ÒÔõDõ%õ%Tã°Ð¢Ö÷F÷%f'5³Òç'VäÖ÷F÷"ÒÔõDõ%õ%Tã°Ð¢ÐÐ¢VÇ6PÐ¢°Ð¢Ö÷F÷%f'5³Òç'VäÖ÷F÷"ÒÔõDõ%õ5Dõ°Ð¢Ö÷F÷%f'5³Òç'VäÖ÷F÷"ÒÔõDõ%õ5Dõ°Ð¢ÐÐ¢ÐÐ¢VÇ6PÐ¢°Ð¢Ö÷F÷%f'5³Òç'VäÖ÷F÷"ÒÔõDõ%õ5Dõ°Ð¢Ö÷F÷%f'5³Òç'VäÖ÷F÷"ÒÔõDõ%õ5Dõ°Ð¢Ö÷F÷%f'5³Òç7VVE&VbÒã°Ð¢Ö÷F÷%f'5³Òç7VVE&VbÒã°Ð¢ÐÐ Ð¢ÐÐ Ð¢òò¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢ Ð¢òò7W'&VçBÆ–Ö—B6WGF–æròGVæ–ær–âFV'VrVçf—&öæÖVç@Ð¢òò¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢ Ð¢–b‡Ö÷F÷"ÓÒfÖ÷F÷%f'5³ÒÐ¢°Ð¢Ö÷F÷"Óæ7W%F‡&W6„†’Ò#C‚°Ð¢†–çCe÷B”Óô5U%$TåEõ44ÄR‡Ö÷F÷"Óæ7W$Æ–Ö—B“°Ð¢Ö÷F÷"Óæ7W%F‡&W6„ÆòÒ#C‚ÐÐ¢†–çCe÷B”Óô5U%$TåEõ44ÄR‡Ö÷F÷"Óæ7W$Æ–Ö—B“°Ð Ð¢Væ&ÆTG&—fTu”òÒTåôtDUôÓ°Ð¢ÐÐ¢VÇ6R–b‡Ö÷F÷"ÓÒfÖ÷F÷%f'5³ÒÐ¢°Ð¢Ö÷F÷"Óæ7W%F‡&W6„†’Ò#C‚°Ð¢†–çCe÷B”Ó%ô5U%$TåEõ44ÄR‡Ö÷F÷"Óæ7W$Æ–Ö—B“°Ð¢Ö÷F÷"Óæ7W%F‡&W6„ÆòÒ#C‚ÐÐ¢†–çCe÷B”Ó%ô5U%$TåEõ44ÄR‡Ö÷F÷"Óæ7W$Æ–Ö—B“°Ð Ð¢Væ&ÆTG&—fTu”òÒTåôtDUôÓ#°Ð¢ÐÐ Ð¢„Å÷6WGW4Õ55ôD5fÇVR†×G$†æFÆRÀÐ¢Ö÷F÷"Óæ7W%F‡&W6„†’ÂÖ÷F÷"Óæ7W%F‡&W6„Æò“°Ð Ð¢òò6†V6²f÷"tÒG&—GVRFò÷fW"7W'&Vç@Ð¢–b‚„UtÕövWEG&—¦öæTfÆu7FGW2†ö&¢ÓçvÔ†æFÆU³Ò’bUtÕõE¥ôdÄuôõ5B’ÇÀÐ¢„UtÕövWEG&—¦öæTfÆu7FGW2†ö&¢ÓçvÔ†æFÆU³Ò’bUtÕõE¥ôdÄuôõ5B’ÇÀÐ¢„UtÕövWEG&—¦öæTfÆu7FGW2†ö&¢ÓçvÔ†æFÆU³%Ò’bUtÕõE¥ôdÄuôõ5B’Ð¢°Ð¢òò–bç’UvÒw2õ5B—26WBÂf÷&6Rõ5BöâÆÂF‡&VRFòD•4$ÄR–çfW'FW Ð¢UtÕöf÷&6UG&—¦öæTWfVçB†ö&¢ÓçvÔ†æFÆU³ÒÂUtÕõE¥ôdõ$4UôUdTåEôõ5B“°Ð¢UtÕöf÷&6UG&—¦öæTWfVçB†ö&¢ÓçvÔ†æFÆU³ÒÂUtÕõE¥ôdõ$4UôUdTåEôõ5B“°Ð¢UtÕöf÷&6UG&—¦öæTWfVçB†ö&¢ÓçvÔ†æFÆU³%ÒÂUtÕõE¥ôdõ$4UôUdTåEôõ5B“°Ð Ð¢Ö÷F÷"ÓçG&—fÆtDÔ2Ò²òòG&—öâDÔ2††ÇBæB•ÒfVÇBG&—Ð¢Ö÷F÷"Óç'VäÖ÷F÷"ÒÔõDõ%õ5Dõ°Ð Ð¢u”õ÷w&—FU–â†Væ&ÆTG&—fTu”òÂ“°Ð¢ÐÐ Ð¢òò–b6ÆV"6ÖB&V6V—fVBÂ&W6WBtÒG&— Ð¢–b‡Ö÷F÷"Óæ6ÆV%G&—fÆtDÔ2Ð¢°Ð¢òò6ÆV"F†Rö7 Ð¢Ö÷F÷"ÓçG&—fÆtDÔ2Ò°Ð¢Ö÷F÷"Óæ6ÆV%G&—fÆtDÔ2Ò°Ð Ð¢òò6ÆV"UtÒG&—fÆw0Ð¢DUd”4UôDTÄ•õU2ƒÂ“°Ð Ð¢òò6ÆV"õ5BbD4UeCfÆw0Ð¢UtÕö6ÆV%G&—¦öæTfÆr†ö&¢ÓçvÔ†æFÆU³ÒÀÐ¢„UtÕõE¥ôdÄuôõ5BÂUtÕõE¥ôdÄuôD4UeC’“°Ð Ð¢UtÕö6ÆV%G&—¦öæTfÆr†ö&¢ÓçvÔ†æFÆU³ÒÀÐ¢„UtÕõE¥ôdÄuôõ5BÂUtÕõE¥ôdÄuôD4UeC’“°Ð Ð¢UtÕö6ÆV%G&—¦öæTfÆr†ö&¢ÓçvÔ†æFÆU³%ÒÀÐ¢„UtÕõE¥ôdÄuôõ5BÂUtÕõE¥ôdÄuôD4UeC’“°Ð Ð¢òò6ÆV"„ÄD4‚Ò†æ÷B–âE$•vVâF‚Ð¢4Õ55ö6ÆV$f–ÇFW$ÆF6„†–v‚†ö&¢Óæ6×74†æFÆU³Ò“°Ð¢4Õ55ö6ÆV$f–ÇFW$ÆF6„†–v‚†ö&¢Óæ6×74†æFÆU³Ò“°Ð¢4Õ55ö6ÆV$f–ÇFW$ÆF6„†–v‚†ö&¢Óæ6×74†æFÆU³%Ò“°Ð Ð¢òò6ÆV"ÄÄD4‚Ò†æ÷B–âE$•vVâF‚Ð¢4Õ55ö6ÆV$f–ÇFW$ÆF6„Æ÷r†ö&¢Óæ6×74†æFÆU³Ò“°Ð¢4Õ55ö6ÆV$f–ÇFW$ÆF6„Æ÷r†ö&¢Óæ6×74†æFÆU³Ò“°Ð¢4Õ55ö6ÆV$f–ÇFW$ÆF6„Æ÷r†ö&¢Óæ6×74†æFÆU³%Ò“°Ð¢ÐÐ Ð¢–b‡Ö÷F÷"Óç'VäÖ÷F÷"ÓÒÔõDõ%õ%TâÐ¢°Ð¢u”õ÷w&—FU–â†Væ&ÆTG&—fTu”òÂ“°Ð¢ÐÐ¢VÇ6PÐ¢°Ð¢u”õ÷w&—FU–â†Væ&ÆTG&—fTu”òÂ“°Ð¢ÐÐ Ð¢&WGW&ã°Ð§ÐÐ Ð¢òðÐ¢òòõ4•D”ôâÄôõUD”Ä•E’eTä5D”ôå0Ð¢òðÐ Ð¢òò6ÆWr&öw&ÖÖ&ÆR&×W Ð¦fÆöC3%÷B&×W"†fÆöC3%÷B–âÂfÆöC3%÷B÷WBÂfÆöC3%÷B&×FVÇFÐ§°Ð¢fÆöC3%÷BW'#°Ð Ð¢W'"Ò–âÒ÷WC°Ð Ð¢–b†W'"â&×FVÇFÐ¢°Ð¢&WGW&â†÷WB²&×FVÇF“°Ð¢ÐÐ¢VÇ6R–b†W'"Â×&×FVÇFÐ¢°Ð¢&WGW&â†÷WBÒ&×FVÇF“°Ð¢ÐÐ¢VÇ6PÐ¢°Ð¢&WGW&â†–â“°Ð¢ÐÐ§ÐÐ Ð¢òðÐ¢òò&VfW&Væ6R÷6—F–öâvVæW&F÷"f÷"÷6—F–öâÆö÷ Ð¢òðÐ¦fÆöC3%÷B&Ve÷4vVâ†fÆöC3%÷B÷WBÂÔõDõ%õf'5÷B§Ö÷F÷"Ð§°Ð¢fÆöC3%÷B–âÒ÷4'&•·Ö÷F÷"Óç÷5G%Ó°Ð Ð¢÷WBÒ&×W"†–âÂ÷WBÂÖ÷F÷"Óç÷56ÆWu&FR“°Ð Ð¢–b†–âÓÒ÷WBÐ¢°Ð¢Ö÷F÷"Óç÷46çG"²³°Ð Ð¢–b‡Ö÷F÷"Óç÷46çG"âÖ÷F÷"Óç÷46çG$Ö‚Ð¢°Ð¢Ö÷F÷"Óç÷46çG"Ò°Ð Ð¢Ö÷F÷"Óç÷5G"²³°Ð Ð¢–b‡Ö÷F÷"Óç÷5G"ãÒÖ÷F÷"Óç÷5G$Ö‚Ð¢°Ð¢Ö÷F÷"Óç÷5G"Ò°Ð¢ÐÐ¢ÐÐ¢ÐÐ Ð¢&WGW&â†÷WB“°Ð§ÐÐ Ð¢òò¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢ Ð¢òò¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢ Ð¢òò'V–ÆBÆWfVÂb¢4e$7W÷'BgVæ7F–öç0Ð¢òò¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢ Ð¢òò¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢ Ð¢6–b„%T”ÄDÄUdTÂÓÒd4ÅôÄUdTÃbÐ¢òò¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢ Ð¢òòW6–ær4e$FööÂ Ð¢òòÓÓÓÓÓÓÓÓÓÓÓÓÓÓÓÓÐÐ¢òòÒ”ä¤T5Bæö—6PÐ¢òòÒ%TâF†R6öçG&öÆÆW Ð¢òòÒ4EU$R÷"4ôÄÄT5BF†R6öçG&öÆÆW"÷WGW@Ð¢òòg&öÒ6öçG&öÆÆW"æÇ—6—27FæGö–çBÂF†—26WVVæ6Rv–ÆÂ&WfVÂF†PÐ¢òò÷WGWBöb6öçG&öÆÆW"f÷"v—fVâ–çWBÂæBF†W&Vf÷&RÂvööBf÷"æÇ—6—0Ð¢òò¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢ Ð§fö–B–æ¦V7E4e$‡fö–BÐ§°Ð¢–b‡6g&FW7DÆö÷ÓÒ4e$õDU5EôEô„•2Ð¢°Ð¢6g&æö—6TBÒ4e$ôc3%ö–æ¦V7Bƒã“°Ð¢ÐÐ¢VÇ6R–b‡6g&FW7DÆö÷ÓÒ4e$õDU5Eõô„•2Ð¢°Ð¢6g&æö—6UÒ4e$ôc3%ö–æ¦V7Bƒã“°Ð¢ÐÐ¢VÇ6R–b‡6g&FW7DÆö÷ÓÒ4e$õDU5Eõ5TTDÄôõÐ¢°Ð¢6g&æö—6UrÒ4e$ôc3%ö–æ¦V7Bƒã“°Ð¢ÐÐ Ð¢&WGW&ã°Ð§ÐÐ Ð¢òò¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢¢ Ð§fö–B6öÆÆV7E4e$„ÔõDõ%õf'5÷B§Ö÷F÷"Ð§°Ð¢–b‡6g&FW7DÆö÷ÓÒ4e$õDU5EôEô„•2Ð¢°Ð¢4e$ôc3%ö6öÆÆV7B‚gÖ÷F÷"Óç•ö–Bæ÷WBÀÐ¢gÖ÷F÷"Óç•ö–Bæf&²“°Ð¢ÐÐ¢VÇ6R–b‡6g&FW7DÆö÷ÓÒ4e$õDU5Eõô„•2Ð¢°Ð¢4e$ôc3%ö6öÆÆV7B‚gÖ÷F÷"ÓçG$d4ÂÓç•ö—æ÷WBÀÐ¢gÖ÷F÷"ÓçG$d4ÂÓç•ö—æf&²“°Ð¢ÐÐ¢VÇ6R–b‡6g&FW7DÆö÷ÓÒ4e$õDU5Eõ5TTDÄôõÐ¢°Ð¢4e$ôc3%ö6öÆÆV7B‚gÖ÷F÷"Óç–E÷7BçFW&Òä÷WBÀÐ¢gÖ÷F÷"Óç–E÷7BçFW&Òäf&²“°Ð¢ÐÐ Ð¢&WGW&ã°Ð§ÐÐ¢6VæF–`Ð Ð¢òðÐ¢òòVæBöb6öFPÐ¢òðÐ