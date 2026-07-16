//#############################################################################
// FILE:   fcl_cpu_cla_dm.h
// TITLE:  Header file to be shared between example and library for CPU data.
//
//  Group:         C2000
// Target Family:  F2837x/F2838x/F28004x
//
//#############################################################################
// $TI Release: MotorControl SDK v2.01.00.00 $
// $Release Date: Mon Nov 11 15:18:07 CST 2019 $
// $Copyright:
// Copyright (C) 2017-2019 Texas Instruments Incorporated
//
//     http://www.ti.com/ ALL RIGHTS RESERVED
// $
//#############################################################################

#ifndef FCL_CPU_CLA_DM_H
#define FCL_CPU_CLA_DM_H

#define   MATH_TYPE      1

#include "IQmathLib.h"

#include "fcl_enum.h"
#include "cpu_cla_shared_dm.h"
#include "dual_axis_servo_drive_settings.h"

#ifndef POSITION_FEEDBACK_QEP
#define POSITION_FEEDBACK_QEP           0U
#endif

#ifndef POSITION_FEEDBACK_ESMO_MONITOR
#define POSITION_FEEDBACK_ESMO_MONITOR  1U
#endif

#ifndef POSITION_FEEDBACK_ESMO
#define POSITION_FEEDBACK_ESMO          2U
#endif

#include "fcl_pi.h"
#include "qep_defs.h"
#include "RAMP_GEN_CLA.h"

#ifndef F2838x_DEVICE
#include "F28x_Project.h"
#else
#include "f28x_project.h"
#endif

#include "driverlib.h"

#include "rmp_cntl.h"           // Include header for the RMPCNTL object
#include "ipark.h"              // Include header for the IPARK object
#include "clarke.h"
#include "park.h"
#include "speed_fr.h"
#include "svgen.h"
#include "pi.h"                 // Include header for the PI  object
#include "pid_grando.h"

//
// typedefs
//

typedef struct _FCL_Parameters_ {
    float32_t   carrierMid;     // Mid point value of carrier count
    float32_t   adcScale;       // ADC conversion scale to pu
    float32_t   cmidsqrt3;      // internal variable

    float32_t   tSamp;          // sampling time
    float32_t   Rd;             // Motor resistance in D axis
    float32_t   Rq;             // Motor resistance in Q axis
    float32_t   Ld;             // Motor inductance in D axis
    float32_t   Lq;             // Motor inductance in Q axis
    float32_t   Vbase;          // Base voltage for the controller
    float32_t   Ibase;          // Base current for the controller
    float32_t   wccD;           // D axis current controller bandwidth
    float32_t   wccQ;           // Q axis current controller bandwidth
    float32_t   Vdcbus;         // DC bus voltage
    float32_t   BemfK;          // Motor Bemf constant
    float32_t   Wbase;          // Controller base frequency (Motor) in rad/sec
} FCL_Parameters_t;

#define FCL_PARS_DEFAULTS {                                                    \
    0, /* carrierMid */                                                        \
    0, /* adcScale */                                                          \
    0, /* cmidsqrt3 */                                                         \
    0, /* tSamp */                                                             \
    0, /* Rd */                                                                \
    0, /* Rq */                                                                \
    0, /* Ld */                                                                \
    0, /* Lq */                                                                \
    0, /* Vbase */                                                             \
    0, /* Ibase */                                                             \
    0, /* wccD */                                                              \
    0, /* wccQ */                                                              \
    0, /* Vdcbus */                                                            \
    0, /* BemfK */                                                             \
    0  /* Wbase */                                                             \
}

//
// Default values for motor variables
// default parameters for motor_1
//
#define MOTOR1_DEFAULTS  {                                                     \
    8.0,                                /* curLimit */                         \
                                                                               \
    0,                                  /* posCntr */                          \
    5000,                               /* posCntrMax */                       \
    0.001,                              /* posSlewRate */                      \
                                                                               \
    M1_BASE_FREQ,                       /* baseFreq */                         \
    M1_POLES,                           /* poles */                            \
                                                                               \
    0.001 / M1_ISR_FREQUENCY,           /* T */                                \
    0,                                  /* maxModIndex */                      \
                                                                               \
    0,                                  /* tempIdRef */                        \
    0.05,                                /* IdRef_start */                     \
    0.0,                                /* IdRef_run */                        \
    0.0,                                /* IdRef */                            \
    0.05,                                /* IqRef */                           \
                                                                               \
   0.1,                                /* speedRef */                          \
   0.0,                                /* positionRef */                       \
   0.02,                               /* lsw1Speed */                         \
                                                                               \
    0,                                  /* offset_currentAs */                 \
    0,                                  /* offset_currentBs */                 \
    0,                                  /* offset_currentCs */                 \
                                                                               \
    0,                                  /* currentAs */                        \
    0,                                  /* currentBs */                        \
    0,                                  /* currentCs */                        \
                                                                               \
    0,                                  /* currentScale */                     \
    0,                                  /* voltageScale */                     \
    0,                                  /* adcScale */                         \
                                                                               \
    0.0,                                /* posElecTheta */                     \
    0.0,                                /* posMechTheta */                     \
                                                                               \
    (uint32_t *)(M1_U_PWM_BASE + EPWM_O_CMPA),  /* *pwmCompA */                \
    (uint32_t *)(M1_V_PWM_BASE + EPWM_O_CMPA),  /* *pwmCompB */                \
    (uint32_t *)(M1_W_PWM_BASE + EPWM_O_CMPA),  /* *pwmCompC */                \
                                                                               \
    (M1_IU_ADCRESULT_BASE + M1_IU_ADC_PPB_NUM + ADC_O_PPB1RESULT),  /* curA_PPBRESULT */ \
    (M1_IV_ADCRESULT_BASE + M1_IV_ADC_PPB_NUM + ADC_O_PPB1RESULT),  /* curB_PPBRESULT */ \
    (M1_IW_ADCRESULT_BASE + M1_IW_ADC_PPB_NUM + ADC_O_PPB1RESULT),  /* curC_PPBRESULT */ \
    (M1_VDC_ADCRESULT_BASE + M1_VDC_ADC_PPB_NUM + ADC_O_PPB1RESULT), /* volDC_PPBRESULT */ \
    (union ADCINTFLG_REG *)(M1_IW_ADC_BASE + ADC_INTFLG_ADCINT1), /* *AdcIntFlag */ \
                                                                               \
    CMPLXPARS_DEFAULTS,                 /* D_cpu */                            \
    RMPCNTL_DEFAULTS,                   /* rc */                               \
                                                                               \
    CLARKE_DEFAULTS,                    /* clarke */                           \
    PARK_DEFAULTS,                      /* park */                             \
    IPARK_DEFAULTS,                     /* ipark */                            \
    SPEED_MEAS_QEP_DEFAULTS,            /* speed */                            \
                                                                               \
    FCL_PI_CONTROLLER_DEFAULTS,         /* pi_id */                            \
    PI_CONTROLLER_DEFAULTS,             /* pi_pos */                           \
    {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS},  /* pid_spd */ \
                                                                               \
    FCL_PARS_DEFAULTS,                  /* FCL_params */                       \
    &fclVars[0],                        /* *ptrFCL */                          \
                                                                               \
    SVGEN_DEFAULTS,                     /* svgen */                            \
                                                                               \
    M1_POSITION_FEEDBACK,               /* positionFeedback */                 \
    0,                                  /* esmoForceRunCntr */                \
    0,                                  /* esmoForceRunCntMax */              \
    0,                                  /* esmoTakeoverCntr */                \
    0,                                  /* esmoTakeoverCntMax */              \
    M1_ESMO_FORCE_SPEED,                /* esmoForceSpeed */                  \
    0.0,                                /* esmoAnglePu */                     \
    0.0,                                /* esmoAngleRad */                    \
    0.0,                                /* esmoRawAnglePu */                  \
    M1_ESMO_ANGLE_OFFSET_PU,            /* esmoAngleOffsetPu */               \
    0.0,                                /* esmoSpeedPu */                     \
    0.0,                                /* esmoSpeedHz */                     \
    0.0,                                /* esmoQepAnglePu */                 \
    0.0,                                /* esmoAngleErrPu */                 \
    0.0,                                /* esmoSpeedErrPu */                 \
    0.0,                                /* esmoAngleDelaySF */                \
    0.0,                                /* esmoPwmScale */                    \
    {0.0, 0.0},                         /* esmoIab_A */                       \
    {0.0, 0.0, 0.0},                    /* esmoVabc_pu */                     \
    (void *)0,                          /* *esmoHandle */                     \
                                                                               \
    0,                                  /* isrTicker */                        \
                                                                               \
    0.0,                                /* fclLatencyInMicroSec */             \
    0,                                  /* fclClrCntr */                       \
    0,                                  /* fclCycleCountMax */                 \
                                                                               \
    0,                                  /* lsw2EntryFlag */                    \
    0,                                  /* offsetDoneFlag */                   \
    0,                                  /* faultFlag */                        \
    0,                                  /* state */                            \
                                                                               \
    0,                                  /* drvEnGPIO */                        \
    0,                                  /* drvFaultGPIO */                     \
                                                                               \
    0,                                  /* tripFlagDMC */                      \
    0,                                  /* clearTripFlagDMC */                 \
                                                                               \
    10,                                 /* speedLoopPrescaler */               \
    1,                                  /* speedLoopCount */                   \
    0,                                  /* alignCntr */                        \
    2000,                               /* alignCnt */                         \
    2,                                  /* posPtrMax */                        \
    0,                                  /* posPtr */                           \
                                                                               \
    M1_CURRENT_SCALE(8.0),              /* curThreshHi */                      \
    M1_CURRENT_SCALE(8.0),              /* curThreshLo */                      \
    MOTOR_STOP                         /* runMotor */                          \
 }

//
// default parameters for motor_2
//
#define MOTOR2_DEFAULTS  {                                                     \
    8.0,                                /* curLimit */                         \
                                                                               \
    0,                                  /* posCntr */                          \
    5000,                               /* posCntrMax */                       \
    0.001,                              /* posSlewRate */                      \
                                                                               \
    M2_BASE_FREQ,                       /* baseFreq */                         \
    M2_POLES,                           /* poles */                            \
                                                                               \
    0.001 / M2_ISR_FREQUENCY,           /* T */                                \
    0,                                  /* maxModIndex */                      \
                                                                               \
    0,                                  /* tempIdRef */                        \
    0.05,                                /* IdRef_start */                     \
    0.0,                                /* IdRef_run */                        \
    0.0,                                /* IdRef */                            \
    0.05,                                /* IqRef */                           \
                                                                               \
   0.1,                                /* speedRef */                          \
   0.0,                                /* positionRef */                       \
   0.02,                               /* lsw1Speed */                         \
                                                                               \
    0,                                  /* offset_currentAs */                 \
    0,                                  /* offset_currentBs */                 \
    0,                                  /* offset_currentCs */                 \
                                                                               \
    0,                                  /* currentAs */                        \
    0,                                  /* currentBs */                        \
    0,                                  /* currentCs */                        \
                                                                               \
    0,                                  /* currentScale */                     \
    0,                                  /* voltageScale */                     \
    0,                                  /* adcScale */                         \
                                                                               \
    0.0,                                /* posElecTheta */                     \
    0.0,                                /* posMechTheta */                     \
                                                                               \
    (uint32_t *)(M2_U_PWM_BASE + EPWM_O_CMPA),  /* *pwmCompA */                \
    (uint32_t *)(M2_V_PWM_BASE + EPWM_O_CMPA),  /* *pwmCompB */                \
    (uint32_t *)(M2_W_PWM_BASE + EPWM_O_CMPA),  /* *pwmCompC */                \
                                                                               \
    (M2_IU_ADCRESULT_BASE + M2_IU_ADC_PPB_NUM + ADC_O_PPB1RESULT),  /* curA_PPBRESULT */ \
    (M2_IV_ADCRESULT_BASE + M2_IV_ADC_PPB_NUM + ADC_O_PPB1RESULT),  /* curB_PPBRESULT */ \
    (M2_IW_ADCRESULT_BASE + M2_IW_ADC_PPB_NUM + ADC_O_PPB1RESULT),  /* curC_PPBRESULT */ \
    (M2_VDC_ADCRESULT_BASE + M2_VDC_ADC_PPB_NUM + ADC_O_PPB1RESULT), /* volDC_PPBRESULT */ \
    (union ADCINTFLG_REG *)(M2_IW_ADC_BASE + ADC_INTFLG_ADCINT1), /* *AdcIntFlag */ \
                                                                               \
    CMPLXPARS_DEFAULTS,                 /* D_cpu */                            \
    RMPCNTL_DEFAULTS,                   /* rc */                               \
                                                                               \
    CLARKE_DEFAULTS,                    /* clarke */                           \
    PARK_DEFAULTS,                      /* park */                             \
    IPARK_DEFAULTS,                     /* ipark */                            \
    SPEED_MEAS_QEP_DEFAULTS,            /* speed */                            \
                                                                               \
    FCL_PI_CONTROLLER_DEFAULTS,         /* pi_id */                            \
    PI_CONTROLLER_DEFAULTS,             /* pi_pos */                           \
    {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS},  /* pid_spd */ \
                                                                               \
    FCL_PARS_DEFAULTS,                  /* FCL_params */                       \
    &fclVars[0],                        /* *ptrFCL */                          \
                                                                               \
    SVGEN_DEFAULTS,                     /* svgen */                            \
                                                                               \
    M2_POSITION_FEEDBACK,               /* positionFeedback */                 \
    0,                                  /* esmoForceRunCntr */                \
    0,                                  /* esmoForceRunCntMax */              \
    0,                                  /* esmoTakeoverCntr */                \
    0,                                  /* esmoTakeoverCntMax */              \
    M2_ESMO_FORCE_SPEED,                /* esmoForceSpeed */                  \
    0.0,                                /* esmoAnglePu */                     \
    0.0,                                /* esmoAngleRad */                    \
    0.0,                                /* esmoRawAnglePu */                  \
    M2_ESMO_ANGLE_OFFSET_PU,            /* esmoAngleOffsetPu */               \
    0.0,                                /* esmoSpeedPu */                     \
    0.0,                                /* esmoSpeedHz */                     \
    0.0,                                /* esmoQepAnglePu */                 \
    0.0,                                /* esmoAngleErrPu */                 \
    0.0,                                /* esmoSpeedErrPu */                 \
    0.0,                                /* esmoAngleDelaySF */                \
    0.0,                                /* esmoPwmScale */                    \
    {0.0, 0.0},                         /* esmoIab_A */                       \
    {0.0, 0.0, 0.0},                    /* esmoVabc_pu */                     \
    (void *)0,                          /* *esmoHandle */                     \
                                                                               \
    0,                                  /* isrTicker */                        \
    0.0,                                /* fclLatencyInMicroSec */             \
    0,                                  /* fclClrCntr */                       \
    0,                                  /* fclCycleCountMax */                 \
                                                                               \
    0,                                  /* lsw2EntryFlag */                    \
    0,                                  /* offsetDoneFlag */                   \
    0,                                  /* faultFlag */                        \
    0,                                  /* state */                            \
                                                                               \
    0,                                  /* drvEnGPIO */                        \
    0,                                  /* drvFaultGPIO */                     \
                                                                               \
    0,                                  /* tripFlagDMC */                      \
    0,                                  /* clearTripFlagDMC */                 \
                                                                               \
    10,                                 /* speedLoopPrescaler */               \
    1,                                  /* speedLoopCount */                   \
    0,                                  /* alignCntr */                        \
    2000,                               /* alignCnt */                         \
    2,                                  /* posPtrMax */                        \
    0,                                  /* posPtr */                           \
                                                                               \
    M2_CURRENT_SCALE(8.0),              /* curThreshHi */                      \
    M2_CURRENT_SCALE(8.0),              /* curThreshLo */                      \
    MOTOR_STOP                          /* runMotor */                         \
 }

//
//!  \brief typedefs for motorVars
//
typedef struct _MOTOR_Vars_t_
{
    float32_t curLimit;

    uint32_t posCntr;
    uint32_t posCntrMax;
    float32_t posSlewRate;

    float32_t baseFreq;
    float32_t poles;

    float32_t Ts;                   // Samping period (sec)
    float32_t maxModIndex;          // Maximum module index

    float32_t tempIdRef;            // tempId reference (pu)
    float32_t IdRef_start;
    float32_t IdRef_run;
    float32_t IdRef;                // Id reference (pu)
    float32_t IqRef;                // Iq reference (pu)

    float32_t speedRef;             // For Closed Loop tests
    float32_t positionRef;          // For Position Loop tests
    float32_t lsw1Speed;            // initial force rotation speed in search
                                    // of QEP index pulse

    float32_t offset_currentAs;     // offset in current A fbk channel
    float32_t offset_currentBs;     // offset in current B fbk channel
    float32_t offset_currentCs;     // offset in current C fbk channel

    float32_t currentAs;            // phase A
    float32_t currentBs;            // phase B
    float32_t currentCs;            // phase C

    float32_t currentScale;         // current scaling cofficient
    float32_t voltageScale;         // voltage scaling cofficient
    float32_t adcScale;             // ADC scale for current and voltage

    float32_t posElecTheta;
    float32_t posMechTheta;

    volatile uint32_t *pwmCompA;
    volatile uint32_t *pwmCompB;
    volatile uint32_t *pwmCompC;

    volatile uint32_t curA_PPBRESULT;
    volatile uint32_t curB_PPBRESULT;
    volatile uint32_t curC_PPBRESULT;
    volatile uint32_t volDC_PPBRESULT;
    volatile union ADCINTFLG_REG *AdcIntFlag;

    cmplxPars_t D_cpu;
    RMPCNTL rc;                     // ramp control

    CLARKE clarke;                  // clarke transform
    PARK park;                      // park transform
    IPARK ipark;                    // inv park transform
    SPEED_MEAS_QEP speed;

    FCL_PIController_t pi_id;
    PI_CONTROLLER pi_pos;
    PID_CONTROLLER  pid_spd;

    FCL_Parameters_t FCL_params;
    FCL_Vars_t  *ptrFCL;

    SVGEN svgen;

    uint16_t positionFeedback;      // POSITION_FEEDBACK_QEP or eSMO option
    uint16_t esmoForceRunCntr;
    uint16_t esmoForceRunCntMax;
    uint16_t esmoTakeoverCntr;
    uint16_t esmoTakeoverCntMax;
    float32_t esmoForceSpeed;
    float32_t esmoAnglePu;
    float32_t esmoAngleRad;
    float32_t esmoRawAnglePu;
    float32_t esmoAngleOffsetPu;
    float32_t esmoSpeedPu;
    float32_t esmoSpeedHz;
    float32_t esmoQepAnglePu;
    float32_t esmoAngleErrPu;
    float32_t esmoSpeedErrPu;
    float32_t esmoAngleDelaySF;
    float32_t esmoPwmScale;
    float32_t esmoIab_A[2];
    float32_t esmoVabc_pu[3];
    void *esmoHandle;

    uint32_t isrTicker;

    float32_t fclLatencyInMicroSec;    // PWM update latency since sampling
    uint16_t  fclClrCntr;
    uint16_t  fclCycleCountMax;

    uint16_t lsw2EntryFlag;
    uint16_t offsetDoneFlag;
    uint16_t faultFlag;
    uint16_t state;

    uint16_t drvEnGPIO;
    uint16_t drvFaultGPIO;

    uint16_t tripFlagDMC;           //PWM trip status
    uint16_t clearTripFlagDMC;

    uint16_t speedLoopPrescaler;    // Speed loop pre scalar
    uint16_t speedLoopCount;        // Speed loop counter
    uint16_t alignCntr;
    uint16_t alignCnt;
    uint16_t posPtrMax;
    uint16_t posPtr;

    uint16_t curThreshHi;
    uint16_t curThreshLo;

    MotorRunStop_e runMotor;
} MOTOR_Vars_t;

extern MOTOR_Vars_t motorVars[2];

//
// function prototypes
//

extern void FCL_initPWM(MOTOR_Vars_t *ptrMotor,
                uint32_t basePhaseU, uint32_t basePhaseV, uint32_t basePhaseW);

extern void FCL_initADC(uint32_t resultBaseA, ADC_PPBNumber baseA_PPB,
                 uint32_t resultBaseB, ADC_PPBNumber baseB_PPB,
                 uint32_t basePhaseW);

extern void FCL_initADC_2I(MOTOR_Vars_t *ptrMotor, uint32_t basePhaseW,
                 uint32_t resultBaseA, ADC_PPBNumber baseA_PPB,
                 uint32_t resultBaseB, ADC_PPBNumber baseB_PPB);

extern void FCL_initADC_3I(MOTOR_Vars_t *ptrMotor, uint32_t basePhaseW,
                 uint32_t resultBaseA, ADC_PPBNumber baseA_PPB,
                 uint32_t resultBaseB, ADC_PPBNumber baseB_PPB,
                 uint32_t resultBaseC, ADC_PPBNumber baseC_PPB);

extern void FCL_initQEP(MOTOR_Vars_t *ptrMotor, const uint32_t baseA);
extern void FCL_resetController(MOTOR_Vars_t *ptrMotor);

extern void FCL_runPICtrl_M1(MOTOR_Vars_t *pMotor);
extern void FCL_runPICtrlWrap_M1(MOTOR_Vars_t *pMotor);
extern void FCL_runComplexCtrl_M1(MOTOR_Vars_t *pMotor);
extern void FCL_runComplexCtrlWrap_M1(MOTOR_Vars_t *pMotor);

extern void FCL_runPICtrl_M2(MOTOR_Vars_t *pMotor);
extern void FCL_runPICtrlWrap_M2(MOTOR_Vars_t *pMotor);
extern void FCL_runComplexCtrl_M2(MOTOR_Vars_t *pMotor);
extern void FCL_runComplexCtrlWrap_M2(MOTOR_Vars_t *pMotor);

#endif  // end of FCL_CPU_CLA_DM_H definition
