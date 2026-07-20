//#############################################################################
//
// FILE:    fcl_enum.h
//
// TITLE:   define enumerations for FCL
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

#ifndef FCL_ENUM_H
#define FCL_ENUM_H

//
//! \brief Enumeration for State Machine typedef for motor QEP calibration
//
typedef enum
{
    QEP_CALIB_LOOPFLUSH = 0,
    QEP_CALIB_EQP1 = 1,
    QEP_CALIB_QEP2 = 2,
    QEP_CALIB_DONE = 3
} QEPCalibSM_e;

//
//! \brief Enumeration for Motor run/ stop command
//
typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_RUN = 1
} MotorRunStop_e;

//
//! \brief Position/speed feedback source selection for staged sensorless bring-up
//
#define POSITION_FEEDBACK_QEP           0U
#define POSITION_FEEDBACK_ESMO_MONITOR  1U
#define POSITION_FEEDBACK_ESMO          2U

//
//! \brief Enumeration for Load motor selection/ reset
//
typedef enum
{
    LOAD_NONE = 0,
    LOAD_MOTOR1 = 1,
    LOAD_MOTOR2 = 2
} LoadMotor_e;

//
//! \brief Enumeration for FCL controller --> PI/ FCL
//
typedef enum
{
    CNTLR_CPI = 0,
    CNTLR_CMPLX = 1
} CurrentCntlr_e;

//
//! \brief Enumeration for SFRA test axis
//
typedef enum
{
    SFRA_TEST_D_AXIS = 0,
    SFRA_TEST_Q_AXIS = 1,
    SFRA_TEST_SPEEDLOOP = 2
} SFRATest_e;

//
//! \brief Enumeration for PWM update mode
//
typedef enum
{
    PWM_UPDATE_IMMEDIATE = 0,
    PWM_UPDATE_SHADOW = 1
} PWMUpdateType_e;

#endif // end of FCL_ENUM_H definition
