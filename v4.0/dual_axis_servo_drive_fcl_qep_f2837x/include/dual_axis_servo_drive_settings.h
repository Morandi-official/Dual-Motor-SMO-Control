//#############################################################################
//
// FILE:    dual_axis_servo_drive_settings.h
//
// TITLE:   User settings
//
// Group:   C2000
//
// Target Family: F2837x
//
//#############################################################################
// $TI Release: MotorControl SDK v2.01.00.00 $
// $Release Date: Mon Nov 11 15:18:13 CST 2019 $
// $Copyright:
// Copyright (C) 2017-2019 Texas Instruments Incorporated
//
//     http://www.ti.com/ ALL RIGHTS RESERVED
// $
//#############################################################################

#ifndef DUAL_AXIS_SERVO_DRIVE_SETTINGS_H
#define DUAL_AXIS_SERVO_DRIVE_SETTINGS_H

//
// Include project specific include files.
//

//
// define math type as float(1)
//
#define   MATH_TYPE      1

#include "IQmathLib.h"
#include "device.h"

#include "ipark.h"              // Include header for the IPARK object
#include "pi.h"                 // Include header for the PI  object
#include "fcl_pi.h"             // Include header for the FCL_PI object
#include "svgen.h"              // Include header for the SVGENDQ object
#include "rampgen.h"            // Include header for the RAMPGEN object
#include "rmp_cntl.h"           // Include header for the RMPCNTL object
#include "volt_calc.h"          // Include header for the PHASEVOLTAGE object
#include "speed_fr.h"           // Include header for the SPEED_MEAS_QEP object
#include "resolver.h"
#include "pid_grando.h"
#include "pid_reg3.h"

#include <math.h>

#include "dlog_4ch_f.h"


//
// List of control GND configurations - COLD or HOT
//
#define  COLD  1           // control GND is COLD
#define  HOT   2           // control GND is HOT

//
// Following is the list of the Build Level choices.
//
#define  FCL_LEVEL1  1           // Verify SVGEN module and PWM generation
#define  FCL_LEVEL2  2           // Verify ADC, park/clarke, calibrate the
                                 // offset and speed measurement
#define  FCL_LEVEL3  3           // Verify closed current(torque) loop + its PI
#define  FCL_LEVEL4  4           // Verify speed loop and speed PID
#define  FCL_LEVEL5  5           // Verify position loop
#define  FCL_LEVEL6  6           // SFRA integration to verify bandwidth

//
// This line sets the SAMPLING FREQUENCY to one of the available choices
//
#define  SINGLE_SAMPLING    1
#define  DOUBLE_SAMPLING    2

//
// Following is the list of Current sense options
//
#define  SHUNT_CURRENT_SENSE   1
#define  LEM_CURRENT_SENSE     2
#define  SD_CURRENT_SENSE      3

//
// Following is the list of Position Encoder options
// Select Position Feedback Option
//
#define  QEP_POS_ENCODER       1
#define  RESOLVER_POS_ENCODER  2
#define  BISS_POS_ENCODER      3
#define  ENDAT_POS_ENCODER     4
#define  SINCOS_POS_ENCODER    5

//
// Following is the list of runtime electrical angle/speed feedback sources.
// Keep these definitions in the project settings header so they are available
// even if fcl_enum.h is still linked from the SDK installation.
//
#ifndef POSITION_FEEDBACK_QEP
#define POSITION_FEEDBACK_QEP           0U
#endif

#ifndef POSITION_FEEDBACK_ESMO_MONITOR
#define POSITION_FEEDBACK_ESMO_MONITOR  1U
#endif

#ifndef POSITION_FEEDBACK_ESMO
#define POSITION_FEEDBACK_ESMO          2U
#endif

//
// Here below, pick current loop controller option
//
#define  CMPLX_CNTLR     1
#define  PI_CNTLR        2


//
// Following is the list of motor
// Select motor to implement SFRA, DLOG and DAC
//
#define  MOTOR_1        1
#define  MOTOR_2        2
#define  MOTOR_NA       3

//
// User can select choices from available control configurations
//
#define  CGND                COLD
#define  BUILDLEVEL          FCL_LEVEL4
#define  SAMPLING_METHOD     SINGLE_SAMPLING   // DOUBLE_SAMPLING   //
#define  FCL_CNTLR           PI_CNTLR          // CMPLX_CNTLR       //
#define  CURRENT_SENSE       LEM_CURRENT_SENSE
#define  POSITION_ENCODER    QEP_POS_ENCODER

#define  SFRA_MOTOR          MOTOR_1

//
// Position feedback source per motor.
// POSITION_FEEDBACK_QEP:          original QEP control path
// POSITION_FEEDBACK_ESMO_MONITOR: QEP still controls, eSMO runs for comparison
// POSITION_FEEDBACK_ESMO:         eSMO angle/speed close the loops
//
// M1_POSITION_FEEDBACK and M2_POSITION_FEEDBACK are defined in
// dual_axis_servo_drive_user.h so CLA code can see the same selection without
// including this driverlib-heavy settings header.

//
// Generate error if
//
#if(CURRENT_SENSE != LEM_CURRENT_SENSE)
#error  Critical: Only LEM_CURRENT_SENSE is supported in this example
#endif

#if(POSITION_ENCODER != QEP_POS_ENCODER)
#error  Critical: Only QEP_POS_ENCODER is supported in this example
#endif

#ifndef BUILDLEVEL
#error  Critical: BUILDLEVEL must be defined !!
#endif  // BUILDLEVEL

#ifndef PI
#define PI 3.14159265358979
#endif


#endif  // end of DUAL_AXIS_SERVO_DRIVE_SETTINGS_H definition
