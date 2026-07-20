//#############################################################################
//
// FILE:    dual_axis_servo_drive.h
//
// TITLE:   Include header files used in the project
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

#ifndef DUAL_AXIS_SERVO_DRIVE_H
#define DUAL_AXIS_SERVO_DRIVE_H

//
//! \file   solutions/common/sensored_foc/include/dual_axis_servo_drive.h
//! \brief  header file to be included in all labs
//!
//


//
//! \defgroup LABS LABS
//! @{
//

//
// includes
//
#include "device.h"
#include "dual_axis_servo_drive_user.h"

#include "clarke.h"
#include "park.h"
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

#include "fcl_enum.h"
#include "fcl_cla_dm.h"
#include "fcl_cpu_cla_dm.h"
#include "cpu_cla_shared_dm.h"
#include "dual_axis_servo_drive_sensorless.h"

#include <math.h>


//
// the function prototypes
//

//! \brief
//! \param[in]  out
//! \return
extern float32_t refPosGen(float32_t out, MOTOR_Vars_t *pMotor);

//! \brief
//! \param[in]  in
//! \param[in]  out
//! \param[in]  rampDelta
//! \return
extern float32_t ramper(float32_t in, float32_t out, float32_t rampDelta);


//! \brief      Get the dc bus voltage
//! \return     The dc bus voltage
static inline float32_t getVdc(MOTOR_Vars_t *ptrMotor)
{
    float32_t vdc;

    vdc = HWREGH(ptrMotor->volDC_PPBRESULT) * ptrMotor->voltageScale;

    if(vdc < 1.0)
    {
        vdc = 1.0;
    }

    return(vdc);
}

//! \brief      Initializes the parameters of motor
//! \details    Initializes all the parameters for each motor
//! \param[in]  pMotor   A pointer to the motorVars object
extern void initMotorParameters(MOTOR_Vars_t *pMotor, HAL_MTR_Handle mtrHandle);

//! \brief      Initializes the control variables of motor
//! \details    Initializes all the control variables for each motor
//! \param[in]  pMotor   A pointer to the motorVars object
extern void initControlVars(MOTOR_Vars_t *pMotor);

//! \brief      Reset the control variables of motor
//! \details    Reset the control variables for each motor
//! \param[in]  pMotor   A pointer to the motorVars object
extern void resetControlVars(MOTOR_Vars_t *pMotor);

//! \brief      Run offser calibration
//! \details    implements offset calculation using filters
//! \param[in]  pMotor   A pointer to the motorVars object
extern void runOffsetsCalculation(MOTOR_Vars_t *pMotor);

//! \brief      Run motor control
//! \details    Set current limitation, check fault
//! \param[in]  pMotor   A pointer to the motorVars object
extern void
runMotorControl(MOTOR_Vars_t *pMotor, HAL_MTR_Handle mtrHandle);

//
// Close the Doxygen group.
//! @} //defgroup
//

#endif // end of DUAL_AXIS_SERVO_DRIVE_H definition
