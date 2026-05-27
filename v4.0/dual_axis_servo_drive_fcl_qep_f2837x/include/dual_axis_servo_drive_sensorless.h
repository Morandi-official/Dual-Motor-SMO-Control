//#############################################################################
//
// FILE:    dual_axis_servo_drive_sensorless.h
//
// TITLE:   eSMO adapter for dual-axis FCL project
//
//#############################################################################

#ifndef DUAL_AXIS_SERVO_DRIVE_SENSORLESS_H
#define DUAL_AXIS_SERVO_DRIVE_SENSORLESS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "esmo.h"

struct _MOTOR_Vars_t_;

extern ESMO_Obj esmoVars[2];

extern void initSensorlessEstimator(struct _MOTOR_Vars_t_ *pMotor);
extern void resetSensorlessEstimator(struct _MOTOR_Vars_t_ *pMotor);
extern void runSensorlessEstimator(struct _MOTOR_Vars_t_ *pMotor);
extern uint16_t isSensorlessActive(const struct _MOTOR_Vars_t_ *pMotor);
extern uint16_t isSensorlessControl(const struct _MOTOR_Vars_t_ *pMotor);
extern uint16_t isSensorlessTakeoverActive(const struct _MOTOR_Vars_t_ *pMotor);
extern uint16_t isSensorlessReadyForClosedLoop(const struct _MOTOR_Vars_t_ *pMotor);
extern void forceSensorlessAngle(struct _MOTOR_Vars_t_ *pMotor, float32_t anglePu);
extern void blendSensorlessAngle(struct _MOTOR_Vars_t_ *pMotor,
                                 float32_t openLoopAnglePu);
extern void syncSensorlessEstimatorAngle(struct _MOTOR_Vars_t_ *pMotor,
                                         float32_t anglePu);

#ifdef __cplusplus
}
#endif

#endif
