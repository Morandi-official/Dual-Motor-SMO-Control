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
extern volatile float32_t esmoThetaErrFFPu[2];

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

// Field-weakening current-reference shaping. Called once per ISR in
// buildLevel46_Mx AFTER the normal pi_id.ref / pi_iq.ref assignment; it
// overrides them with the FW split (negative Id / reduced Iq) only when
// FW is enabled AND the motor is in steady closed loop. When disabled or
// outside steady closed loop it leaves the references untouched, so the
// build with FW off is bit-identical to the pre-FW behavior.
extern void runFieldWeakening(struct _MOTOR_Vars_t_ *pMotor,
                              uint16_t motorIdx);

#ifdef __cplusplus
}
#endif

#endif
