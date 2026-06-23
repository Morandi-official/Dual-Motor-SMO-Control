//#############################################################################
//
// FILE:    dual_axis_servo_drive_user.h
//
// TITLE:   motor parameters definition
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

#ifndef DUAL_AXIS_SERVO_DRIVE_USER_H
#define DUAL_AXIS_SERVO_DRIVE_USER_H

//
// CLA-safe feedback selection macros. Keep these free of driverlib/std headers
// because cpu_cla_shared_dm.h includes this file from CLA code.
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

#define M1_POSITION_FEEDBACK            POSITION_FEEDBACK_ESMO
#define M2_POSITION_FEEDBACK            POSITION_FEEDBACK_ESMO

//
// Include project specific include files.
//


//
// PWM, SAMPLING FREQUENCY and Current Loop Band width definitions for motor 1
// motor 2, can be set separately
//
#define DM_PWM_FREQUENCY        10   // in KHz


//
// Analog scaling with ADC
//
#define ADC_PU_SCALE_FACTOR         0.000244140625     // 1/2^12, 12bits ADC
#define ADC_PU_PPB_SCALE_FACTOR     0.000488281250     // 1/2^11, 12bits ADC

//
// ADC and PWM Related defines for M1
//
#define M1_IU_ADC_BASE         ADCC_BASE           //C2, NC: Set up based board
#define M1_IV_ADC_BASE         ADCB_BASE           //B2, NC: Set up based board
#define M1_IW_ADC_BASE         ADCA_BASE           //A2, NC: Set up based board
#define M1_VDC_ADC_BASE        ADCD_BASE           //D14, NC: Set up based board

#define M1_IU_ADCRESULT_BASE   ADCCRESULT_BASE     //C2, NC: Set up based board
#define M1_IV_ADCRESULT_BASE   ADCBRESULT_BASE     //B2, NC: Set up based board
#define M1_IW_ADCRESULT_BASE   ADCARESULT_BASE     //A2, NC: Set up based board
#define M1_VDC_ADCRESULT_BASE  ADCDRESULT_BASE     //D14, NC: Set up based board

#define M1_IU_ADC_CH_NUM       ADC_CH_ADCIN2       //C2, NC: Set up based board
#define M1_IV_ADC_CH_NUM       ADC_CH_ADCIN2       //B2, NC: Set up based board
#define M1_IW_ADC_CH_NUM       ADC_CH_ADCIN2       //A2, NC: Set up based board
#define M1_VDC_ADC_CH_NUM      ADC_CH_ADCIN14      //D14, NC: Set up based board

#define M1_IU_ADC_SOC_NUM      ADC_SOC_NUMBER0     //C2, NC: Set up based board
#define M1_IV_ADC_SOC_NUM      ADC_SOC_NUMBER0     //B2, NC: Set up based board
#define M1_IW_ADC_SOC_NUM      ADC_SOC_NUMBER0     //A2, NC: Set up based board
#define M1_VDC_ADC_SOC_NUM     ADC_SOC_NUMBER0     //D14, NC: Set up based board

#define M1_IU_ADC_PPB_NUM      ADC_PPB_NUMBER1     //C2, NC: Set up based board
#define M1_IV_ADC_PPB_NUM      ADC_PPB_NUMBER1     //B2, NC: Set up based board
#define M1_IW_ADC_PPB_NUM      ADC_PPB_NUMBER1     //A2, NC: Set up based board
#define M1_VDC_ADC_PPB_NUM     ADC_PPB_NUMBER1     //D14, NC: Set up based board

#define M1_U_CMPSS_BASE        CMPSS6_BASE         // NC: Set up based board
#define M1_V_CMPSS_BASE        CMPSS3_BASE         // NC: Set up based board
#define M1_W_CMPSS_BASE        CMPSS1_BASE         // NC: Set up based board

#define M1_ADC_TRIGGER_SOC     ADC_TRIGGER_EPWM1_SOCA  // NC: Set up based board

#define M1_U_PWM_BASE          EPWM1_BASE          // NC: Set up based board
#define M1_V_PWM_BASE          EPWM2_BASE          // NC: Set up based board
#define M1_W_PWM_BASE          EPWM3_BASE          // NC: Set up based board

#define M1_INT_PWM             INT_EPWM1           // NC: Set up based board

#define M1_QEP_BASE            EQEP1_BASE          // NC: Set up based board

#define M1_SPI_BASE            SPIA_BASE           // NC: Set up based board

#define M1_IFB_U      ADC_readResult(M1_IU_ADCRESULT_BASE, M1_IU_ADC_SOC_NUM)
#define M1_IFB_V      ADC_readResult(M1_IV_ADCRESULT_BASE, M1_IV_ADC_SOC_NUM)
#define M1_IFB_W      ADC_readResult(M1_IW_ADCRESULT_BASE, M1_IW_ADC_SOC_NUM)

#define M1_VDC      ADC_readResult(M1_VDC_ADCRESULT_BASE, M1_VDC_ADC_SOC_NUM)

#define M1_IFB_U_PPB  ADC_readPPBResult(M1_IU_ADCRESULT_BASE, M1_IU_ADC_PPB_NUM)
#define M1_IFB_V_PPB  ADC_readPPBResult(M1_IV_ADCRESULT_BASE, M1_IV_ADC_PPB_NUM)
#define M1_IFB_W_PPB  ADC_readPPBResult(M1_IW_ADCRESULT_BASE, M1_IW_ADC_PPB_NUM)

#define M1_VDC_PPB  ADC_readPPBResult(M1_VDC_ADCRESULT_BASE, M1_VDC_ADC_PPB_NUM)

//
// ADC and PWM Related defines for M2
//
#define M2_IU_ADC_BASE         ADCC_BASE           //C4, NC: Set up based board
#define M2_IV_ADC_BASE         ADCB_BASE           //B4, NC: Set up based board
#define M2_IW_ADC_BASE         ADCA_BASE           //A4, NC: Set up based board
#define M2_VDC_ADC_BASE        ADCD_BASE           //D15, NC: Set up based board

#define M2_IU_ADCRESULT_BASE   ADCCRESULT_BASE     //C4, NC: Set up based board
#define M2_IV_ADCRESULT_BASE   ADCBRESULT_BASE     //B4, NC: Set up based board
#define M2_IW_ADCRESULT_BASE   ADCARESULT_BASE     //A4, NC: Set up based board
#define M2_VDC_ADCRESULT_BASE  ADCDRESULT_BASE     //D15, NC: Set up based board

#define M2_IU_ADC_CH_NUM       ADC_CH_ADCIN4       //C4, NC: Set up based board
#define M2_IV_ADC_CH_NUM       ADC_CH_ADCIN4       //B4, NC: Set up based board
#define M2_IW_ADC_CH_NUM       ADC_CH_ADCIN4       //A4, NC: Set up based board
#define M2_VDC_ADC_CH_NUM      ADC_CH_ADCIN15      //D15, NC: Set up based board

#define M2_IU_ADC_SOC_NUM      ADC_SOC_NUMBER1     //C4, NC: Set up based board
#define M2_IV_ADC_SOC_NUM      ADC_SOC_NUMBER1     //B4, NC: Set up based board
#define M2_IW_ADC_SOC_NUM      ADC_SOC_NUMBER1     //A4, NC: Set up based board
#define M2_VDC_ADC_SOC_NUM     ADC_SOC_NUMBER1     //D15, NC: Set up based board

#define M2_IU_ADC_PPB_NUM      ADC_PPB_NUMBER2     // NC: Set up based board
#define M2_IV_ADC_PPB_NUM      ADC_PPB_NUMBER2     // NC: Set up based board
#define M2_IW_ADC_PPB_NUM      ADC_PPB_NUMBER2     // NC: Set up based board
#define M2_VDC_ADC_PPB_NUM     ADC_PPB_NUMBER2     // NC: Set up based board

#define M2_U_CMPSS_BASE        CMPSS5_BASE         // NC: Set up based board
#define M2_V_CMPSS_BASE        CMPSS5_BASE         // NC: Set up based board
#define M2_W_CMPSS_BASE        CMPSS2_BASE         // NC: Set up based board

#define M2_ADC_TRIGGER_SOC     ADC_TRIGGER_EPWM4_SOCA  // NC: Set up based board

#define M2_U_PWM_BASE          EPWM4_BASE          // NC: Set up based board
#define M2_V_PWM_BASE          EPWM5_BASE          // NC: Set up based board
#define M2_W_PWM_BASE          EPWM6_BASE          // NC: Set up based board

#define M2_INT_PWM             INT_EPWM4           // NC: Set up based board

#define M2_QEP_BASE            EQEP2_BASE          // NC: Set up based board

#define M2_SPI_BASE            SPIB_BASE           // NC: Set up based board

#define M2_IFB_U      ADC_readResult(M2_IU_ADCRESULT_BASE, M2_IU_ADC_SOC_NUM)
#define M2_IFB_V      ADC_readResult(M2_IV_ADCRESULT_BASE, M2_IV_ADC_SOC_NUM)
#define M2_IFB_W      ADC_readResult(M2_IW_ADCRESULT_BASE, M2_IW_ADC_SOC_NUM)

#define M2_VDC      ADC_readResult(M2_VDC_ADCRESULT_BASE, M2_VDC_ADC_SOC_NUM)

#define M2_IFB_U_PPB  ADC_readPPBResult(M2_IU_ADCRESULT_BASE, M2_IU_ADC_PPB_NUM)
#define M2_IFB_V_PPB  ADC_readPPBResult(M2_IV_ADCRESULT_BASE, M2_IV_ADC_PPB_NUM)
#define M2_IFB_W_PPB  ADC_readPPBResult(M2_IW_ADCRESULT_BASE, M2_IW_ADC_PPB_NUM)

#define M2_VDC_PPB  ADC_readPPBResult(M2_VDC_ADCRESULT_BASE, M2_VDC_ADC_PPB_NUM)

//
// Motor_1 Parameters
//

//
// PWM, SAMPLING FREQUENCY and Current Loop Band width definitions
//
#define M1_PWM_FREQUENCY           10   // in KHz

#if(SAMPLING_METHOD == SINGLE_SAMPLING)
#define M1_ISR_FREQUENCY           (M1_PWM_FREQUENCY)

#elif(SAMPLING_METHOD == DOUBLE_SAMPLING)
#define M1_ISR_FREQUENCY           (2 * M1_PWM_FREQUENCY)

#endif

//
// Keep PWM Period same between single sampling and double sampling
//
#define M1_INV_PWM_TICKS        (((SYSTEM_FREQUENCY/2.0)/M1_PWM_FREQUENCY)*1000)
#define M1_INV_PWM_DB            (200.0)
#define M1_QEP_UNIT_TIMER_TICKS  (SYSTEM_FREQUENCY/(2*M1_PWM_FREQUENCY) * 1000)

#define M1_INV_PWM_TBPRD         (M1_INV_PWM_TICKS / 2)
#define M1_INV_PWM_HALF_TBPRD    (M1_INV_PWM_TBPRD / 2)
#define M1_SAMPLING_FREQ         (M1_ISR_FREQUENCY * 1000)
#define M1_CUR_LOOP_BANDWIDTH    (2.0F * PI * M1_SAMPLING_FREQ / 18)

#define M1_TPWM_CARRIER          (1000.0 / (M1_PWM_FREQUENCY))    //in uSec

//
// FCL Computation time predetermined from library
// tests on F2837xD
//
#define M1_FCL_COMPUTATION_TIME  (1.00)  //in uS

//
// set the motor parameters to the one available
//
#define M1_ENCODER_LINES         1000 // Encoder lines for Tekic

//
// Define the electrical motor parameters
//
#define M1_RS_LINE 0.59            // Line-line resistance (ohm), from motor parameter txt
#define M1_RS      (M1_RS_LINE * 0.5) // Phase resistance for Y winding, used by FCL/eSMO
#define M1_RR      NULL            // Rotor resistance (ohm)
#define M1_LS_LINE 0.00066         // Line inductance (H), from motor parameter txt
#define M1_LS      0.00102675      // Average dq stator inductance (H)
#define M1_LD      0.0010233       // Stator d-axis inductance (H)
#define M1_LQ      0.0010302       // Stator q-axis inductance (H)
#define M1_LR      NULL            // Rotor inductance (H)
#define M1_LM      NULL            // Magnetizing inductance (H)
#define M1_KB      ((M1_BASE_FLUX * 6.283185307179586F * M1_BASE_FREQ) / M1_BASE_VOLTAGE)
                                  // FCL pu BEMF gain, flux*wbase/base phase voltage
#define M1_POLES   8               // Number of poles

//
// Notes:
// - FCL/eSMO use phase resistance, so the line-line resistance in the motor
//   parameter txt is divided by two for the Y-connected winding.
// - BASE_VOLTAGE is the controller voltage base. Keep 13.86V when the inverter
//   DC bus is 24V. If the motor "24V" rating is line-line RMS instead, use
//   19.60V (= 24*sqrt(2/3)) and retune eSMO/FCL voltage related gains.
// - MAXIMUM_SCALE_CURRENT/VOLATGE below are board ADC scaling values, not motor
//   nameplate values.
//

//
// Define the base quantites
//
#define M1_BASE_VOLTAGE     13.86 // Base phase peak voltage (V), assumes 24Vdc SVPWM: Vdc/sqrt(3)
#define M1_BASE_CURRENT     13.5  // Base peak phase current (amp),
                                  // the maximum measurable peak current
#define M1_BASE_TORQUE      NULL  // Base torque (N.m)
#define M1_BASE_FLUX        0.0114 // Base flux linkage (volt.sec/rad)
#define M1_BASE_FREQ        266.6667 // Base electrical frequency (Hz)
#define M1_MAXIMUM_CURRENT  13.5  // Motor peak current (amp)

#define M1_ESMO_KSLIDE_MAX        0.55
#define M1_ESMO_KSLIDE_MIN        0.10
#define M1_ESMO_PLL_KP_MAX        5.25
#define M1_ESMO_PLL_KP_MIN        1.50
#define M1_ESMO_PLL_KP_SF         16.0
#define M1_ESMO_BEMF_THRESHOLD    0.5
#define M1_ESMO_BEMF_KSLF_FC_HZ   1.0
#define M1_ESMO_THETA_OFFSET_SF   1.0
// NOTE: this offset also positions the handoff seed frame. Re-trim it in
// small steps only (<= ~0.03 pu per build); a 0.098 pu step was proven on
// hardware to throw the handoff into the anti-phase lock (reversal/OCP).
#define M1_ESMO_ANGLE_OFFSET_PU   0.40
#define M1_ESMO_SPEED_LPF_FC_HZ   60.0
#define M1_ESMO_FORCE_SPEED       0.07
#define M1_ESMO_FORCE_RUN_SEC     1.0
#define M1_ESMO_TAKEOVER_SEC      2.00
#define M1_ESMO_TAKEOVER_MIN_SETPOINT 0.065
#define M1_ESMO_TAKEOVER_MIN_SPEED    0.03
#define M1_ESMO_TAKEOVER_MIN_BEMF 0.055
#define M1_STARTUP_ID_REF         0.15
#define M1_STARTUP_IQ_REF         0.07
#define M1_STARTUP_IQ_MIN_SCALE   0.20
#define M1_STARTUP_OVERSPEED_BAND 0.03
#define M1_ESMO_TAKEOVER_IQ_LIMIT M1_STARTUP_IQ_REF

// Current-loop voltage output limits, normalized to maxModIndex. High-speed
// operation with Id ~= 0 still needs negative d-axis voltage to cancel motor
// cross-coupling. The original 0.5 d-axis limit became the first bottleneck
// before the actual SVGEN modulation limit was reached.
#define M1_PI_ID_UMAX_SF          0.75
#define M1_PI_IQ_UMAX_SF          0.80

// Field-weakening control (FWC). A voltage-magnitude PI: when the stator
// voltage Vs = sqrt(vd^2+vq^2) exceeds VsRef (a fraction of the modulation
// limit), it tilts the current vector to inject NEGATIVE id, reducing the
// back-emf so the drive can run past base speed (~0.55-0.6pu here). Uses the
// SDK fwc library. DISABLED by default - enable per motor via flagEnableFWC
// in CCS after the bias work, and bring the gains up gradually on the bench.
// Vs/VsRef and the id/iq split all run in the project voltage pu (same units
// as pi_id.out / pi_iq.out, whose limit^2 logs as vsLimitSq). Gains are
// starting guesses for this pu scaling and the 10kHz ISR rate - TUNE on HW.
#define M1_FWC_KP                 3.0
#define M1_FWC_KI                 0.003
#define M1_FWC_VSREF_SF           0.95     // VsRef = SF * voltage limit
#define M1_FWC_MAX_ANGLE_DEG      -40.0    // max current-vector FW tilt
#define M1_FWC_ID_MAX_NEG_PU      0.15     // hard clamp on |negative id| (pu)

//
// Current sensors scaling
// 1.0pu current ==> 9.95A -> 2048 counts ==> 8A -> 1647
//
#define M1_CURRENT_SCALE(A)            (2048 * A / M1_BASE_CURRENT)

//
// Analog scaling with ADC
//
#define M1_ADC_PU_SCALE_FACTOR          0.000244140625     // 1/2^12
#define M1_ADC_PU_PPB_SCALE_FACTOR      0.000488281250     // 1/2^11

//
// Current Scale
//
#define M1_MAXIMUM_SCALE_CURRENT        27.0
#define M1_CURRENT_SENSE_SCALE          (M1_MAXIMUM_SCALE_CURRENT / 4096.0)

//
// Voltage Scale
//
#define M1_MAXIMUM_SCALE_VOLATGE        74.1
#define M1_VOLTAGE_SENSE_SCALE          (M1_MAXIMUM_SCALE_VOLATGE / 4096.0)

//
// Motor_2 Parameters
//

//
// PWM, SAMPLING FREQUENCY and Current Loop Band width definitions
//
#define M2_PWM_FREQUENCY           10   // in KHz

#if(SAMPLING_METHOD == SINGLE_SAMPLING)
#define M2_ISR_FREQUENCY           (M2_PWM_FREQUENCY)

#elif(SAMPLING_METHOD == DOUBLE_SAMPLING)
#define M2_ISR_FREQUENCY           (2 * M2_PWM_FREQUENCY)

#endif

//
// Keep PWM Period same between single sampling and double sampling
//
#define M2_INV_PWM_TICKS        (((SYSTEM_FREQUENCY/2.0)/M2_PWM_FREQUENCY)*1000)
#define M2_INV_PWM_DB            (200.0)
#define M2_QEP_UNIT_TIMER_TICKS  (SYSTEM_FREQUENCY/(2*M2_PWM_FREQUENCY) * 1000)

#define M2_INV_PWM_TBPRD         (M2_INV_PWM_TICKS / 2)
#define M2_INV_PWM_HALF_TBPRD    (M2_INV_PWM_TBPRD / 2)
#define M2_SAMPLING_FREQ         (M2_ISR_FREQUENCY * 1000)
#define M2_CUR_LOOP_BANDWIDTH    (2.0F * PI * M2_SAMPLING_FREQ / 18)

#define M2_TPWM_CARRIER          (1000.0 / M2_PWM_FREQUENCY)    //in uSec

//
// FCL Computation time predetermined from library
// tests on F2837xD
//
#define M2_FCL_COMPUTATION_TIME  (1.00)  //in uS

//
// set the motor parameters to the one available
//
#define M2_ENCODER_LINES          1000        // Encoder lines

//
// Define the electrical motor parameters
//
#define M2_RS_LINE  0.59            // Line-line resistance (ohm), from motor parameter txt
#define M2_RS       (M2_RS_LINE * 0.5) // Phase resistance for Y winding, used by FCL/eSMO
#define M2_RR       NULL            // Rotor resistance (ohm)
#define M2_LS_LINE  0.00066         // Line inductance (H), from motor parameter txt
#define M2_LS       0.00102675      // Average dq stator inductance (H)
#define M2_LD       0.0010233       // Stator d-axis inductance (H)
#define M2_LQ       0.0010302       // Stator q-axis inductance (H)
#define M2_LR       NULL            // Rotor inductance (H)
#define M2_LM       NULL            // Magnetizing inductance (H)
#define M2_KB       ((M2_BASE_FLUX * 6.283185307179586F * M2_BASE_FREQ) / M2_BASE_VOLTAGE)
                                   // FCL pu BEMF gain, flux*wbase/base phase voltage
#define M2_POLES    8               // Number of poles

//
// Define the base quantites
//
#define M2_BASE_VOLTAGE     13.86 // Base phase peak voltage (V), assumes 24Vdc SVPWM: Vdc/sqrt(3)
#define M2_BASE_CURRENT     13.5  // Base peak phase current (amp),
                                  // the maximum measurable peak current
#define M2_BASE_TORQUE      NULL  // Base torque (N.m)
#define M2_BASE_FLUX        0.0114 // Base flux linkage (volt.sec/rad)
#define M2_BASE_FREQ        266.6667 // Base electrical frequency (Hz)
#define M2_MAXIMUM_CURRENT  13.5  // Motor peak current (amp)

#define M2_ESMO_KSLIDE_MAX        0.55
#define M2_ESMO_KSLIDE_MIN        0.10
#define M2_ESMO_PLL_KP_MAX        5.25
#define M2_ESMO_PLL_KP_MIN        1.50
#define M2_ESMO_PLL_KP_SF         16.0
#define M2_ESMO_BEMF_THRESHOLD    0.5
#define M2_ESMO_BEMF_KSLF_FC_HZ   1.0
#define M2_ESMO_THETA_OFFSET_SF   1.0
// See M1_ESMO_ANGLE_OFFSET_PU note.
#define M2_ESMO_ANGLE_OFFSET_PU   0.40
#define M2_ESMO_SPEED_LPF_FC_HZ   60.0
#define M2_ESMO_FORCE_SPEED       0.07
#define M2_ESMO_FORCE_RUN_SEC     1.0
#define M2_ESMO_TAKEOVER_SEC      2.00
#define M2_ESMO_TAKEOVER_MIN_SETPOINT 0.065
#define M2_ESMO_TAKEOVER_MIN_SPEED    0.03
#define M2_ESMO_TAKEOVER_MIN_BEMF 0.055
#define M2_STARTUP_ID_REF         0.15
#define M2_STARTUP_IQ_REF         0.07
#define M2_STARTUP_IQ_MIN_SCALE   0.20
#define M2_STARTUP_OVERSPEED_BAND 0.06
#define M2_ESMO_TAKEOVER_IQ_LIMIT M2_STARTUP_IQ_REF

#define M2_PI_ID_UMAX_SF          0.75
#define M2_PI_IQ_UMAX_SF          0.80

// See M1_FWC_* notes. FWC disabled by default.
#define M2_FWC_KP                 3.0
#define M2_FWC_KI                 0.003
#define M2_FWC_VSREF_SF           0.95
#define M2_FWC_MAX_ANGLE_DEG      -40.0
#define M2_FWC_ID_MAX_NEG_PU      0.15

//
// Current sensors scaling
// 1.0pu current ==> 9.95A -> 2048 counts ==> 8A -> 1647
//
#define M2_CURRENT_SCALE(A)             (2048 * A / M2_BASE_CURRENT)

//
// Analog scaling with ADC
//
#define M2_ADC_PU_SCALE_FACTOR          0.000244140625     // 1/2^12
#define M2_ADC_PU_PPB_SCALE_FACTOR      0.000488281250     // 1/2^11

//
// Current Scale
//
#define M2_MAXIMUM_SCALE_CURRENT        27.0
#define M2_CURRENT_SENSE_SCALE          (M2_MAXIMUM_SCALE_CURRENT / 4096.0)

//
// Voltage Scale
//
#define M2_MAXIMUM_SCALE_VOLATGE        74.1
#define M2_VOLTAGE_SENSE_SCALE          (M2_MAXIMUM_SCALE_VOLATGE / 4096.0)

#endif  // end of DUAL_AXIS_SERVO_DRIVE_USER_H definition
