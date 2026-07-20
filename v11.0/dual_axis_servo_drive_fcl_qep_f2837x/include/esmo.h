//#############################################################################
//
// FILE:    esmo.h
//
// TITLE:   Enhanced sliding mode observer interface for dual FCL project
//
//#############################################################################

#ifndef ESMO_H
#define ESMO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <math.h>
#include <stddef.h>
#include "device.h"

#ifndef MATH_ONE_OVER_THREE
#define MATH_ONE_OVER_THREE       ((float32_t)(0.3333333333333333))
#endif

#ifndef MATH_ONE_OVER_SQRT_THREE
#define MATH_ONE_OVER_SQRT_THREE  ((float32_t)(0.5773502691896258))
#endif

#ifndef MATH_ONE_OVER_TWO_PI
#define MATH_ONE_OVER_TWO_PI      ((float32_t)(0.1591549430918954))
#endif

#ifndef MATH_PI
#define MATH_PI                   ((float32_t)(3.1415926535897932))
#endif

#ifndef MATH_TWO_PI
#define MATH_TWO_PI               ((float32_t)(6.2831853071795865))
#endif

#ifndef ESMO_MATH_VEC_TYPES
#define ESMO_MATH_VEC_TYPES
typedef struct _MATH_Vec2_
{
    float32_t value[2];
} MATH_Vec2;

typedef MATH_Vec2 MATH_vec2;

typedef struct _MATH_Vec3_
{
    float32_t value[3];
} MATH_Vec3;

typedef MATH_Vec3 MATH_vec3;
#endif

static inline float32_t ESMO_incrAngle(const float32_t angle_rad,
                                       const float32_t angleDelta_rad)
{
    float32_t angleNew_rad = angle_rad + angleDelta_rad;

    if(angleNew_rad > MATH_PI)
    {
        angleNew_rad -= MATH_TWO_PI;
    }
    else if(angleNew_rad < -MATH_PI)
    {
        angleNew_rad += MATH_TWO_PI;
    }

    return(angleNew_rad);
}

typedef struct _ESMO_Params_
{
    float32_t maxFrequency_Hz;
    float32_t ctrlPeriod_sec;
    float32_t voltageBase_V;
    float32_t currentBase_A;
    float32_t motor_Rs_d_Ohm;
    float32_t motor_Rs_q_Ohm;
    float32_t motor_Ls_d_H;
    float32_t motor_Ls_q_H;
} ESMO_Params;

typedef struct _ESMO_Obj_
{
    float32_t scaleFreq_Hz;

    float32_t speed_sf;
    float32_t voltage_sf;
    float32_t current_sf;

    float32_t Ts;
    float32_t base_wTs;
    float32_t filterFc_Hz;

    float32_t Gdsmopos;
    float32_t Gqsmopos;
    float32_t Fdsmopos;
    float32_t Fqsmopos;
    float32_t Kslf;
    float32_t E0;

    float32_t Kslide;
    float32_t KslideMax;
    float32_t KslideMin;

    float32_t Valpha;
    float32_t Vbeta;

    float32_t EstIalpha;
    float32_t EstIbeta;

    float32_t Ealpha;
    float32_t Ebeta;

    float32_t Zalpha;
    float32_t Zbeta;

    float32_t Ed;
    float32_t Eq;
    float32_t Eq_mag;

    float32_t thetaOffset_rad;
    float32_t thetaElec_rad;

    float32_t thetaErr;
    float32_t thetaErrSF;
    float32_t theta;
    float32_t thetaPll;
    float32_t thetaDelta;
    float32_t offsetSF;
    float32_t thetaEst;

    float32_t speedRef;
    float32_t speedEst;
    float32_t speedFlt;

    float32_t pll_Out;
    float32_t pll_Umax;
    float32_t pll_Umin;
    float32_t pll_ui;

    float32_t pll_Kp;
    float32_t pll_KpMax;
    float32_t pll_KpMin;
    float32_t pll_KpSF;
    float32_t pll_Ki;

    float32_t lpf_b0;
    float32_t lpf_a1;

    float32_t lpfFc_Hz;
} ESMO_Obj;

typedef struct _ESMO_Obj_ *ESMO_Handle;

extern ESMO_Handle ESMO_init(void *pMemory, const size_t numBytes);
extern void ESMO_resetParams(ESMO_Handle handle);
extern void ESMO_setParams(ESMO_Handle handle, const ESMO_Params *pParams);
extern void ESMO_setPLLParams(ESMO_Handle handle,
                              const float32_t pll_KpMax,
                              const float32_t pll_KpMin,
                              const float32_t pll_KpSF);
extern void ESMO_setKslideParams(ESMO_Handle handle,
                                 const float32_t KslideMax,
                                 const float32_t KslideMin);
extern void ESMO_updateFilterParams(ESMO_Handle handle);
extern void ESMO_updatePLLParams(ESMO_Handle handle);
extern void ESMO_run(ESMO_Handle handle,
                     float32_t Vdcbus,
                     MATH_vec3 *pVabc_pu,
                     MATH_vec2 *pIabVec);
extern void ESMO_full_run(ESMO_Handle handle,
                          float32_t Vdcbus,
                          MATH_vec3 *pVabc_pu,
                          MATH_vec2 *pIabVec);

static inline void ESMO_setBEMFThreshold(ESMO_Handle handle,
                                         const float32_t bemfThreshold)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->E0 = bemfThreshold;

    return;
}

static inline void ESMO_setPLLKpSF(ESMO_Handle handle,
                                   const float32_t pll_KpSF)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->pll_KpSF = pll_KpSF;

    return;
}

static inline void ESMO_setKslide(ESMO_Handle handle, const float32_t Kslide)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->Kslide = Kslide;

    return;
}

static inline void ESMO_resetPLL(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->pll_ui = 0.0f;
    obj->pll_Out = 0.0f;

    return;
}

static inline void ESMO_setOffsetCoef(ESMO_Handle handle,
                                      const float32_t offsetSF)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->offsetSF = offsetSF;

    return;
}

static inline void ESMO_setBEMFKslfFreq(ESMO_Handle handle,
                                        const float32_t filterFc_Hz)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->filterFc_Hz = filterFc_Hz;

    return;
}

static inline void ESMO_setSpeedFilterFreq(ESMO_Handle handle,
                                           const float32_t lpfFc_Hz)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->lpfFc_Hz = lpfFc_Hz;

    return;
}

static inline void ESMO_setPLLKp(ESMO_Handle handle, const float32_t pll_Kp)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->pll_Kp = pll_Kp;

    return;
}

// Must be called AFTER ESMO_setParams(), which initializes pll_Ki to zero.
static inline void ESMO_setPLLKi(ESMO_Handle handle, const float32_t pll_Ki)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->pll_Ki = pll_Ki;

    return;
}

static inline void ESMO_setAnglePu(ESMO_Handle handle,
                                   const float32_t theta_rad)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->theta = theta_rad * MATH_ONE_OVER_TWO_PI;

    return;
}

static inline void ESMO_setAngleElecPu(ESMO_Handle handle,
                                       const float32_t theta_pu)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->theta = theta_pu;
    obj->thetaEst = theta_pu * MATH_TWO_PI;

    if(obj->thetaEst > MATH_PI)
    {
        obj->thetaEst -= MATH_TWO_PI;
    }
    else if(obj->thetaEst < -MATH_PI)
    {
        obj->thetaEst += MATH_TWO_PI;
    }

    obj->thetaPll = theta_pu;
    obj->thetaElec_rad = obj->thetaEst;
    obj->thetaOffset_rad = 0.0f;
    obj->thetaErr = 0.0f;

    return;
}

static inline void ESMO_setSpeedRef(ESMO_Handle handle,
                                    const float32_t speedRef_Hz)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->speedRef = speedRef_Hz * obj->speed_sf;

    return;
}

static inline float32_t ESMO_getSpeed_Hz(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    return(obj->speedEst * obj->scaleFreq_Hz);
}

static inline float32_t ESMO_getSpeedPLL_Hz(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    return(obj->speedFlt * obj->scaleFreq_Hz);
}

static inline float32_t ESMO_getAnglePLL(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    return(obj->thetaEst);
}

static inline float32_t ESMO_getAngleElec(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    return(obj->thetaElec_rad);
}

static inline void ESMO_updateKslide(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    if(obj->Kslide < obj->KslideMax)
    {
        obj->Kslide += 0.000002f;
    }

    return;
}

#ifdef __cplusplus
}
#endif

#endif
