#ifndef PID_H
#define PID_H

#define LIMIT_MAX_MIN(x, max, min) (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

#include "stm32h7xx_hal.h"

typedef struct {
    double SetPoint;
    double SetPointLast;
    double deadband;

    double Kp;
    double Ki;
    double Kd;

    double LastError;
    double PreError;
    double SumError;
    double dError;

    double ErrorMax;
    double IMax;

    double POut;
    double IOut;
    double DOut;
    double OutMax;

    double out;
} pid_type_def;

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
#ifdef __cplusplus
extern "C" {
#endif

extern void PID_init(pid_type_def *pid, const double PID[3], double max_out, double max_iout);

void PID_init_raw(pid_type_def *pid, double p, double i, double d, double max_out,
        double max_iout);



/**
  * @brief          pid calculate
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data
  * @param[in]      set: set point
  * @retval         pid out
  */
extern double PID_calc(pid_type_def *pid, double ref, double set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

double PID_calc_preset_set_value(pid_type_def *P, double ref);

#ifdef __cplusplus
}
#endif

#endif