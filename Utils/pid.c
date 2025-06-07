#include "pid.h"
#include "main.h"
#include "math.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

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
void PID_init(pid_type_def *pid, const double PID[3], double max_out, double max_iout) {
    if (pid == NULL || PID == NULL) {
        return;
    }
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->OutMax = max_out;
    pid->IMax = max_iout;

    pid->SetPoint = 0.0f;
    pid->SetPointLast = 0.0f;
    pid->deadband = 0.0f;


    pid->LastError = 0.0f;
    pid->PreError = 0.0f;
    pid->SumError = 0.0f;
    pid->dError = 0.0f;

    pid->ErrorMax = 9999999.0f;

    pid->POut = 0.0f;
    pid->IOut = 0.0f;
    pid->DOut = 0.0f;
}

float pid_abs(float value) {
    return value >= 0 ? value : -value;
}

/**
  * @brief          pid calculate
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data
  * @param[in]      set: set point
  * @retval         pid out
  */
double PID_calc(pid_type_def *P, double ref, double set) {
    if (isnan(P->SumError)) {
        P->SumError = 0;
    }

    P->SetPointLast = P->SetPoint;
    P->SetPoint = set;
    P->PreError = P->SetPoint - ref;
    if(pid_abs(P->PreError) < P->deadband){
        return 0;
    }
    P->dError = P->PreError - P->LastError;

    P->SetPointLast = P->SetPoint;

    if(P->PreError > -P->ErrorMax && P->PreError < P->ErrorMax)
    {
        P->SumError += (P->PreError + P->LastError)/2;
    }

    P->LastError = P->PreError;

    if(P->SumError >= P->IMax)
        P->SumError = P->IMax;
    else if(P->SumError <= -P->IMax)
        P->SumError = -P->IMax;

    P->POut = P->Kp * P->PreError;
    P->IOut = P->Ki * P->SumError;
    P->DOut = P->Kd * P->dError;

    P->out = LIMIT_MAX_MIN(P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax);

    return P->out;
}

double PID_calc_preset_set_value(pid_type_def *P, double ref) {
    return PID_calc(P, ref, P->SetPoint);
}


/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
void PID_clear(pid_type_def *pid) {
    if (pid == NULL) {
        return;
    }


    pid->SetPoint = 0.0f;
    pid->SetPointLast = 0.0f;
    pid->deadband = 0.0f;


    pid->LastError = 0.0f;
    pid->PreError = 0.0f;
    pid->SumError = 0.0f;
    pid->dError = 0.0f;

    pid->ErrorMax = 9999999.0f;

    pid->POut = 0.0f;
    pid->IOut = 0.0f;
    pid->DOut = 0.0f;
}