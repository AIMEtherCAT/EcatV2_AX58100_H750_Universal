#include "pid.h"
#include <cmath>

void pid_init(pid_t *pid, const float p, const float i, const float d, const float max_out, const float max_iout) {
    if (pid == nullptr) {
        return;
    }

    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
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

float pid_calculate(pid_t *pid, const float ref, const float set) {
    if (std::isnan(pid->SumError)) {
        pid->SumError = 0;
    }

    pid->SetPointLast = pid->SetPoint;
    pid->SetPoint = set;
    pid->PreError = pid->SetPoint - ref;
    if (std::fabsf(pid->PreError) < pid->deadband) {
        return 0;
    }
    pid->dError = pid->PreError - pid->LastError;

    pid->SetPointLast = pid->SetPoint;

    if (pid->PreError > -pid->ErrorMax && pid->PreError < pid->ErrorMax) {
        pid->SumError += (pid->PreError + pid->LastError) / 2;
    }

    pid->LastError = pid->PreError;

    if (pid->SumError >= pid->IMax) {
        pid->SumError = pid->IMax;
    } else if (pid->SumError <= -pid->IMax) {
        pid->SumError = -pid->IMax;
    }

    pid->POut = pid->Kp * pid->PreError;
    pid->IOut = pid->Ki * pid->SumError;
    pid->DOut = pid->Kd * pid->dError;

    pid->out = LIMIT_MAX_MIN(pid->POut + pid->IOut + pid->DOut, pid->OutMax, -pid->OutMax);

    return pid->out;
}

float pid_calculate(pid_t *pid, const float ref) {
    return pid_calculate(pid, ref, pid->SetPoint);
}

void pid_clear(pid_t *pid) {
    if (pid == nullptr) {
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