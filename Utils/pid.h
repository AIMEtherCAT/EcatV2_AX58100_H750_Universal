#ifndef PID_H
#define PID_H

#define LIMIT_MAX_MIN(x, max, min) (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

typedef struct {
    float SetPoint;
    float SetPointLast;
    float deadband;

    float Kp;
    float Ki;
    float Kd;

    float LastError;
    float PreError;
    float SumError;
    float dError;

    float ErrorMax;
    float IMax;

    float POut;
    float IOut;
    float DOut;
    float OutMax;

    float out;
} pid_t;

void pid_init(pid_t *pid, float p, float i, float d, float max_out, float max_iout);

float pid_calculate(pid_t *pid, float ref, float set);

float pid_calculate(pid_t *pid, float ref);

void pid_clear(pid_t *pid);

#endif