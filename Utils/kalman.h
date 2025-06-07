//
// Created by Hang Xu on 18/01/2024.
//

#ifndef INFANTRY_CHASSIS_KALMAN_H
#define INFANTRY_CHASSIS_KALMAN_H
typedef struct __KalmanTypeDef {
    float x;
    float A;
    float H;
    float q;
    float r;
    float p;
    float gain;
} KalmanTypeDef;

#ifdef __cplusplus
extern "C" {
#endif

void KalmanFilter_Init(KalmanTypeDef *);
float KalmanFilter_Update(KalmanTypeDef *, float);

#ifdef __cplusplus
}
#endif
#endif //INFANTRY_CHASSIS_KALMAN_H
