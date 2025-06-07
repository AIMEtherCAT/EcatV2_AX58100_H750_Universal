//
// Created by Hang XU on 2024/8/30.
//

#ifndef SENTINEL_GIMBAL_MAIN_LOWPASS_FILTER_H
#define SENTINEL_GIMBAL_MAIN_LOWPASS_FILTER_H

typedef struct {
    float prevInput;
    float prevOutput;
    float alpha;
} ButterworthLowpass;

void initButterworthLowpass(ButterworthLowpass *filter, float cutoffFreq, float samplingRate);

float processButterworthLowpass(ButterworthLowpass *filter, float input);

#endif //SENTINEL_GIMBAL_MAIN_LOWPASS_FILTER_H
