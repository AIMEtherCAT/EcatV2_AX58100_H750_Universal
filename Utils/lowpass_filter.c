//
// Created by Hang XU on 2024/8/30.
//

#include "lowpass_filter.h"

#include <stdio.h>
#include <stdlib.h>

// 初始化滤波器
void initButterworthLowpass(ButterworthLowpass *filter, float cutoffFreq, float samplingRate) {
    filter->prevInput = 0.0;
    filter->prevOutput = 0.0;
    // 计算alpha (通常是根据截止频率和采样率)
    float dt = 1.0 / samplingRate;
    float RC = 1.0 / (2 * 3.14159265358979323846 * cutoffFreq);
    filter->alpha = dt / (RC + dt);
}

// 更新滤波器并返回新的输出值
float processButterworthLowpass(ButterworthLowpass *filter, float input) {
    float output = filter->prevOutput + filter->alpha * (input - filter->prevOutput);
    filter->prevInput = input;
    filter->prevOutput = output;
    return output;
}

float lowpass_filter(float prev, float current, float alpha) {
    return alpha * current + (1.0f - alpha) * prev;
}