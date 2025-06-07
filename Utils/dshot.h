//
// Created by Hang XU on 20/04/2025.
//

#ifndef DSHOT_H
#define DSHOT_H

#include "tim.h"

#define DSHOT1_TIMER_CLOCK 240000000
#define DSHOT1_TIM             (&htim3)
#define DSHOT1_MOTOR_1_TIM_CHANNEL     TIM_CHANNEL_1
#define DSHOT1_MOTOR_2_TIM_CHANNEL     TIM_CHANNEL_2
#define DSHOT1_MOTOR_3_TIM_CHANNEL     TIM_CHANNEL_3
#define DSHOT1_MOTOR_4_TIM_CHANNEL     TIM_CHANNEL_4

#define DSHOT2_TIMER_CLOCK 240000000
#define DSHOT2_TIM             (&htim2)
#define DSHOT2_MOTOR_1_TIM_CHANNEL     TIM_CHANNEL_1
#define DSHOT2_MOTOR_2_TIM_CHANNEL     TIM_CHANNEL_2
#define DSHOT2_MOTOR_3_TIM_CHANNEL     TIM_CHANNEL_3
#define DSHOT2_MOTOR_4_TIM_CHANNEL     TIM_CHANNEL_4

#define MHZ_TO_HZ(x) 			((x) * 1000000)

#define DSHOT600_HZ     		MHZ_TO_HZ(12)
//#define DSHOT300_HZ     		MHZ_TO_HZ(6)
//#define DSHOT150_HZ     		MHZ_TO_HZ(3)

#define MOTOR_BIT_0            	7
#define MOTOR_BIT_1            	14
#define MOTOR_BITLENGTH        	20

#define DSHOT_FRAME_SIZE       	16
#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */

#define DSHOT_MIN_THROTTLE      48
#define DSHOT_MAX_THROTTLE     	2047
#define DSHOT_RANGE 			(DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

#ifdef __cplusplus
extern "C" {
#endif

void init_dshot1();

void init_dshot2();

void write_dshot1(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4);

void write_dshot2(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4);

#ifdef __cplusplus
}
#endif

#endif //DSHOT_H
