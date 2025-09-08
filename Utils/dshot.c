//
// Created by Hang XU on 20/04/2025.
//
#include "dshot.h"
#include "main.h"
#include "pid.h"

DMA_BUFFER uint32_t dshot1_motor1_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
DMA_BUFFER uint32_t dshot1_motor2_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
DMA_BUFFER uint32_t dshot1_motor3_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
DMA_BUFFER uint32_t dshot1_motor4_dmabuffer[DSHOT_DMA_BUFFER_SIZE];

DMA_BUFFER uint32_t dshot2_motor1_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
DMA_BUFFER uint32_t dshot2_motor2_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
DMA_BUFFER uint32_t dshot2_motor3_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
DMA_BUFFER uint32_t dshot2_motor4_dmabuffer[DSHOT_DMA_BUFFER_SIZE];

void dshot_dma_tc_callback(DMA_HandleTypeDef *hdma) {
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

	if (hdma == htim->hdma[TIM_DMA_ID_CC1]) {
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC2]) {
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC3]) {
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC4]) {
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
	}
}

uint16_t dshot_prepare_packet(uint16_t value) {
	uint16_t packet;
	uint8_t dshot_telemetry = 0;

	packet = (value << 1) | (dshot_telemetry);

	// compute checksum
	unsigned csum = 0;
	unsigned csum_data = packet;

	for(int i = 0; i < 3; i++)
	{
		csum ^=  csum_data; // xor data by nibbles
		csum_data >>= 4;
	}

	csum &= 0xf;
	packet = (packet << 4) | csum;

	return packet;
}

void dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value) {
	uint16_t packet;
	// packet = dshot_prepare_packet(LIMIT_MAX_MIN(LIMIT_MAX_MIN(value, 2000, 0) + 48, 2047, 48));
	packet = dshot_prepare_packet(LIMIT_MAX_MIN(value, 2047, 0));

	for(int i = 0; i < 16; i++) {
		motor_dmabuffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
		packet <<= 1;
	}

	motor_dmabuffer[16] = 0;
	motor_dmabuffer[17] = 0;
}

void init_dshot1() {
	uint16_t dshot_prescaler;
	uint32_t timer_clock = DSHOT1_TIMER_CLOCK;
	dshot_prescaler = lrintf((float) timer_clock / DSHOT600_HZ + 0.01f) - 1;
	__HAL_TIM_SET_PRESCALER(DSHOT1_TIM, dshot_prescaler);
	__HAL_TIM_SET_AUTORELOAD(DSHOT1_TIM, MOTOR_BITLENGTH);

	DSHOT1_TIM->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = dshot_dma_tc_callback;
	DSHOT1_TIM->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = dshot_dma_tc_callback;
	DSHOT1_TIM->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = dshot_dma_tc_callback;
	DSHOT1_TIM->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = dshot_dma_tc_callback;

	HAL_TIM_PWM_Start(DSHOT1_TIM, DSHOT1_MOTOR_1_TIM_CHANNEL);
	HAL_TIM_PWM_Start(DSHOT1_TIM, DSHOT1_MOTOR_2_TIM_CHANNEL);
	HAL_TIM_PWM_Start(DSHOT1_TIM, DSHOT1_MOTOR_3_TIM_CHANNEL);
	HAL_TIM_PWM_Start(DSHOT1_TIM, DSHOT1_MOTOR_4_TIM_CHANNEL);
}

void init_dshot2() {
	uint16_t dshot_prescaler;
	uint32_t timer_clock = DSHOT2_TIMER_CLOCK;
	dshot_prescaler = lrintf((float) timer_clock / DSHOT600_HZ + 0.01f) - 1;
	__HAL_TIM_SET_PRESCALER(DSHOT2_TIM, dshot_prescaler);
	__HAL_TIM_SET_AUTORELOAD(DSHOT2_TIM, MOTOR_BITLENGTH);

	DSHOT2_TIM->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = dshot_dma_tc_callback;
	DSHOT2_TIM->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = dshot_dma_tc_callback;
	DSHOT2_TIM->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = dshot_dma_tc_callback;
	DSHOT2_TIM->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = dshot_dma_tc_callback;

	HAL_TIM_PWM_Start(DSHOT2_TIM, DSHOT2_MOTOR_1_TIM_CHANNEL);
	HAL_TIM_PWM_Start(DSHOT2_TIM, DSHOT2_MOTOR_2_TIM_CHANNEL);
	HAL_TIM_PWM_Start(DSHOT2_TIM, DSHOT2_MOTOR_3_TIM_CHANNEL);
	HAL_TIM_PWM_Start(DSHOT2_TIM, DSHOT2_MOTOR_4_TIM_CHANNEL);
}

void write_dshot1(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4) {
	dshot_prepare_dmabuffer(dshot1_motor1_dmabuffer, motor1);
	dshot_prepare_dmabuffer(dshot1_motor2_dmabuffer, motor2);
	dshot_prepare_dmabuffer(dshot1_motor3_dmabuffer, motor3);
	dshot_prepare_dmabuffer(dshot1_motor4_dmabuffer, motor4);

	HAL_DMA_Start_IT(DSHOT1_TIM->hdma[TIM_DMA_ID_CC1], (uint32_t)dshot1_motor1_dmabuffer, (uint32_t)&DSHOT1_TIM->Instance->CCR1, DSHOT_DMA_BUFFER_SIZE);
	HAL_DMA_Start_IT(DSHOT1_TIM->hdma[TIM_DMA_ID_CC2], (uint32_t)dshot1_motor2_dmabuffer, (uint32_t)&DSHOT1_TIM->Instance->CCR2, DSHOT_DMA_BUFFER_SIZE);
	HAL_DMA_Start_IT(DSHOT1_TIM->hdma[TIM_DMA_ID_CC3], (uint32_t)dshot1_motor3_dmabuffer, (uint32_t)&DSHOT1_TIM->Instance->CCR3, DSHOT_DMA_BUFFER_SIZE);
	HAL_DMA_Start_IT(DSHOT1_TIM->hdma[TIM_DMA_ID_CC4], (uint32_t)dshot1_motor4_dmabuffer, (uint32_t)&DSHOT1_TIM->Instance->CCR4, DSHOT_DMA_BUFFER_SIZE);

	__HAL_TIM_ENABLE_DMA(DSHOT1_TIM, TIM_DMA_CC1);
	__HAL_TIM_ENABLE_DMA(DSHOT1_TIM, TIM_DMA_CC2);
	__HAL_TIM_ENABLE_DMA(DSHOT1_TIM, TIM_DMA_CC3);
	__HAL_TIM_ENABLE_DMA(DSHOT1_TIM, TIM_DMA_CC4);
}

void write_dshot2(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4) {
	dshot_prepare_dmabuffer(dshot2_motor1_dmabuffer, motor1);
	dshot_prepare_dmabuffer(dshot2_motor2_dmabuffer, motor2);
	dshot_prepare_dmabuffer(dshot2_motor3_dmabuffer, motor3);
	dshot_prepare_dmabuffer(dshot2_motor4_dmabuffer, motor4);

	HAL_DMA_Start_IT(DSHOT2_TIM->hdma[TIM_DMA_ID_CC1], (uint32_t)dshot2_motor1_dmabuffer, (uint32_t)&DSHOT2_TIM->Instance->CCR1, DSHOT_DMA_BUFFER_SIZE);
	HAL_DMA_Start_IT(DSHOT2_TIM->hdma[TIM_DMA_ID_CC2], (uint32_t)dshot2_motor2_dmabuffer, (uint32_t)&DSHOT2_TIM->Instance->CCR2, DSHOT_DMA_BUFFER_SIZE);
	HAL_DMA_Start_IT(DSHOT2_TIM->hdma[TIM_DMA_ID_CC3], (uint32_t)dshot2_motor3_dmabuffer, (uint32_t)&DSHOT2_TIM->Instance->CCR3, DSHOT_DMA_BUFFER_SIZE);
	HAL_DMA_Start_IT(DSHOT2_TIM->hdma[TIM_DMA_ID_CC4], (uint32_t)dshot2_motor4_dmabuffer, (uint32_t)&DSHOT2_TIM->Instance->CCR4, DSHOT_DMA_BUFFER_SIZE);

	__HAL_TIM_ENABLE_DMA(DSHOT2_TIM, TIM_DMA_CC1);
	__HAL_TIM_ENABLE_DMA(DSHOT2_TIM, TIM_DMA_CC2);
	__HAL_TIM_ENABLE_DMA(DSHOT2_TIM, TIM_DMA_CC3);
	__HAL_TIM_ENABLE_DMA(DSHOT2_TIM, TIM_DMA_CC4);
}

