//
// Created by Hang XU on 21/04/2025.
//

#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <stm32h7xx_hal.h>

#define DJIRC_APP_ID 1
#define LK_APP_ID 2
#define HIPNUC_IMU_CAN_APP_ID 3
#define DSHOT_APP_ID 4
#define DJICAN_APP_ID 5
#define VANILLA_PWM_APP_ID 6
#define EXTERNAL_PWM_APP_ID 7
#define MS5876_30BA_APP_ID 8

#ifdef __cplusplus
extern "C" {
#endif

class CustomRunnable {
public:
    virtual ~CustomRunnable() = default;

    // unit: ms
    uint16_t period{};

    uint8_t task_type{};

    uint8_t running{};

    virtual void run_task();

    virtual void collect_outputs(uint8_t *output, int *output_offset);

    virtual void collect_inputs(uint8_t *input, int *input_offset);

    virtual void exit();
};

class CanRunnable : public virtual CustomRunnable {
public:
    ~CanRunnable() override = default;

    FDCAN_HandleTypeDef *can_inst{};

    virtual void can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);
};

class UartRunnable : public virtual CustomRunnable {
public:
    ~UartRunnable() override = default;

    UART_HandleTypeDef *uart_inst{};

    virtual void uart_recv(uint16_t size, uint8_t *rx_data);

    virtual void uart_recv_err();

    virtual void uart_dma_tx_finished_callback();
};

class I2CRunnable : public virtual CustomRunnable {
public:
    ~I2CRunnable() override = default;

    I2C_HandleTypeDef *i2c_inst{};

    virtual void i2c_recv(uint8_t *rx_data);

    virtual void i2c_recv_err();

    virtual void i2c_dma_tx_finished_callback();
};

void task_spin();

void collect_spin();

void task_load();

typedef struct {
    CustomRunnable *runnable;
    uint16_t period;
} runnable_conf;

#ifdef __cplusplus
}
#endif

#endif //TASK_MANAGER_H
