//
// Created by Hang XU on 04/06/2025.
//

#ifndef TASK_DEFS_H
#define TASK_DEFS_H

#include <memory>

class CustomRunnable;

extern "C" {
#include "main.h"
}

enum class TaskType : uint8_t {
    DBUS_RC = 1,
    LK_MOTOR = 2,
    HIPNUC_IMU_CAN = 3,
    DSHOT = 4,
    DJI_MOTOR = 5,
    ONBOARD_PWM = 6,
    EXTERNAL_PWM = 7,
    MS5876_30BA = 8,
    ADC = 9,
    CAN_PMU = 10,
    SBUS_RC = 11,
    DM_MOTOR = 12
};

struct runnable_conf {
    std::unique_ptr<CustomRunnable> runnable;
    uint16_t period{};

    ThreadSafeFlag is_can_task;
    ThreadSafeFlag is_uart_task;
    ThreadSafeFlag is_i2c_task;
    ThreadSafeFlag is_adc_task;
};

class CustomRunnable {
public:
    virtual ~CustomRunnable() = default;

    // unit: ms
    uint16_t period{};

    TaskType task_type{};

    ThreadSafeFlag running{true};

    virtual void run_task() {
    }

    virtual void write_to_master(Buffer *slave_to_master_buf);

    virtual void read_from_master(Buffer *master_to_slave_buf);

    virtual void exit() {
    }

    template<typename T = Peripheral>
    [[nodiscard]] T *get_peripheral() const {
        return static_cast<T *>(_peripheral);
    }

protected:
    Peripheral *_peripheral{};
};

class CanRunnable : public CustomRunnable {
public:
    ~CanRunnable() override = default;

    FDCAN_HandleTypeDef *can_inst{};

    uint32_t can_id_type{};

    virtual void can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);
};

class UartRunnable : public CustomRunnable {
public:
    ~UartRunnable() override = default;

    virtual void uart_recv(uint16_t size);

    virtual void uart_err();

    virtual void uart_dma_tx_finished_callback();
};

class I2CRunnable : public CustomRunnable {
public:
    ~I2CRunnable() override = default;

    virtual void i2c_recv(uint16_t size);

    virtual void i2c_err();

    virtual void i2c_dma_tx_finished_callback();
};

class Task_DBUS_RC final : public UartRunnable {
public:
    explicit Task_DBUS_RC(Buffer buffer);

    void write_to_master(Buffer *slave_to_master_buf) override;

    void uart_recv(uint16_t size) override;

    void uart_err() override;

    void exit() override;

private:
    uint32_t _last_receive_time{};
    uint8_t _buf[19]{};
};

#endif //TASK_DEFS_H
