//
// Created by Hang XU on 04/06/2025.
//

#ifndef TASK_DEFS_H
#define TASK_DEFS_H

#include <memory>
#include <vector>

extern "C" {
#include "main.h"
}

namespace aim::ecat::task {

    using utils::ThreadSafeFlag;
    using namespace io;
    using namespace hardware;

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

    class CustomRunnable {
    public:
        virtual ~CustomRunnable() = default;

        // unit: ms
        uint16_t period{};

        TaskType task_type{};

        ThreadSafeFlag running{true};

        virtual void run_task() {
        }

        virtual void write_to_master(buffer::Buffer *slave_to_master_buf);

        virtual void read_from_master(buffer::Buffer *master_to_slave_buf);

        virtual void exit() {
        }

        template<typename T = peripheral::Peripheral>
        [[nodiscard]] T *get_peripheral() const {
            return static_cast<T *>(_peripheral);
        }

    protected:
        peripheral::Peripheral *_peripheral{};
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
        explicit Task_DBUS_RC(buffer::Buffer buffer);

        void write_to_master(buffer::Buffer *slave_to_master_buf) override;

        void uart_recv(uint16_t size) override;

        void uart_err() override;

        void exit() override;

    private:
        uint32_t _last_receive_time{};
        uint8_t _buf[19]{};
    };

    class Task_DJI_MOTOR final : public CanRunnable {
    public:
        explicit Task_DJI_MOTOR(buffer::Buffer buffer);

        void write_to_master(buffer::Buffer *slave_to_master_buf) override;

        void read_from_master(buffer::Buffer *master_to_slave_buf) override;

        void can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) override;

        void run_task() override;

        void exit() override;

    private:

        struct dji_motor_status_t {
            uint16_t ecd;
            int16_t rpm;
            int16_t current;
            uint8_t temperature;
        } ;

        enum class dji_ctrl_mode_e : uint8_t {
            OPENLOOP_CURRENT = 0x01,
            SPEED = 0x02,
            SINGLE_ROUND_POSITION = 0x03
        };

        enum class dji_motor_status_e {

        };

        struct dji_motor_t {
            bool enabled{false};
            dji_ctrl_mode_e control_mode{};
            dji_motor_status_t status{};

        };

        FDCAN_TxHeaderTypeDef _shared_tx_header{};
        uint8_t _shared_tx_buf[8]{};

    };
}

#endif //TASK_DEFS_H
