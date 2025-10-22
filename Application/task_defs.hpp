//
// Created by Hang XU on 04/06/2025.
//

#ifndef TASK_DEFS_H
#define TASK_DEFS_H

#include <memory>
#include <vector>

#include "pid.hpp"

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

        virtual void write_to_master(buffer::Buffer * /* slave_to_master_buf */) {
        }

        virtual void read_from_master(buffer::Buffer * /* master_to_slave_buf */) {
        }

        virtual void exit() {
        }

        template<typename T = peripheral::Peripheral>
        [[nodiscard]] T *get_peripheral() const {
            return static_cast<T *>(peripheral_);
        }

    protected:
        peripheral::Peripheral *peripheral_{};
    };

    class CanRunnable : public CustomRunnable {
    public:
        ~CanRunnable() override = default;

        FDCAN_HandleTypeDef *can_inst_{};

        uint32_t can_id_type_{};

        virtual void can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);
    };

    class UartRunnable : public CustomRunnable {
    public:
        ~UartRunnable() override = default;

        virtual void uart_recv(uint16_t size);

        virtual void uart_err() {
        }

        virtual void uart_dma_tx_finished_callback() {
        }
    };

    // ReSharper disable once CppClassCanBeFinal
    class I2CRunnable : public CustomRunnable {
    public:
        ~I2CRunnable() override = default;

        virtual void i2c_recv(uint16_t size);

        virtual void i2c_err() {
        }

        virtual void i2c_dma_tx_finished_callback() {
        }
    };

    namespace dbus_rc {
        constexpr uint16_t DBUS_RC_CHANNAL_ERROR_VALUE = 1700;

        class DBUS_RC final : public UartRunnable {
        public:
            explicit DBUS_RC(buffer::Buffer *buffer);

            void write_to_master(buffer::Buffer *slave_to_master_buf) override;

            void uart_recv(uint16_t size) override;

            void uart_err() override;

            void exit() override;

        private:
            uint32_t last_receive_time_{};
            uint8_t buf_[19]{};
        };
    }

    namespace dji_motor {
        struct MotorReport {
            std::atomic<uint16_t> ecd{};
            std::atomic<int16_t> rpm{};
            std::atomic<int16_t> current{};
            std::atomic<uint8_t> temperature{};
            std::atomic<uint32_t> last_receive_time{};
        };

        enum class CtrlMode : uint8_t {
            OPENLOOP_CURRENT = 0x01,
            SPEED = 0x02,
            SINGLE_ROUND_POSITION = 0x03
        };

        struct ControlCommand {
            std::atomic<bool> is_enable;
            std::atomic<int16_t> cmd;
        };

        struct Motor {
            bool is_exist{false};

            MotorReport report{};
            CtrlMode mode{};
            ControlCommand command{};
            uint32_t report_packet_id{};

            algorithms::PID speed_pid{};
            algorithms::PID angle_pid{};

            [[nodiscard]] bool is_online() const {
                return HAL_GetTick() - report.last_receive_time.load(std::memory_order_acquire) <= 50;
            }
        };

        inline int16_t calculate_err(const uint16_t current_angle, const uint16_t target_angle) {
            constexpr int total_positions = 8192;
            const int clockwise_difference = (target_angle - current_angle
                                              + total_positions) % total_positions;
            const int counterclockwise_difference = (current_angle - target_angle
                                                     + total_positions) % total_positions;
            if (clockwise_difference <= counterclockwise_difference) {
                return static_cast<int16_t>(clockwise_difference);
            }
            return static_cast<int16_t>(-counterclockwise_difference);
        }

        class DJI_MOTOR final : public CanRunnable {
        public:
            explicit DJI_MOTOR(buffer::Buffer *buffer);

            void write_to_master(buffer::Buffer *slave_to_master_buf) override;

            void read_from_master(buffer::Buffer *master_to_slave_buf) override;

            void can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) override;

            void run_task() override;

            void exit() override;

        private:
            FDCAN_TxHeaderTypeDef shared_tx_header_{};
            uint8_t shared_tx_buf_[8]{};
            Motor motors_[4]{};
        };
    }
}

#endif //TASK_DEFS_H
