//
// Created by Hang XU on 04/06/2025.
//

#ifndef TASK_DEFS_H
#define TASK_DEFS_H

#include "task_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RC_CHANNAL_ERROR_VALUE 1700

struct servo_cmd_t {
    uint8_t header;

    uint16_t expected_period;

    uint16_t servo_cmd[16];

    uint16_t checksum;
} __attribute__((packed));

class App_DJI_RC : public UartRunnable {
public:
    App_DJI_RC(uint8_t *args, int *offset);

    void collect_outputs(uint8_t *output, int *output_offset) override;

    void uart_recv(uint16_t size, uint8_t *rx_data) override;

    void uart_recv_err() override;

    void exit() override;

private:
    uint32_t rc_last_receive_time{};
    uint8_t rc_buf[19]{};
};

class App_DSHOT : public CustomRunnable {
public:
    App_DSHOT(uint8_t *args, int *offset);

    void collect_inputs(uint8_t *input, int *input_offset) override;

    void exit() override;

private:
    uint8_t dshot_id{};
};

class App_HIPNUC_IMU : public CanRunnable {
public:
    App_HIPNUC_IMU(uint8_t *args, int *offset);

    void collect_outputs(uint8_t *output, int *output_offset) override;

    void can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) override;

    void exit() override;

private:
    uint8_t imu_buf[8 + 8 + 5]{};
};


#define TIM_FREQ 240000000

class App_Vanilla_PWM : public CustomRunnable {
public:
    App_Vanilla_PWM(uint8_t *args, int *offset);

    void collect_inputs(uint8_t *input, int *input_offset) override;

    void exit() override;

private:
    uint8_t tim_id{};
    TIM_HandleTypeDef *tim_inst{};

    uint16_t best_psc{};
    uint16_t best_arr{};
    uint16_t expected_period{};
    uint16_t cmd[4]{};
    uint8_t tim_started{};

    uint32_t calc_compare(uint16_t expected_high_pulse) const;
};

class App_External_PWM : public UartRunnable {
public:
    App_External_PWM(uint8_t *args, int *offset);

    void collect_inputs(uint8_t *input, int *input_offset) override;

    void exit() override;

    void uart_dma_tx_finished_callback() override;

private:
    uint8_t uart_id{};
    uint8_t buf[37]{};
    uint8_t enabled_channel_count{};
    uint16_t expected_period{};
    uint32_t last_tx_ts{};
    uint32_t last_rst_ts{};
    servo_cmd_t cmd{};
};

#define MS5837_D1_OSR256_CMD 0x40
#define MS5837_D1_OSR512_CMD 0x42
#define MS5837_D1_OSR1024_CMD 0x44
#define MS5837_D1_OSR2048_CMD 0x46
#define MS5837_D1_OSR4096_CMD 0x48
#define MS5837_D1_OSR8192_CMD 0x4A

#define MS5837_D2_OSR256_CMD 0x50
#define MS5837_D2_OSR512_CMD 0x52
#define MS5837_D2_OSR1024_CMD 0x54
#define MS5837_D2_OSR2048_CMD 0x56
#define MS5837_D2_OSR4096_CMD 0x58
#define MS5837_D2_OSR8192_CMD 0x5A

#define MS5837_ADC_READ_CMD 0x00
#define MS5837_PROM_READ_CMD_BEGIN 0xA0
#define MS5837_RESET_CMD 0x1E
#define MS5837_ADDR (0x76 << 1)

class App_MS5837_30BA : public I2CRunnable {
public:
    App_MS5837_30BA(uint8_t *args, int *offset);

    void collect_inputs(uint8_t *input, int *input_offset) override;

    void collect_outputs(uint8_t *output, int *output_offset) override;

    void i2c_dma_tx_finished_callback() override;

    void i2c_recv(uint8_t *rx_data) override;

    void exit() override;

    void i2c_recv_err() override;

private:
    uint8_t i2c_id{};
    uint8_t osr_id{};

    uint8_t d1_cmd{};
    uint8_t d2_cmd{};
    uint32_t adc_wait_time{};
    uint32_t last_recv_time{};
    uint32_t last_rst_time{};
    uint8_t inited{};

    enum {
        INITIALIZING,

        READING_C1,
        READING_C2,
        READING_C3,
        READING_C4,
        READING_C5,
        READING_C6,

        READING_D1,
        READING_D2,

        WAITING,
    } current_stage{};

    uint32_t last_act_ts{};
    uint8_t send_called{};
    uint8_t recv_called{};

    uint16_t C1_Pressure_sensitivity{};
    uint16_t C2_Pressure_offset{};
    uint16_t C3_Temperature_coefficient_of_pressure_sensitivity{};
    uint16_t C4_Temperature_coefficient_of_pressure_offset{};
    uint16_t C5_Reference_temperature{};
    uint16_t C6_Temperature_coefficient_of_the_temperature{};

    uint32_t D1_Digital_pressure_value{};
    uint32_t D2_Digital_temperature_value{};

    int32_t dT{};
    int32_t TEMP{};

    int64_t OFF{};
    int64_t SENS{};
    int32_t P{};

    uint8_t buf[256]{};
    uint8_t tx_finished{};

    int64_t Ti{};
    int64_t OFFi{};
    int64_t SENSi{};
    int64_t OFF2{};
    int64_t SENS2{};

    float TEMP2{};
    float P2{};
};
#ifdef __cplusplus
}
#endif
#endif //TASK_DEFS_H
