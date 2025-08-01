//
// Created by Hang XU on 04/06/2025.
//

#ifndef TASK_DEFS_H
#define TASK_DEFS_H

#include "pid.h"
#include "task_manager.h"

#define DJIRC_APP_ID 1
#define LK_APP_ID 2
#define HIPNUC_IMU_CAN_APP_ID 3
#define DSHOT_APP_ID 4
#define DJICAN_APP_ID 5
#define VANILLA_PWM_APP_ID 6
#define EXTERNAL_PWM_APP_ID 7
#define MS5876_30BA_APP_ID 8
#define ADC_APP_ID 9
#define CAN_PMU_APP_ID 10

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

    uint32_t can_id_type{};

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

typedef struct {
    CustomRunnable *runnable;
    uint16_t period;
} runnable_conf;

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

class App_ADC : public CustomRunnable {
public:
    App_ADC(uint8_t *args, int *offset);

    void collect_outputs(uint8_t *input, int *input_offset) override;

    void exit() override;

private:
    float parsed_adc_value[2]{};
    float coefficient[2]{};
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

    uint8_t err_times{};

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

    float _TEMP2{};
    float _P2{};
};

#define UAVCAN_BUF_SIZE 64
#define UAVCAN_TID_TIMEOUT_MS 1000

typedef struct {
    uint8_t buffer[UAVCAN_BUF_SIZE];
    uint16_t len;
    uint16_t crc;
    uint8_t initialized;
    uint8_t toggle;
    uint8_t transfer_id;
    uint32_t last_ts;
} UAVCAN_RxState_t;

typedef struct {
    uint8_t start;
    uint8_t end;
    uint8_t toggle;
    uint8_t tid;
} UAVCAN_TailByte_t;

class App_CAN_PMU : public CanRunnable {
public:
    App_CAN_PMU(uint8_t *args, int *offset);

    void collect_inputs(uint8_t *input, int *input_offset) override;

    void collect_outputs(uint8_t *input, int *input_offset) override;

    void can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) override;

    void exit() override;

private:
    UAVCAN_RxState_t uavcan_rx{};

    static UAVCAN_TailByte_t parse_tail_byte(uint8_t tail) {
        UAVCAN_TailByte_t t;
        t.start = (tail >> 7) & 1;
        t.end = (tail >> 6) & 1;
        t.toggle = (tail >> 5) & 1;
        t.tid = tail & 0x1F;
        return t;
    }

    uint8_t pmu_buffer[6]{};
    uint8_t node_status_data[8]{};
    FDCAN_TxHeaderTypeDef can_tx_header{};
    uint32_t last_node_status_pub_ts{};
    uint32_t uptime{};
    uint8_t transfer_id{};
};

typedef struct {
    uint16_t ecd;
    int16_t rpm;
    int16_t current;
    uint8_t temperature;
} dji_motor_status_t;

typedef enum {
    OPENLOOP_CURRENT = 0x01, SPEED = 0x02, SINGLE_ROUND_POSITION = 0x03
} dji_ctrl_mode_e;

class App_DJIMotor : public CanRunnable {
public:
    App_DJIMotor(uint8_t *args, int *offset);

    void collect_inputs(uint8_t *input, int *input_offset) override;

    void collect_outputs(uint8_t *input, int *input_offset) override;

    void can_recv(FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) override;

    void exit() override;

    void run_task() override;

private:
    FDCAN_TxHeaderTypeDef shared_tx_header{};
    uint8_t shared_tx_data[8]{};
    dji_motor_status_t motor_status[4]{};
    uint32_t can_motor_report_packet_id[4]{};
    uint32_t can_packet_id{};
    uint8_t can_inst_id{};
    dji_ctrl_mode_e ctrl_mode[4]{};
    uint8_t cmd_motor_enable[4]{};
    int16_t cmd[4]{};
    pid_type_def speed_pid[4]{};
    pid_type_def angle_pid[4]{};

    void disable_motor();

    static int16_t calc_err(int16_t current_angle, int16_t target_angle);
};

#ifdef __cplusplus
}
#endif
#endif //TASK_DEFS_H
