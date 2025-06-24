//
// Created by Hang XU on 2024/3/15.
//

#include "task_manager.h"
#include <queue>
#include "utypes.h"
#include "cmsis_os.h"
#include "bsp_delay.h"
#include "task_defs.h"
#include "IOUtils.h"

std::vector<CustomRunnable *> task_list;
std::vector<CanRunnable *> can_list;
std::vector<UartRunnable *> uart_list;
std::vector<I2CRunnable *> i2c_list;
std::vector<runnable_conf *> run_confs;

void CanRunnable::can_recv(FDCAN_RxHeaderTypeDef *, unsigned char *) {
}

void UartRunnable::uart_recv(uint16_t, uint8_t *) {
}

void UartRunnable::uart_recv_err() {
}

void UartRunnable::uart_dma_tx_finished_callback() {
}

void I2CRunnable::i2c_recv(uint8_t *) {
}

void I2CRunnable::i2c_recv_err() {
}

void I2CRunnable::i2c_dma_tx_finished_callback() {
}

void CustomRunnable::run_task() {
}

void CustomRunnable::collect_outputs(unsigned char *, int *) {
}

void CustomRunnable::collect_inputs(unsigned char *, int *) {
}

void CustomRunnable::exit() {
}

std::vector<osThreadId> taskHandles;
std::vector<const osThreadDef_t *> taskDefs;

extern uint8_t global_inputs[1024];
extern uint8_t global_outputs[1024];
extern uint8_t global_args[1024];

void task_spin() {
    int input_offset = 0;
    for (CustomRunnable *runnable: task_list) {
        runnable->collect_inputs(global_inputs, &input_offset);
    }
}

void collect_spin() {
    int output_offset = 0;
    for (CustomRunnable *runnable: task_list) {
        runnable->collect_outputs(global_outputs, &output_offset);
    }
}

uint8_t do_terminate = 0;
uint8_t terminated_app_count = 0;

__attribute__((noreturn)) void soes_device_handle(void const *argument) {
    runnable_conf *conf_inst = (runnable_conf *) argument;
    while (true) {
        if (do_terminate == 1) {
            conf_inst->runnable->exit();
            terminated_app_count++;
            vTaskDelete(nullptr);
            return;
        }

        conf_inst->runnable->run_task();
        vTaskDelay(conf_inst->period);
    }
}

void task_load() {
    int offset = 1;
    uint8_t task_count = read_uint8(global_args, &offset);

    while (task_list.size() != task_count) {
        switch (read_uint8(global_args, &offset)) {
            case DJIRC_APP_ID: {
                App_DJI_RC *app = new App_DJI_RC(global_args, &offset);
                task_list.push_back(app);
                uart_list.push_back(app);
                break;
            }
            case HIPNUC_IMU_CAN_APP_ID: {
                App_HIPNUC_IMU *app = new App_HIPNUC_IMU(global_args, &offset);
                task_list.push_back(app);
                can_list.push_back(app);
                break;
            }
            case DSHOT_APP_ID: {
                App_DSHOT *app = new App_DSHOT(global_args, &offset);
                task_list.push_back(app);
                break;
            }
            case EXTERNAL_PWM_APP_ID: {
                App_External_PWM *app = new App_External_PWM(global_args, &offset);
                task_list.push_back(app);
                uart_list.push_back(app);
                break;
            }
            case VANILLA_PWM_APP_ID: {
                App_Vanilla_PWM *app = new App_Vanilla_PWM(global_args, &offset);
                task_list.push_back(app);
                break;
            }
            case MS5876_30BA_APP_ID: {
                App_MS5837_30BA *app = new App_MS5837_30BA(global_args, &offset);
                task_list.push_back(app);
                i2c_list.push_back(app);
                break;
            }
        }
    }
}
