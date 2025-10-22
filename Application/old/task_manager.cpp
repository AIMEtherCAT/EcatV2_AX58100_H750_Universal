//
// Created by Hang XU on 2024/3/15.
//

#include "../task_manager.h"
#include <queue>
#include "utypes.h"
#include "cmsis_os.h"
#include "bsp_delay.h"
#include "task_defs.h"
#include "io_utils.h"

extern "C" {
#include "main.h"
}

std::vector<CustomRunnable *> task_list;
std::vector<CanRunnable *> can_list;
std::vector<UartRunnable *> uart_list;
std::vector<I2CRunnable *> i2c_list;
std::vector<runnable_conf *> run_confs;

void CanRunnable::can_recv(FDCAN_RxHeaderTypeDef *, unsigned char *) {
}

void UartRunnable::uart_recv(uint16_t, uint8_t *) {
}

void UartRunnable::uart_err() {
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
            case SBUS_RC_APP_ID: {
                App_SBUS_RC *app = new App_SBUS_RC(global_args, &offset);
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
            case ADC_APP_ID: {
                App_ADC *app = new App_ADC(global_args, &offset);
                task_list.push_back(app);
                break;
            }
            case CAN_PMU_APP_ID: {
                App_CAN_PMU *app = new App_CAN_PMU(global_args, &offset);
                task_list.push_back(app);
                can_list.push_back(app);
                break;
            }
            case DJICAN_APP_ID: {
                App_DJIMotor *app = new App_DJIMotor(global_args, &offset);
                task_list.push_back(app);
                can_list.push_back(app);
                runnable_conf *conf = new runnable_conf();
                conf->runnable = app;
                conf->period = app->period;
                run_confs.push_back(conf);

                const osThreadDef_t task = {
                    nullptr, (soes_device_handle),
                    (osPriorityRealtime), (0), (512),
                    NULL,
                    NULL
                };
                taskDefs.push_back(&task);
                taskHandles.push_back(
                    osThreadCreate(taskDefs.back(), run_confs.back()));
                break;
            }
            case DM_MOTOR_APP_ID: {
                App_DMMotor *app = new App_DMMotor(global_args, &offset);
                task_list.push_back(app);
                can_list.push_back(app);
                runnable_conf *conf = new runnable_conf();
                conf->runnable = app;
                conf->period = app->period;
                run_confs.push_back(conf);

                const osThreadDef_t task = {
                    nullptr, (soes_device_handle),
                    (osPriorityRealtime), (0), (512),
                    NULL,
                    NULL
                };
                taskDefs.push_back(&task);
                taskHandles.push_back(
                    osThreadCreate(taskDefs.back(), run_confs.back()));
                break;
            }
            case LK_APP_ID: {
                App_LkMotor *app = new App_LkMotor(global_args, &offset);
                task_list.push_back(app);
                can_list.push_back(app);
                runnable_conf *conf = new runnable_conf();
                conf->runnable = app;
                conf->period = app->period;
                run_confs.push_back(conf);

                const osThreadDef_t task = {
                    nullptr, (soes_device_handle),
                    (osPriorityRealtime), (0), (512),
                    NULL,
                    NULL
                };
                taskDefs.push_back(&task);
                taskHandles.push_back(
                    osThreadCreate(taskDefs.back(), run_confs.back()));
                break;
            }
        }
    }
}

uint8_t do_task_load = 0;
uint8_t task_loaded = 0;
uint32_t ts3, last_ts3;
uint32_t within = 0;

void soes_task_loader() {
    while (1) {
        if (HAL_GetTick() - within <= 250) {
            ts3 = HAL_GetTick();
            if (ts3 - last_ts3 >= 25) {
                HAL_GPIO_TogglePin(LED_LBOARD_2_GPIO_Port, LED_LBOARD_2_Pin);
                last_ts3 = ts3;
            }
        } else {
            HAL_GPIO_WritePin(LED_LBOARD_2_GPIO_Port, LED_LBOARD_2_Pin, GPIO_PIN_SET);
        }


        if (do_task_load == 1 && task_loaded == 0) {
            within = HAL_GetTick();

            taskENTER_CRITICAL();
            task_load();
            taskEXIT_CRITICAL();

            task_loaded = 1;
        }

        vTaskDelay(5);
    }
}
