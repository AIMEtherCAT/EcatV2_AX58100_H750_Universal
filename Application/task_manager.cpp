//
// Created by Hang XU on 21/10/2025.
//

#include "task_manager.h"

#include <memory>
#include <vector>

#include "buffer_manager.hpp"
#include "peripheral_manager.hpp"
#include "settings.h"
#include "task_defs.h"

extern "C" {
#include "gpio.h"
#include "tim.h"
}

extern ThreadSafeFlag is_task_loaded;
extern ThreadSafeFlag is_task_ready_to_load;
extern ThreadSafeFlag is_slave_ready;
osMutexId appTerminationMutexHandle;
osMutexId runConfMutexHandle;
uint8_t terminated_task_counter = 0;

std::vector<std::shared_ptr<runnable_conf> > run_confs;

void init_task_manager() {
    osMutexDef(appTerminationMutex);
    appTerminationMutexHandle = osMutexCreate(osMutex(appTerminationMutex));
    osMutexDef(runConfMutex);
    runConfMutexHandle = osMutexCreate(osMutex(runConfMutex));
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
void task_thread_func(void *argument) {
    std::shared_ptr<runnable_conf> conf_inst; {
        osMutexWait(runConfMutexHandle, osWaitForever);
        for (auto &ptr: run_confs) {
            if (ptr.get() == argument) {
                conf_inst = ptr;
                break;
            }
        }
        osMutexRelease(runConfMutexHandle);
    }

    // ReSharper disable once CppDFAEndlessLoop
    while (true) {
        if (!conf_inst->runnable->running.get()) {
            break;
        }
        conf_inst->runnable->run_task();
        vTaskDelay(conf_inst->runnable->period);
    }

    conf_inst->runnable->exit();
    osMutexWait(appTerminationMutexHandle, osWaitForever);
    terminated_task_counter++;
    osMutexRelease(appTerminationMutexHandle);
    vTaskDelete(nullptr);
}

static void load_task() {
    get_buffer(BufferType::ECAT_ARGS)->reset_index();
    get_buffer(BufferType::ECAT_ARGS)->skip(1);
    const uint8_t task_count = get_buffer(BufferType::ECAT_ARGS)->read_uint8();
    while (run_confs.size() < task_count) {
        auto conf = std::make_shared<runnable_conf>();

        switch (get_buffer(BufferType::ECAT_ARGS)->read_uint8()) {
            case static_cast<uint8_t>(TaskType::DBUS_RC): {
                conf->is_uart_task.set();
                break;
            }
            case static_cast<uint8_t>(TaskType::LK_MOTOR): {
                break;
            }
            case static_cast<uint8_t>(TaskType::HIPNUC_IMU_CAN): {
                break;
            }
            case static_cast<uint8_t>(TaskType::DSHOT): {
                break;
            }
            case static_cast<uint8_t>(TaskType::DJI_MOTOR): {
                break;
            }
            case static_cast<uint8_t>(TaskType::ONBOARD_PWM): {
                break;
            }
            case static_cast<uint8_t>(TaskType::EXTERNAL_PWM): {
                break;
            }
            case static_cast<uint8_t>(TaskType::MS5876_30BA): {
                break;
            }
            case static_cast<uint8_t>(TaskType::ADC): {
                break;
            }
            case static_cast<uint8_t>(TaskType::CAN_PMU): {
                break;
            }
            case static_cast<uint8_t>(TaskType::SBUS_RC): {
                break;
            }
            case static_cast<uint8_t>(TaskType::DM_MOTOR): {
                break;
            }
            default: {
            }
        }

        run_confs.push_back(conf);
    }
}

[[noreturn]] void task_manager() {
    while (true) {
        if (is_task_loaded.get()) {
            HAL_GPIO_WritePin(LED_LBOARD_2_GPIO_Port, LED_LBOARD_2_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(LED_LBOARD_2_GPIO_Port, LED_LBOARD_2_Pin, GPIO_PIN_SET);
        }

        if (is_slave_ready.get()) {
            // task loaded, 20hz / 50ms
            __HAL_TIM_SET_AUTORELOAD(&htim17, LED_50MS_ARR);
        } else {
            // task not loaded, 4hz / 250ms
            __HAL_TIM_SET_AUTORELOAD(&htim17, LED_250MS_ARR);
        }

        if (is_task_ready_to_load.get()) {
            load_task();
            is_task_ready_to_load.clear();
            is_task_loaded.set();
        }

        vTaskDelay(100);
    }
}
