//
// Created by Hang XU on 21/10/2025.
//

#include "task_manager.hpp"

#include "buffer_manager.hpp"
#include "peripheral_manager.hpp"
#include "settings.h"
#include "task_defs.hpp"
#include "c_task_warpper.h"
#include "soes_application.hpp"

extern "C" {
#include "gpio.h"
#include "tim.h"
}

void task_thread_func(void const *argument) {
    aim::ecat::task::task_thread_func_impl(argument);
    vTaskDelete(nullptr);
}

namespace aim::ecat::task {
    osMutexId runConfMutexHandle;

    utils::ThreadSafeCounter terminated_counter;

    utils::ThreadSafeCounter *get_terminated_counter() {
        return &terminated_counter;
    }

    std::vector<std::shared_ptr<runnable_conf> > run_confs;

    std::vector<std::shared_ptr<runnable_conf> > *get_run_confs() {
        return &run_confs;
    }

    void init_task_manager() {
        osMutexDef(runConfMutex);
        runConfMutexHandle = osMutexCreate(osMutex(runConfMutex));
    }

    // ReSharper disable once CppParameterMayBeConstPtrOrRef
    void task_thread_func_impl(void const *argument) {
        std::shared_ptr<runnable_conf> conf_inst;
        osMutexWait(runConfMutexHandle, osWaitForever);
        for (std::shared_ptr<runnable_conf> &ptr: run_confs) {
            if (ptr.get() == argument) {
                conf_inst = ptr;
                break;
            }
        }
        osMutexRelease(runConfMutexHandle);

        // ReSharper disable once CppDFAEndlessLoop
        while (true) {
            if (!conf_inst->runnable->running.get()) {
                break;
            }
            conf_inst->runnable->run_task();
            vTaskDelay(conf_inst->runnable->period);
        }

        conf_inst->runnable->exit();
        terminated_counter.increment();
    }

    static void load_task() {
        buffer::get_buffer(buffer::Type::ECAT_ARGS)->reset_index();
        buffer::get_buffer(buffer::Type::ECAT_ARGS)->skip(1);
        const uint8_t task_count = buffer::get_buffer(buffer::Type::ECAT_ARGS)->read_uint8();
        while (run_confs.size() < task_count) {
            auto conf = std::make_shared<runnable_conf>();

            switch (buffer::get_buffer(buffer::Type::ECAT_ARGS)->read_uint8()) {
                case static_cast<uint8_t>(TaskType::DBUS_RC): {
                    conf->is_uart_task.set();
                    conf->runnable = std::make_unique<dbus_rc::DBUS_RC>(
                        buffer::get_buffer(buffer::Type::ECAT_ARGS)
                    );
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
                    conf->is_can_task.set();
                    conf->runnable = std::make_unique<dji_motor::DJI_MOTOR>(
                        buffer::get_buffer(buffer::Type::ECAT_ARGS)
                    );
                    conf->thread_def = {
                        .name = nullptr,
                        .pthread = task_thread_func,
                        .tpriority = osPriorityRealtime,
                        .instances = 0,
                        .stacksize = 1024,
                        .buffer = nullptr,
                        .controlblock = nullptr
                    };

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
            if (conf->thread_def.stacksize != 0) {
                osThreadCreate(&conf->thread_def, conf.get());
            }
        }
    }

    static ThreadSafeFlag last_slave_ready_flag;

    [[noreturn]] void task_manager_impl() {
        while (true) {
            if (application::get_is_task_loaded()->get()) {
                HAL_GPIO_WritePin(LED_LBOARD_2_GPIO_Port, LED_LBOARD_2_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(LED_LBOARD_2_GPIO_Port, LED_LBOARD_2_Pin, GPIO_PIN_SET);
            }

            if (last_slave_ready_flag.get() != application::get_is_slave_ready()->get()) {
                if (application::get_is_slave_ready()->get()) {
                    // task loaded, 40hz / 25ms
                    __HAL_TIM_SET_AUTORELOAD(&htim17, LED_25MS_ARR);
                    __HAL_TIM_SET_COUNTER(&htim17, 0);
                    last_slave_ready_flag.set();
                } else {
                    // task not loaded, 4hz / 250ms
                    __HAL_TIM_SET_AUTORELOAD(&htim17, LED_250MS_ARR);
                    __HAL_TIM_SET_COUNTER(&htim17, 0);
                    last_slave_ready_flag.clear();
                }
            }

            if (application::get_is_task_ready_to_load()->get()) {
                load_task();
                application::get_is_task_ready_to_load()->clear();
                application::get_is_task_loaded()->set();
            }

            vTaskDelay(10);
        }
    }
}

[[noreturn]] void task_manager() {
    aim::ecat::task::task_manager_impl();
}
