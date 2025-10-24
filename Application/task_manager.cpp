//
// Created by Hang XU on 21/10/2025.
//

#include "task_manager.hpp"

#include "buffer_manager.hpp"
#include "peripheral_manager.hpp"
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

    ThreadSafeCounter terminated_counter;

    ThreadSafeCounter *get_terminated_counter() {
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
                    conf->is_can_task.set();
                    conf->runnable = std::make_unique<lk_motor::LK_MOTOR>(
                        buffer::get_buffer(buffer::Type::ECAT_ARGS)
                    );
                    break;
                }
                case static_cast<uint8_t>(TaskType::HIPNUC_IMU_CAN): {
                    conf->is_can_task.set();
                    conf->runnable = std::make_unique<hipnuc_imu::HIPNUC_IMU_CAN>(
                        buffer::get_buffer(buffer::Type::ECAT_ARGS)
                    );
                    break;
                }
                case static_cast<uint8_t>(TaskType::DSHOT): {
                    conf->runnable = std::make_unique<pwm::DSHOT600>(
                        buffer::get_buffer(buffer::Type::ECAT_ARGS)
                    );
                    break;
                }
                case static_cast<uint8_t>(TaskType::DJI_MOTOR): {
                    conf->is_can_task.set();
                    conf->runnable = std::make_unique<dji_motor::DJI_MOTOR>(
                        buffer::get_buffer(buffer::Type::ECAT_ARGS)
                    );
                    break;
                }
                case static_cast<uint8_t>(TaskType::ONBOARD_PWM): {
                    conf->runnable = std::make_unique<pwm::PWM_ONBOARD>(
                        buffer::get_buffer(buffer::Type::ECAT_ARGS)
                    );
                    break;
                }
                case static_cast<uint8_t>(TaskType::EXTERNAL_PWM): {
                    conf->runnable = std::make_unique<pwm::PWM_EXTERNAL>(
                        buffer::get_buffer(buffer::Type::ECAT_ARGS)
                    );
                    break;
                }
                case static_cast<uint8_t>(TaskType::MS5876_30BA): {
                    break;
                }
                case static_cast<uint8_t>(TaskType::ADC): {
                    conf->runnable = std::make_unique<adc::ADC>(
                        buffer::get_buffer(buffer::Type::ECAT_ARGS)
                    );
                    break;
                }
                case static_cast<uint8_t>(TaskType::CAN_PMU): {
                    conf->is_can_task.set();
                    conf->runnable = std::make_unique<pmu_uavcan::PMU_UAVCAN>(
                        buffer::get_buffer(buffer::Type::ECAT_ARGS)
                    );
                    break;
                }
                case static_cast<uint8_t>(TaskType::SBUS_RC): {
                    conf->is_uart_task.set();
                    conf->runnable = std::make_unique<sbus_rc::SBUS_RC>(
                        buffer::get_buffer(buffer::Type::ECAT_ARGS)
                    );
                    break;
                }
                case static_cast<uint8_t>(TaskType::DM_MOTOR): {
                    conf->is_can_task.set();
                    conf->runnable = std::make_unique<dm_motor::DM_MOTOR>(
                        buffer::get_buffer(buffer::Type::ECAT_ARGS)
                    );
                    break;
                }
                default: {
                }
            }

            run_confs.push_back(conf);
            if (conf->runnable->is_run_task_enabled_) {
                conf->thread_def = {
                    .name = nullptr,
                    .pthread = task_thread_func,
                    .tpriority = osPriorityRealtime,
                    .instances = 0,
                    .stacksize = 1024,
                    .buffer = nullptr,
                    .controlblock = nullptr
                };
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
                    __HAL_TIM_SET_AUTORELOAD(&htim17, LED_TASK_LOADED_ARR);
                    __HAL_TIM_SET_COUNTER(&htim17, 0);
                    last_slave_ready_flag.set();
                } else {
                    __HAL_TIM_SET_AUTORELOAD(&htim17, LED_TASK_NOT_LOADED_ARR);
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

void task_manager(const void *) {
    aim::ecat::task::task_manager_impl();
}
