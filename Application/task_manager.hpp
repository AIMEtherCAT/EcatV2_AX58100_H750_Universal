//
// Created by Hang XU on 21/10/2025.
//

#ifndef ECATV2_AX58100_H750_UNIVERSAL_TASK_MANAGER_H
#define ECATV2_AX58100_H750_UNIVERSAL_TASK_MANAGER_H

#include "peripheral_manager.hpp"
#include "task_defs.hpp"

namespace aim::ecat::task {
    struct runnable_conf {
        std::unique_ptr<CustomRunnable> runnable{nullptr};
        osThreadDef_t thread_def{};

        ThreadSafeFlag is_can_task{};
        ThreadSafeFlag is_uart_task{};
        ThreadSafeFlag is_i2c_task{};
    };

    void task_thread_func_impl(void const *argument);

    void init_task_manager();

    std::vector<std::shared_ptr<runnable_conf> > *get_run_confs();

    ThreadSafeCounter *get_terminated_counter();

    constexpr uint32_t LED_PSC = 23999;
    // task loaded, 40hz / 25ms
    constexpr uint32_t LED_TASK_LOADED_ARR = 249;
    // task not loaded, 4hz / 250ms
    constexpr uint32_t LED_TASK_NOT_LOADED_ARR = 2499;
}

#endif //ECATV2_AX58100_H750_UNIVERSAL_TASK_MANAGER_H