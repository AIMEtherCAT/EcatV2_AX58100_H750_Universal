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
        ThreadSafeFlag is_adc_task{};
    };

    void task_thread_func_impl(void const *argument);

    void init_task_manager();

    std::vector<std::shared_ptr<runnable_conf> > *get_run_confs();

    utils::ThreadSafeCounter *get_terminated_counter();
}

#endif //ECATV2_AX58100_H750_UNIVERSAL_TASK_MANAGER_H
