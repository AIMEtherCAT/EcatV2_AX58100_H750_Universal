//
// Created by Hang XU on 21/04/2025.
//

#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <stm32h7xx_hal.h>


#ifdef __cplusplus
extern "C" {
#endif

void soes_task_loader();

void task_spin();

void collect_spin();

void task_load();

#ifdef __cplusplus
}
#endif

#endif //TASK_MANAGER_H