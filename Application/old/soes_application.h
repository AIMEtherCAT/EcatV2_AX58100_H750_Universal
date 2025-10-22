//
// Created by Hang XU on 21/04/2025.
//

#ifndef SOES_APPLICATION_H
#define SOES_APPLICATION_H

#define SLAVE_INITIALIZING 1
#define SLAVE_READY 2
#define SLAVE_CONFIRM_READY 3

#define MASTER_REQUEST_REBOOT 1
#define MASTER_SENDING_ARGUMENTS 2
#define MASTER_READY 3

#ifdef __cplusplus
extern "C" {
#endif

void soes_init();

void soes_task();

void test_task();

#ifdef __cplusplus
}
#endif

#endif //SOES_APPLICATION_H