//
// Created by Hang XU on 21/04/2025.
//
#include <cstring>
#include <vector>
#include "cmsis_os.h"
#include "CRC8_CRC16.h"

#include "i2c_processor.h"
#include "task_defs.h"

extern "C" {
#include "ecat_slv.h"
#include "device_conf.h"
#include "fdcan.h"
#include "usart.h"
#include "IOUtils.h"
}

#include "soes_application.h"
#include "utypes.h"
#include "main.h"
#include "task_manager.h"
#include "uart_processor.h"

uint8_t global_args[1024];
uint8_t global_inputs[1024];
uint8_t global_outputs[1024];
_Objects Obj;
extern _ESCvar ESCvar;

void prepare_inputs() {
    memcpy(&global_inputs, (uint8_t *) Obj.master2slave, 80);
}

void prepare_outputs() {
    memcpy((uint8_t *) Obj.slave2master, &global_outputs, 80);
}

void ESC_interrupt_enable(uint32_t mask) {
    if (ESCREG_ALEVENT_DC_SYNC0 & mask) {
        mask &= ~ESCREG_ALEVENT_DC_SYNC0;
    }
    if (ESCREG_ALEVENT_DC_SYNC1 & mask) {
        //    mask &= ~ESCREG_ALEVENT_DC_SYNC1;
    }
    if (ESCREG_ALEVENT_DC_LATCH & mask) {
        // mask &= ~ESCREG_ALEVENT_DC_LATCH;
    }

    ESC_ALeventmaskwrite(ESC_ALeventmaskread() | mask);
}

/** ESC interrupt disable function by the Slave stack in IRQ mode.
 *
 * @param[in]   mask     = interrupts to disable
 */
void ESC_interrupt_disable(uint32_t mask) {
    if (ESCREG_ALEVENT_DC_SYNC0 & mask) {
        mask &= ~ESCREG_ALEVENT_DC_SYNC0;
    }
    if (ESCREG_ALEVENT_DC_SYNC1 & mask) {
        //    mask &= ~ESCREG_ALEVENT_DC_SYNC1;
    }
    if (ESCREG_ALEVENT_DC_LATCH & mask) {
        //    mask &= ~ESCREG_ALEVENT_DC_LATCH;
    }

    ESC_ALeventmaskwrite(~mask & ESC_ALeventmaskread());
}

uint8_t inited = 0;
extern std::vector<CustomRunnable *> task_list;
extern std::vector<CanRunnable *> can_list;
extern std::vector<runnable_conf *> run_confs;
extern std::vector<UartRunnable *> uart_list;
extern std::vector<I2CRunnable *> i2c_list;

void pre_state_change(uint8_t *as, uint8_t *an);

esc_cfg_t _config = {
    .user_arg = NULL,
    .use_interrupt = 1,
    .watchdog_cnt = INT32_MAX,
    .set_defaults_hook = NULL,
    .pre_state_change_hook = pre_state_change,
    .post_state_change_hook = NULL,
    .application_hook = NULL,
    .safeoutput_override = NULL,
    .pre_object_download_hook = NULL,
    .post_object_download_hook = NULL,
    .rxpdo_override = NULL,
    .txpdo_override = NULL,
    .esc_hw_interrupt_enable = ESC_interrupt_enable,
    .esc_hw_interrupt_disable = ESC_interrupt_disable,
    .esc_hw_eep_handler = NULL,
    .esc_check_dc_handler = NULL,
};


uint32_t last_cycle_time = 0;
uint16_t arg_recv_idx = 0;
uint32_t task_load_time = 0;
uint8_t pdi_irq_flag = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    last_cycle_time = HAL_GetTick();
    pdi_irq_flag = 1;
}

extern uint8_t do_task_load;
extern uint8_t task_loaded;

void cb_get_inputs() {
    prepare_inputs();

    if (!inited || Obj.master_status == MASTER_SENDING_ARGUMENTS) {
        if (arg_recv_idx != Obj.sdo_len) {
            Obj.slave_status = SLAVE_INITIALIZING;
        }

        arg_recv_idx = ((uint16_t) *(global_inputs + 1) << 8)
                       | *(global_inputs + (0));
        global_args[arg_recv_idx] = global_inputs[2];
    }

    if (Obj.master_status == MASTER_REQUEST_REBOOT) {
        HAL_NVIC_SystemReset();
    } else if (Obj.master_status == MASTER_READY) {
        if (task_loaded == 0) {
            do_task_load = 1;
        } else {
            task_load_time = HAL_GetTick();
            HAL_GPIO_WritePin(LED_LBOARD_1_GPIO_Port, LED_LBOARD_1_Pin, GPIO_PIN_RESET);
            inited = 1;
            Obj.slave_status = SLAVE_CONFIRM_READY;
        }
    } else if (Obj.master_status > MASTER_READY) {
        task_spin();
    }
}

uint32_t ts, last_ts;
uint32_t ts2, last_ts2;

void cb_set_outputs() {
    ts = HAL_GetTick();
    if (ts - last_ts >= 25) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        last_ts = ts;
    }

    // cuz this module don't require any initializing
    if (Obj.sdo_len == 0 && !inited) {
        Obj.slave_status = SLAVE_READY;
    }

    if (Obj.master_status == MASTER_SENDING_ARGUMENTS && !inited) {
        if (arg_recv_idx != Obj.sdo_len) {
            Obj.slave_status = SLAVE_INITIALIZING;
        }

        *(global_outputs) = arg_recv_idx & 0xFF;
        *(global_outputs + 1) = arg_recv_idx >> 8 & 0xFF;
        global_outputs[2] = global_args[arg_recv_idx];
        if (arg_recv_idx == Obj.sdo_len) {
            Obj.slave_status = SLAVE_READY;
        }
    }

    if (Obj.master_status > MASTER_READY) {
        collect_spin();
        Obj.slave_status = Obj.master_status;
    }
    prepare_outputs();
}

void soes_init() {
    memset(global_args, 0, 1024);
    memset(global_inputs, 0, 1024);
    memset(global_outputs, 0, 1024);
    init_uart_buf();
    init_i2c_buf();
    Obj.slave_status = SLAVE_INITIALIZING;
    ecat_slv_init(&_config);
}

extern uint8_t do_terminate;
extern uint8_t terminated_app_count;

void reset_soem_app() {
    do_terminate = 1;
    for (CustomRunnable *runnable: task_list) {
        runnable->running = 0;
        runnable->exit();
    }
    do_task_load = 0;
    task_loaded = 0;
    task_load_time = 0;
    last_cycle_time = 0;
    memset(global_args, 0, 1024);
    memset(global_inputs, 0, 1024);
    memset(global_outputs, 0, 1024);
    init_uart_buf();
    init_i2c_buf();
    task_list.clear();
    can_list.clear();
    uart_list.clear();
    i2c_list.clear();
    run_confs.clear();
    Obj.master_status = 0;
    Obj.slave_status = SLAVE_INITIALIZING;
    HAL_GPIO_WritePin(LED_LBOARD_1_GPIO_Port, LED_LBOARD_1_Pin, GPIO_PIN_SET);
    inited = 0;
    do_terminate = 0;
}

void pre_state_change(uint8_t *as, uint8_t *an) {
    uint8_t target = (*as >> 4) & 0x0F;
    if (target == ESCinit) {
        reset_soem_app();
    }
}

void soes_task() {

    // vTaskDelay(300);
    // int offset = 0;
    // uint8_t tempbuf[255];
    // memset(tempbuf, 0, sizeof(tempbuf));
    // write_uint16(1, tempbuf, &offset);
    // write_uint16(0x01, tempbuf, &offset);
    // write_uint16(0x00, tempbuf, &offset);
    // write_uint8(1, tempbuf, &offset);
    // write_uint8(3, tempbuf, &offset);
    // offset = 0;
    // App_DMMotor *app = new App_DMMotor(tempbuf, &offset);
    // task_list.push_back(app);
    // can_list.push_back(app);
    // memset(tempbuf, 0, sizeof(tempbuf));
    // uint32_t last_ts = HAL_GetTick();
    // while (1) {
    //
    //     if (HAL_GetTick() - last_ts > 1000) {
    //         if (tempbuf[0] == 0 ) {
    //             tempbuf[0] = 1;
    //         } else {
    //             tempbuf[0] = 0;
    //         }
    //         last_ts = HAL_GetTick();
    //     }
    //
    //     offset = 0;
    //     app->collect_inputs(tempbuf, &offset);
    //     app->run_task();
    //     vTaskDelay(1);
    // }
    //
    // return;


    vTaskDelay(300);
    soes_init();

    while (true) {
        ecat_slv();

        // waiting state
        if (inited == 0) {
            ts2 = HAL_GetTick();
            if (ts2 - last_ts2 >= 250) {
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                last_ts2 = ts2;
            }
        }
    }
}
