//
// Created by Hang XU on 21/10/2025.
//
#include "buffer_manager.hpp"
#include "main.h"
#include "peripheral_manager.hpp"
#include "settings.h"
#include "soes_application.h"
#include "utypes.h"

extern "C" {
#include "ecat_slv.h"
}

_Objects Obj;
uint16_t arg_recv_idx = 0;
ThreadSafeFlag is_task_loaded;
ThreadSafeFlag is_task_ready_to_load;
ThreadSafeFlag is_slave_ready;

esc_cfg_t config = {
    .user_arg = nullptr,
    // use free-run currently
    .use_interrupt = 0,
    .watchdog_cnt = INT32_MAX,
    .skip_default_initialization = false,
    .set_defaults_hook = nullptr,
    // ReSharper disable once CppParameterMayBeConstPtrOrRef
    // NOLINTNEXTLINE(*-non-const-parameter)
    .pre_state_change_hook = [](uint8_t *as, uint8_t * /* an */) {
        if (const uint8_t target = (*as >> 4) & 0x0F; target == ESCinit) {
            // initialize_soes();
        }
    },
    .post_state_change_hook = nullptr,
    .application_hook = nullptr,
    .safeoutput_override = nullptr,
    .pre_object_download_hook = nullptr,
    .post_object_download_hook = nullptr,
    .pre_object_upload_hook = nullptr,
    .post_object_upload_hook = nullptr,
    .rxpdo_override = nullptr,
    .txpdo_override = nullptr,
    .esc_hw_interrupt_enable = [](uint32_t mask) {
        if (ESCREG_ALEVENT_DC_SYNC0 & mask) {
            mask &= ~ESCREG_ALEVENT_DC_SYNC0;
        }
        ESC_ALeventmaskwrite(ESC_ALeventmaskread() | mask);
    },
    .esc_hw_interrupt_disable = [](uint32_t mask) {
        if (ESCREG_ALEVENT_DC_SYNC0 & mask) {
            mask &= ~ESCREG_ALEVENT_DC_SYNC0;
        }
        ESC_ALeventmaskwrite(~mask & ESC_ALeventmaskread());
    },
    .esc_hw_eep_handler = nullptr,
    .esc_check_dc_handler = nullptr,
    .get_device_id = nullptr
};

void init_soes_buffers() {
    arg_recv_idx = 0;
    is_task_loaded.clear();
    is_task_ready_to_load.clear();
    is_slave_ready.clear();

    memset(Obj.master2slave, 0, 80);
    memset(Obj.slave2master, 0, 80);
    Obj.sdo_len = 0;
    Obj.master_status = MASTER_UNKNOWN;
    Obj.slave_status = SLAVE_INITIALIZING;

    get_buffer(BufferType::ECAT_ARGS)->reset();
    get_buffer(BufferType::ECAT_SLAVE_TO_MASTER)->reset();
    get_buffer(BufferType::ECAT_MASTER_TO_SLAVE)->reset();
}

void cb_get_inputs() {
    get_buffer(BufferType::ECAT_MASTER_TO_SLAVE)->reset();
    get_buffer(BufferType::ECAT_MASTER_TO_SLAVE)->raw_write(reinterpret_cast<uint8_t *>(Obj.master2slave), 80);

    // no any packet received yet
    if (Obj.master_status == MASTER_UNKNOWN) {
        return;
    }

    // sdo not loaded
    if (Obj.master_status == MASTER_SENDING_ARGUMENTS) {
        // read sdo idx and corresponding content
        arg_recv_idx = get_buffer(BufferType::ECAT_MASTER_TO_SLAVE)->read_uint16();
        // index starting with 1, 0 means no any meaningful content received yet
        if (arg_recv_idx == 0) {
            return;
        }
        get_buffer(BufferType::ECAT_ARGS)->get_buf_pointer<uint8_t>()[arg_recv_idx] = get_buffer(
            BufferType::ECAT_MASTER_TO_SLAVE)->read_uint8();
        return;
    }

    if (Obj.master_status == MASTER_REQUEST_REBOOT) {
        HAL_NVIC_SystemReset();
        return;
    }

    // master claims that they've sent all sdo data
    if (Obj.master_status == MASTER_READY) {
        if (!is_task_loaded.get()) {
            is_task_ready_to_load.set();
        } else {
            // turned on when task loaded
            HAL_GPIO_WritePin(LED_LBOARD_1_GPIO_Port, LED_LBOARD_1_Pin, GPIO_PIN_RESET);
            is_slave_ready.set();
            Obj.slave_status = SLAVE_CONFIRM_READY;
        }
        return;
    }

    // task_spin();
}

void cb_set_outputs() {
    // no any packet received yet
    if (Obj.master_status == MASTER_UNKNOWN) {
        return;
    }

    get_buffer(BufferType::ECAT_SLAVE_TO_MASTER)->reset();

    // slave not ready
    if (!is_slave_ready.get()) {
        // sdo_len != 0
        if (Obj.master_status == MASTER_SENDING_ARGUMENTS) {
            // write back sdo lastest received idx and corresponding content for verification
            get_buffer(BufferType::ECAT_SLAVE_TO_MASTER)->write_uint16(arg_recv_idx);
            get_buffer(BufferType::ECAT_SLAVE_TO_MASTER)->write_uint8(
                get_buffer(BufferType::ECAT_ARGS)->get_buf_pointer<uint8_t>()[arg_recv_idx]);

            if (arg_recv_idx == Obj.sdo_len) {
                Obj.slave_status = SLAVE_READY;
            }
        } else if (Obj.master_status == MASTER_READY) {
            Obj.slave_status = SLAVE_READY;
        }
    } else {
        // collect_spin();
        // this flag is for round-trip latency calculation
        // master -> slave -> master
        Obj.slave_status = Obj.master_status;
    }

    get_buffer(BufferType::ECAT_SLAVE_TO_MASTER)->raw_read(reinterpret_cast<uint8_t *>(Obj.slave2master), 80);
}

// use free-run currently, no irq required
// void HAL_GPIO_EXTI_Callback(uint16_t /* GPIO_Pin */) {
//     pdi_irq_flag = 1;
// }

[[noreturn]] void soes_application() {
    while (true) {
        ecat_slv();
    }
}
