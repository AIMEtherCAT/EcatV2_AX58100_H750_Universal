//
// Created by Hang XU on 21/10/2025.
//
#include "buffer_manager.hpp"
#include "main.h"
#include "peripheral_manager.hpp"
#include "soes_application.hpp"
#include "utypes.h"
#include "task_manager.hpp"

extern "C" {
#include "ecat_slv.h"
#include "c_task_warpper.h"
}

_Objects Obj;
esc_cfg_t config = {
    .user_arg = nullptr,
    .use_interrupt = 1,
    .watchdog_cnt = INT32_MAX,
    .skip_default_initialization = false,
    .set_defaults_hook = nullptr,
    // ReSharper disable once CppParameterMayBeConstPtrOrRef
    // NOLINTNEXTLINE(*-non-const-parameter)
    .pre_state_change_hook = [](uint8_t *as, uint8_t * /* an */) {
        if (const uint8_t target = *as >> 4 & 0x0F; target == ESCinit) {
            HAL_NVIC_SystemReset();
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

namespace aim::ecat::application {
    using namespace io;

    uint16_t arg_recv_idx = 0;
    ThreadSafeFlag is_task_loaded{};
    ThreadSafeFlag is_task_ready_to_load{};
    ThreadSafeFlag is_slave_ready{};
    ThreadSafeFlag pdi_called{};

    ThreadSafeValue<uint8_t> last_frame_idx{};
    ThreadSafeTimestamp last_master_packet_received{};

    ThreadSafeTimestamp *get_last_master_packet_received() {
        return &last_master_packet_received;
    }

    ThreadSafeFlag *get_is_task_loaded() {
        return &is_task_loaded;
    }

    ThreadSafeFlag *get_is_task_ready_to_load() {
        return &is_task_ready_to_load;
    }

    ThreadSafeFlag *get_is_slave_ready() {
        return &is_slave_ready;
    }

    void init_soes_env() {
        arg_recv_idx = 0;
        is_task_loaded.clear();
        is_task_ready_to_load.clear();
        is_slave_ready.clear();

        memset(Obj.master2slave, 0, 80);
        memset(Obj.slave2master, 0, 80);
        Obj.sdo_len = 0;
        Obj.master_status = MASTER_UNKNOWN;
        Obj.slave_status = SLAVE_INITIALIZING;

        buffer::get_buffer(buffer::Type::ECAT_ARGS)->reset();
        buffer::get_buffer(buffer::Type::ECAT_SLAVE_TO_MASTER)->reset();
        buffer::get_buffer(buffer::Type::ECAT_MASTER_TO_SLAVE)->reset();
    }

    // currently deprecated
    // i cant figure out why it sometimes stuck and cannot terminate correctly
    // so this func is now replaced by HAL_NVIC_SystemReset();
    // it works :)
    void do_terminate() {
        uint32_t thread_count = 0;
        for (const std::shared_ptr<task::runnable_conf> &conf: *task::get_run_confs()) {
            conf->runnable->running.clear();
            if (conf->runnable->is_run_task_enabled_) {
                thread_count++;
            }
        }

        while (task::get_terminated_counter()->get() != thread_count) {
            vTaskDelay(1);
        }

        for (const std::shared_ptr<task::runnable_conf> &conf: *task::get_run_confs()) {
            conf->runnable->exit();
        }

        init_soes_env();
        buffer::clear_all_buffers();
        task::get_terminated_counter()->reset();
        task::get_run_confs()->clear();
    }

    void cb_set_outputs_impl() {
        buffer::get_buffer(buffer::Type::ECAT_MASTER_TO_SLAVE)->reset();
        buffer::get_buffer(buffer::Type::ECAT_MASTER_TO_SLAVE)->raw_write(
            reinterpret_cast<uint8_t *>(Obj.master2slave), 80);

        // no any packet received yet
        if (Obj.master_status == MASTER_UNKNOWN) {
            return;
        }

        // sdo not loaded
        if (Obj.master_status == MASTER_SENDING_ARGUMENTS) {
            // read sdo idx and corresponding content
            arg_recv_idx = buffer::get_buffer(buffer::Type::ECAT_MASTER_TO_SLAVE)->read_uint16(
                buffer::EndianType::LITTLE);
            // index starting with 1, 0 means no any meaningful content received yet
            if (arg_recv_idx == 0) {
                return;
            }
            buffer::get_buffer(buffer::Type::ECAT_ARGS)->get_buf_pointer<uint8_t>()[arg_recv_idx] = buffer::get_buffer(
                buffer::Type::ECAT_MASTER_TO_SLAVE)->read_uint8(buffer::EndianType::LITTLE);
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

        for (const std::shared_ptr<task::runnable_conf> &conf: *task::get_run_confs()) {
            conf->runnable->read_from_master(buffer::get_buffer(buffer::Type::ECAT_MASTER_TO_SLAVE));
        }
    }

    void cb_get_inputs_impl() {
        // no any packet received yet
        if (Obj.master_status == MASTER_UNKNOWN) {
            return;
        }

        buffer::get_buffer(buffer::Type::ECAT_SLAVE_TO_MASTER)->reset();

        // slave not ready
        if (!is_slave_ready.get()) {
            // sdo_len != 0
            if (Obj.master_status == MASTER_SENDING_ARGUMENTS) {
                // write back sdo lastest received idx and corresponding content for verification
                buffer::get_buffer(buffer::Type::ECAT_SLAVE_TO_MASTER)->write_uint16(
                    buffer::EndianType::LITTLE, arg_recv_idx);
                buffer::get_buffer(buffer::Type::ECAT_SLAVE_TO_MASTER)->write_uint8(
                    buffer::EndianType::LITTLE,
                    buffer::get_buffer(buffer::Type::ECAT_ARGS)->get_buf_pointer<uint8_t>()[arg_recv_idx]
                );

                if (arg_recv_idx == Obj.sdo_len) {
                    Obj.slave_status = SLAVE_READY;
                }
            } else if (Obj.master_status == MASTER_READY) {
                Obj.slave_status = SLAVE_READY;
            }
        } else {
            for (const std::shared_ptr<task::runnable_conf> &conf: *task::get_run_confs()) {
                conf->runnable->write_to_master(buffer::get_buffer(buffer::Type::ECAT_SLAVE_TO_MASTER));
            }
            // this flag is for round-trip latency calculation
            // master -> slave -> master
            Obj.slave_status = Obj.master_status;
            if (Obj.master_status != last_frame_idx.get()) {
                last_frame_idx.set(Obj.master_status);
                last_master_packet_received.set_current();
            }
        }

        buffer::get_buffer(buffer::Type::ECAT_SLAVE_TO_MASTER)->raw_read(
            reinterpret_cast<uint8_t *>(Obj.slave2master), 80);
    }

    [[noreturn]] void soes_application_impl() {
        while (true) {
            // I don't actually know why
            // but if I add this duplicated free-run func here
            // the overall latency will decrease
            // ???????
            ecat_slv();
            if (pdi_called.get()) {
                DIG_process(DIG_PROCESS_OUTPUTS_FLAG | DIG_PROCESS_APP_HOOK_FLAG |
                            DIG_PROCESS_INPUTS_FLAG);
                pdi_called.clear();
            }
        }
    }
}

// for c warppers
using namespace aim::ecat::application;

void soes_application(const void *) {
    soes_application_impl();
}

void cb_get_inputs() {
    cb_get_inputs_impl();
}

void cb_set_outputs() {
    cb_set_outputs_impl();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    UNUSED(GPIO_Pin);
    pdi_called.set();
}
