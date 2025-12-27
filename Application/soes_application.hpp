//
// Created by Hang XU on 21/10/2025.
//

#ifndef ECATV2_AX58100_H750_UNIVERSAL_SOES_APPLICATION_H
#define ECATV2_AX58100_H750_UNIVERSAL_SOES_APPLICATION_H

namespace aim::ecat::application {
    using namespace utils::thread_safety;

    void init_soes_env();

    void do_terminate();

    ThreadSafeFlag *get_is_task_loaded();

    ThreadSafeFlag *get_is_task_ready_to_load();

    ThreadSafeFlag *get_is_slave_ready();

    ThreadSafeTimestamp *get_last_master_packet_received();

    constexpr uint8_t SLAVE_INITIALIZING = 1;
    constexpr uint8_t SLAVE_READY = 2;
    constexpr uint8_t SLAVE_CONFIRM_READY = 3;

    constexpr uint8_t MASTER_UNKNOWN = 0;
    constexpr uint8_t MASTER_REQUEST_REBOOT = 1;
    constexpr uint8_t MASTER_SENDING_ARGUMENTS = 2;
    constexpr uint8_t MASTER_READY = 3;
}

#endif //ECATV2_AX58100_H750_UNIVERSAL_SOES_APPLICATION_H
