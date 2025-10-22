//
// Created by Hang XU on 21/10/2025.
//

#ifndef ECATV2_AX58100_H750_UNIVERSAL_SOES_APPLICATION_H
#define ECATV2_AX58100_H750_UNIVERSAL_SOES_APPLICATION_H

namespace aim::ecat::application {
    using utils::ThreadSafeFlag;

    void init_soes_env();

    void do_terminate();

    ThreadSafeFlag *get_is_task_loaded();

    ThreadSafeFlag *get_is_task_ready_to_load();

    ThreadSafeFlag *get_is_slave_ready();
}

#endif //ECATV2_AX58100_H750_UNIVERSAL_SOES_APPLICATION_H
