set(PROJECT_SOURCES
        Application/soes_application.cpp
        Application/task_manager.cpp
        Application/callbacks.cpp

        Application/Task/dbus_rc.cpp
        Application/Task/dji_motor.cpp
        Application/Task/hipnuc_imu_can.cpp
        Application/Task/dm_motor.cpp
        Application/Task/adc.cpp
        Application/Task/dshot.cpp
        Application/Task/lk_motor.cpp
        Application/Task/pmu_uavcan.cpp
        Application/Task/pwm_external.cpp
        Application/Task/pwm_onboard.cpp
        Application/Task/sbus_rc.cpp
        Application/Task/ms5837_30ba.cpp
        Application/Task/super_cap.cpp
        Application/Task/vt13_rc.cpp

        Utils/pid_utils.cpp
        Utils/crc_utils.cpp
        Application/initializer.cpp
        Utils/peripheral_utils.cpp
        Utils/io_utils.cpp
        Utils/buffer_utils.cpp

        ecat/device/slave_objectlist.c
        ecat/hal/esc_hw.c
        ecat/soes/soes/ecat_slv.c
        ecat/soes/soes/esc.c
        ecat/soes/soes/esc_coe.c
        ecat/soes/soes/esc_eep.c
        ecat/soes/soes/esc_eoe.c
        ecat/soes/soes/esc_foe.c
)

set(PROJECT_INCLUDES
        Application/
        Bsp/
        Utils/
        ecat/soes/soes
        ecat/hal
        ecat/device
        ecat/soes/soes/include/sys/gcc)