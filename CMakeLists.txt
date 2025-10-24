cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME EcatV2_AX58100_H750_Universal)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        "Application/soes_application.cpp"
        "Application/task_manager.cpp"
        "Application/callbacks.cpp"

        "Application/Task/dbus_rc.cpp"
        "Application/Task/dji_motor.cpp"
        "Application/Task/hipnuc_imu_can.cpp"
        "Application/Task/dm_motor.cpp"
        "Application/Task/adc.cpp"
        "Application/Task/dshot.cpp"
        "Application/Task/lk_motor.cpp"
        "Application/Task/pmu_uavcan.cpp"
        "Application/Task/pwm_external.cpp"
        "Application/Task/pwm_onboard.cpp"
        "Application/Task/sbus_rc.cpp"
        "Application/Task/ms5837_30ba.cpp"

        "Utils/pid.cpp"
        "Utils/crc16.cpp"
        "Utils/initializer.cpp"
        "Utils/peripheral_manager.cpp"
        "Utils/io_utils.cpp"
        "Utils/buffer_manager.cpp"

        "soes/device/slave_objectlist.c"
        "soes/hal/esc_hw.c"
        "soes/ecat_slv.c"
        "soes/esc.c"
        "soes/esc_coe.c"
        "soes/esc_eep.c"
        "soes/esc_eoe.c"
        "soes/esc_foe.c"
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths

        Application/
        Bsp/
        Utils/
        soes/
        soes/hal
        soes/device
        soes/include/sys/gcc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
