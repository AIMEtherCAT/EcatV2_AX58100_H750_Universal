//
// Created by Hang XU on 21/10/2025.
//
#include "buffer_manager.hpp"

#include <map>
#include <memory>

#include "settings.h"

namespace aim::io::buffer {

    DMA_BUFFER static uint32_t dshot1_motor1_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
    DMA_BUFFER static uint32_t dshot1_motor2_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
    DMA_BUFFER static uint32_t dshot1_motor3_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
    DMA_BUFFER static uint32_t dshot1_motor4_dmabuffer[DSHOT_DMA_BUFFER_SIZE];

    DMA_BUFFER static uint32_t dshot2_motor1_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
    DMA_BUFFER static uint32_t dshot2_motor2_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
    DMA_BUFFER static uint32_t dshot2_motor3_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
    DMA_BUFFER static uint32_t dshot2_motor4_dmabuffer[DSHOT_DMA_BUFFER_SIZE];

    DMA_BUFFER static uint8_t i2c3_recv_buf[512];
    DMA_BUFFER static uint8_t i2c3_send_buf[512];

    DMA_BUFFER static uint8_t uart8_recv_buf[512];
    DMA_BUFFER static uint8_t uart8_send_buf[512];

    DMA_BUFFER static uint8_t uart4_recv_buf[512];
    DMA_BUFFER static uint8_t uart4_send_buf[512];

    DMA_BUFFER static uint8_t usart1_recv_buf[512];
    DMA_BUFFER static uint8_t usart1_send_buf[512];

    DMA_BUFFER static uint16_t adc1_recv_buf[2];

    static uint8_t ecat_args_buf[1024];
    static uint8_t ecat_slave_to_master_buf[80];
    static uint8_t ecat_master_to_slave_buf[80];

    static std::map<Type, std::unique_ptr<Buffer>> instances;

    void init_buffer_manager() {
        if (!instances.empty()) return;

        instances[Type::DSHOT1_MOTOR1] = std::make_unique<Buffer>(dshot1_motor1_dmabuffer, sizeof(dshot1_motor1_dmabuffer));
        instances[Type::DSHOT1_MOTOR2] = std::make_unique<Buffer>(dshot1_motor2_dmabuffer, sizeof(dshot1_motor2_dmabuffer));
        instances[Type::DSHOT1_MOTOR3] = std::make_unique<Buffer>(dshot1_motor3_dmabuffer, sizeof(dshot1_motor3_dmabuffer));
        instances[Type::DSHOT1_MOTOR4] = std::make_unique<Buffer>(dshot1_motor4_dmabuffer, sizeof(dshot1_motor4_dmabuffer));

        instances[Type::DSHOT2_MOTOR1] = std::make_unique<Buffer>(dshot2_motor1_dmabuffer, sizeof(dshot2_motor1_dmabuffer));
        instances[Type::DSHOT2_MOTOR2] = std::make_unique<Buffer>(dshot2_motor2_dmabuffer, sizeof(dshot2_motor2_dmabuffer));
        instances[Type::DSHOT2_MOTOR3] = std::make_unique<Buffer>(dshot2_motor3_dmabuffer, sizeof(dshot2_motor3_dmabuffer));
        instances[Type::DSHOT2_MOTOR4] = std::make_unique<Buffer>(dshot2_motor4_dmabuffer, sizeof(dshot2_motor4_dmabuffer));

        instances[Type::I2C3_RECV] = std::make_unique<Buffer>(i2c3_recv_buf, sizeof(i2c3_recv_buf));
        instances[Type::I2C3_SEND] = std::make_unique<Buffer>(i2c3_send_buf, sizeof(i2c3_send_buf));

        instances[Type::UART4_RECV] = std::make_unique<Buffer>(uart4_recv_buf, sizeof(uart4_recv_buf));
        instances[Type::UART4_SEND] = std::make_unique<Buffer>(uart4_send_buf, sizeof(uart4_send_buf));

        instances[Type::UART8_RECV] = std::make_unique<Buffer>(uart8_recv_buf, sizeof(uart8_recv_buf));
        instances[Type::UART8_SEND] = std::make_unique<Buffer>(uart8_send_buf, sizeof(uart8_send_buf));

        instances[Type::USART1_RECV] = std::make_unique<Buffer>(usart1_recv_buf, sizeof(usart1_recv_buf));
        instances[Type::USART1_SEND] = std::make_unique<Buffer>(usart1_send_buf, sizeof(usart1_send_buf));

        instances[Type::ADC1_RECV] = std::make_unique<Buffer>(adc1_recv_buf, sizeof(adc1_recv_buf));

        instances[Type::ECAT_ARGS] = std::make_unique<Buffer>(ecat_args_buf, sizeof(ecat_args_buf));
        instances[Type::ECAT_SLAVE_TO_MASTER] = std::make_unique<Buffer>(ecat_slave_to_master_buf, sizeof(ecat_slave_to_master_buf));
        instances[Type::ECAT_MASTER_TO_SLAVE] = std::make_unique<Buffer>(ecat_master_to_slave_buf, sizeof(ecat_master_to_slave_buf));
    }

    Buffer *get_buffer(const Type type) {
        if (const auto it = instances.find(type); it != instances.end()) {
            return it->second.get();
        }
        return nullptr;
    }
}