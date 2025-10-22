//
// Created by Hang XU on 21/10/2025.
//
#include "buffer_manager.hpp"

#include <map>
#include <memory>

#include "settings.h"

DMA_BUFFER uint32_t dshot1_motor1_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
DMA_BUFFER uint32_t dshot1_motor2_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
DMA_BUFFER uint32_t dshot1_motor3_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
DMA_BUFFER uint32_t dshot1_motor4_dmabuffer[DSHOT_DMA_BUFFER_SIZE];

DMA_BUFFER uint32_t dshot2_motor1_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
DMA_BUFFER uint32_t dshot2_motor2_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
DMA_BUFFER uint32_t dshot2_motor3_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
DMA_BUFFER uint32_t dshot2_motor4_dmabuffer[DSHOT_DMA_BUFFER_SIZE];

DMA_BUFFER uint8_t i2c3_recv_buf[512];
DMA_BUFFER uint8_t i2c3_send_buf[512];

DMA_BUFFER uint8_t uart8_recv_buf[512];
DMA_BUFFER uint8_t uart8_send_buf[512];

DMA_BUFFER uint8_t uart4_recv_buf[512];
DMA_BUFFER uint8_t uart4_send_buf[512];

DMA_BUFFER uint8_t usart1_recv_buf[512];
DMA_BUFFER uint8_t usart1_send_buf[512];

DMA_BUFFER uint16_t adc1_recv_buf[2];

uint8_t ecat_args_buf[1024];
uint8_t ecat_slave_to_master_buf[80];
uint8_t ecat_master_to_slave_buf[80];

static std::map<BufferType, std::unique_ptr<Buffer>> instances;

void init_buffer_manager() {
    if (!instances.empty()) return;

    instances[BufferType::DSHOT1_MOTOR1] = std::make_unique<Buffer>(dshot1_motor1_dmabuffer, sizeof(dshot1_motor1_dmabuffer));
    instances[BufferType::DSHOT1_MOTOR2] = std::make_unique<Buffer>(dshot1_motor2_dmabuffer, sizeof(dshot1_motor2_dmabuffer));
    instances[BufferType::DSHOT1_MOTOR3] = std::make_unique<Buffer>(dshot1_motor3_dmabuffer, sizeof(dshot1_motor3_dmabuffer));
    instances[BufferType::DSHOT1_MOTOR4] = std::make_unique<Buffer>(dshot1_motor4_dmabuffer, sizeof(dshot1_motor4_dmabuffer));

    instances[BufferType::DSHOT2_MOTOR1] = std::make_unique<Buffer>(dshot2_motor1_dmabuffer, sizeof(dshot2_motor1_dmabuffer));
    instances[BufferType::DSHOT2_MOTOR2] = std::make_unique<Buffer>(dshot2_motor2_dmabuffer, sizeof(dshot2_motor2_dmabuffer));
    instances[BufferType::DSHOT2_MOTOR3] = std::make_unique<Buffer>(dshot2_motor3_dmabuffer, sizeof(dshot2_motor3_dmabuffer));
    instances[BufferType::DSHOT2_MOTOR4] = std::make_unique<Buffer>(dshot2_motor4_dmabuffer, sizeof(dshot2_motor4_dmabuffer));

    instances[BufferType::I2C3_RECV] = std::make_unique<Buffer>(i2c3_recv_buf, sizeof(i2c3_recv_buf));
    instances[BufferType::I2C3_SEND] = std::make_unique<Buffer>(i2c3_send_buf, sizeof(i2c3_send_buf));

    instances[BufferType::UART4_RECV] = std::make_unique<Buffer>(uart4_recv_buf, sizeof(uart4_recv_buf));
    instances[BufferType::UART4_SEND] = std::make_unique<Buffer>(uart4_send_buf, sizeof(uart4_send_buf));

    instances[BufferType::UART8_RECV] = std::make_unique<Buffer>(uart8_recv_buf, sizeof(uart8_recv_buf));
    instances[BufferType::UART8_SEND] = std::make_unique<Buffer>(uart8_send_buf, sizeof(uart8_send_buf));

    instances[BufferType::USART1_RECV] = std::make_unique<Buffer>(usart1_recv_buf, sizeof(usart1_recv_buf));
    instances[BufferType::USART1_SEND] = std::make_unique<Buffer>(usart1_send_buf, sizeof(usart1_send_buf));

    instances[BufferType::ADC1_RECV] = std::make_unique<Buffer>(adc1_recv_buf, sizeof(adc1_recv_buf));

    instances[BufferType::ECAT_ARGS] = std::make_unique<Buffer>(ecat_args_buf, sizeof(ecat_args_buf));
    instances[BufferType::ECAT_SLAVE_TO_MASTER] = std::make_unique<Buffer>(ecat_slave_to_master_buf, sizeof(ecat_slave_to_master_buf));
    instances[BufferType::ECAT_MASTER_TO_SLAVE] = std::make_unique<Buffer>(ecat_master_to_slave_buf, sizeof(ecat_master_to_slave_buf));
}

Buffer *get_buffer(const BufferType type) {
    if (const auto it = instances.find(type); it != instances.end()) {
        return it->second.get();
    }
    return nullptr;
}
