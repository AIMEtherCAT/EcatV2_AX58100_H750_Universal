//
// Created by Xiuqi Wang on 2026/3/23.
//
#include "buffer_utils.hpp"
#include "main.h"
#include "peripheral_utils.hpp"
#include "task_defs.hpp"

namespace aim::ecat::task::vt13_rc
{
    VT13_RC::VT13_RC(buffer::Buffer* /* buffer */) : UartRunnable(false, TaskType::VT13_RC)
    {
        init_peripheral(peripheral::Type::PERIPHERAL_USART1);

        get_peripheral<peripheral::UartPeripheral>()->receive_by_dma(39);
    }

    void VT13_RC::write_to_master(buffer::Buffer* slave_to_master_buf)
    {
        uint8_t report_buf[17] = {};
        buf_.read(report_buf, 17);
        slave_to_master_buf->write(report_buf, 17);
        slave_to_master_buf->write_uint8(buffer::EndianType::LITTLE, HAL_GetTick() - last_receive_time_.get() <= 100);
    }

    void VT13_RC::uart_recv(const uint16_t size)
    {
        if (const auto pUart1Buffer = get_peripheral<peripheral::UartPeripheral>()->recv_buf_->get_buf_pointer<
                uint8_t>();
            size == 21 && pUart1Buffer[0] == 0xA9 && pUart1Buffer[1] == 0x53)
        {
            if (algorithm::crc16::verify_CRC16_check_sum(pUart1Buffer, 21))
            {
                uint8_t recv_buf[21] = {};
                get_peripheral<peripheral::UartPeripheral>()->recv_buf_->raw_read(recv_buf, 21);
                if (const uint16_t channel = (recv_buf[2] | recv_buf[3] << 8) & 0x07ff; channel <
                    VT13_RC_CHANNAL_ERROR_VALUE)
                {
                    buf_.write(recv_buf + 2, 17);
                    last_receive_time_.set_current();
                }
            }
        }

        get_peripheral<peripheral::UartPeripheral>()->receive_by_dma(39);
    }

    void VT13_RC::uart_err()
    {
        get_peripheral<peripheral::UartPeripheral>()->receive_by_dma(39);
    }

    void VT13_RC::exit()
    {
        get_peripheral()->deinit();
    }
}
