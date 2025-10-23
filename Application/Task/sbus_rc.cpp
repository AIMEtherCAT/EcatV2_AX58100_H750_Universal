#include "buffer_manager.hpp"
#include "peripheral_manager.hpp"
#include "task_defs.hpp"

namespace aim::ecat::task::sbus_rc {
    SBUS_RC::SBUS_RC(buffer::Buffer * /* buffer */) : UartRunnable(false) {
        init_peripheral(peripheral::Type::PERIPHERAL_UART8);

        get_peripheral<peripheral::UartPeripheral>()->receive_by_dma(25);
    }

    void SBUS_RC::write_to_master(buffer::Buffer *slave_to_master_buf) {
        uint8_t report_buf[23] = {};
        buf_.read(report_buf, 23);
        slave_to_master_buf->write(report_buf, 23);
        slave_to_master_buf->write_uint8(HAL_GetTick() - last_receive_time_.get() <= 20);
    }

    void SBUS_RC::uart_recv(const uint16_t size) {
        if (size == 25) {
            if (get_peripheral<peripheral::UartPeripheral>()->_recv_buf->get_buf_pointer<uint8_t>()[0] == 0x0f &&
                get_peripheral<peripheral::UartPeripheral>()->_recv_buf->get_buf_pointer<uint8_t>()[24] == 0x00) {
                last_receive_time_.set_current();
                uint8_t recv_buf[23] = {};

                get_peripheral<peripheral::UartPeripheral>()->_recv_buf->reset_index();
                get_peripheral<peripheral::UartPeripheral>()->_recv_buf->skip(1);
                get_peripheral<peripheral::UartPeripheral>()->_recv_buf->read(recv_buf, 23);
                buf_.write(recv_buf, 23);
            }
        }
        get_peripheral<peripheral::UartPeripheral>()->receive_by_dma(25);
    }

    void SBUS_RC::uart_err() {
        get_peripheral<peripheral::UartPeripheral>()->receive_by_dma(25);
    }
}
