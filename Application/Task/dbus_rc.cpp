#include "buffer_manager.hpp"
#include "peripheral_manager.hpp"
#include "task_defs.hpp"

namespace aim::ecat::task::dbus_rc {
    DBUS_RC::DBUS_RC(buffer::Buffer * /* buffer */) {
        peripheral_ = peripheral::get_peripheral(peripheral::Type::PERIPHERAL_UART8);

        get_peripheral()->init();
        get_peripheral<peripheral::UartPeripheral>()->receive_by_dma(18);
    }

    void DBUS_RC::write_to_master(buffer::Buffer *slave_to_master_buf) {
        uint8_t report_buf[18] = {};
        buf_.read(report_buf, 18);
        slave_to_master_buf->write(report_buf, 18);
        slave_to_master_buf->write_uint8(HAL_GetTick() - last_receive_time_.get() <= 20);
    }

    void DBUS_RC::uart_recv(const uint16_t size) {
        if (size == 18) {
            if (const uint16_t channel = (get_peripheral<peripheral::UartPeripheral>()->_recv_buf->get_buf_pointer<
                                              uint8_t>()[0] |
                                          get_peripheral<peripheral::UartPeripheral>()->_recv_buf->get_buf_pointer<
                                              uint8_t>()[1] <<
                                          8) & 0x07ff; channel < DBUS_RC_CHANNAL_ERROR_VALUE) {
                last_receive_time_.set_current();
                uint8_t recv_buf[18] = {};
                get_peripheral<peripheral::UartPeripheral>()->_recv_buf->raw_read(recv_buf, 18);
                buf_.write(recv_buf, 18);
            }
        }
        get_peripheral<peripheral::UartPeripheral>()->receive_by_dma(18);
    }

    void DBUS_RC::uart_err() {
        get_peripheral<peripheral::UartPeripheral>()->receive_by_dma(18);
    }
}
