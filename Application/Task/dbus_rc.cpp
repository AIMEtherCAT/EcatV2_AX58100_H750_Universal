#include "buffer_manager.hpp"
#include "peripheral_manager.hpp"
#include "task_defs.h"
#include "settings.h"

Task_DBUS_RC::Task_DBUS_RC(Buffer /* arguments */) {
    _peripheral = ::get_peripheral(PeripheralType::PERIPHERAL_UART8);

    get_peripheral()->init();
    get_peripheral<UartPeripheral>()->receive_by_dma(18);
}

void Task_DBUS_RC::write_to_master(Buffer *slave_to_master_buf) {
    _buf[18] = HAL_GetTick() - _last_receive_time <= 20;
    slave_to_master_buf->write(_buf, 19);
}

void Task_DBUS_RC::uart_recv(const uint16_t size) {
    if (size == 18) {
        if (const uint16_t channel = (get_peripheral<UartPeripheral>()->_recv_buf->get_buf_pointer<uint8_t>()[0] |
                                      get_peripheral<UartPeripheral>()->_recv_buf->get_buf_pointer<uint8_t>()[1] <<
                                      8) & 0x07ff; channel > DBUS_RC_CHANNAL_ERROR_VALUE) {
            _last_receive_time = HAL_GetTick();
            get_peripheral<UartPeripheral>()->_recv_buf->raw_read(_buf, 18);
        }
    }
    get_peripheral<UartPeripheral>()->receive_by_dma(18);
}

void Task_DBUS_RC::uart_err() {
    get_peripheral<UartPeripheral>()->receive_by_dma(18);
}

void Task_DBUS_RC::exit() {
    get_peripheral()->deinit();
}

