// //
// // Created by Xiuqi Wang on 2026/3/27.
// //
// #include "buffer_utils.hpp"
// #include "main.h"
// #include "peripheral_utils.hpp"
// #include "task_defs.hpp"
// #include "crc_utils.hpp"
//
// namespace aim::ecat::task::custom_controller {
//   CUSTOM_CONTROLLER::CUSTOM_CONTROLLER(buffer::Buffer * /* buffer */) : UartRunnable(false, TaskType::CUSTOM_CONTROLLER) {
//     init_peripheral(peripheral::Type::PERIPHERAL_USART1);
//
//     get_peripheral<peripheral::UartPeripheral>()->receive_by_dma(DMA_RECV_LEN);
//   }
//
//   void CUSTOM_CONTROLLER::write_to_master(buffer::Buffer *slave_to_master_buf) {
//     uint8_t report_buf[RC_MSG_PKG_LEN] = {};
//     buf_.read(report_buf, RC_MSG_PKG_LEN);
//     slave_to_master_buf->write(report_buf, RC_MSG_PKG_LEN);
//     slave_to_master_buf->write_uint8(buffer::EndianType::LITTLE, HAL_GetTick() - last_receive_time_.get() <= 20);
//   }
//
//   void CUSTOM_CONTROLLER::uart_recv(const uint16_t size) {
//     auto pUart1Buffer = get_peripheral<peripheral::UartPeripheral>()->recv_buf_->get_buf_pointer<uint8_t>();
//     //VT13 Task
//     if (pUart1Buffer[0] == 0xA9 && pUart1Buffer[1] == 0x53 && size == RC_FULL_PKG_LEN) {
//       if (algorithm::crc16::verify_CRC16_check_sum(pUart1Buffer,RC_FULL_PKG_LEN)) {
//         uint8_t recv_buf[RC_FULL_PKG_LEN] = {};
//         get_peripheral<peripheral::UartPeripheral>()->recv_buf_->raw_read(recv_buf, RC_FULL_PKG_LEN);
//         buf_.write(&recv_buf[2], RC_MSG_PKG_LEN);
//       }
//     }
//     //custom controller
//     if (pUart1Buffer[0] == 0xA5
//       && static_cast<uint16_t>(pUart1Buffer[5] | pUart1Buffer[6]) == IMAGE_TRANS_LINK_SOF
//       && size == CUSTOM_C_PKG_LEN) {
//       if (algorithm::crc16::verify_CRC16_check_sum(pUart1Buffer, CUSTOM_C_PKG_LEN)) {
//         uint8_t recv_buf[CUSTOM_C_PKG_LEN] = {};
//         get_peripheral<peripheral::UartPeripheral>()->recv_buf_->raw_read(recv_buf, CUSTOM_C_PKG_LEN);
//         buf_.write(&recv_buf[7], CUSTOM_C_PKG_LEN);
//       }
//     }
//     get_peripheral<peripheral::UartPeripheral>()->receive_by_dma(DMA_RECV_LEN);
//   }
//
//   void CUSTOM_CONTROLLER::uart_err() {
//     get_peripheral<peripheral::UartPeripheral>()->receive_by_dma(DMA_RECV_LEN);
//   }
//
//   void CUSTOM_CONTROLLER::exit() {
//     get_peripheral()->deinit();
//   }
// }
