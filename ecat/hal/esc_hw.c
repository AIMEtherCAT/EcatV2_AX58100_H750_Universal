//
// Created by Hang XU on 21/04/2025.
//
//
// Created by Hang Xu on 21/08/2024.
//
/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/** \file
* \brief
* ESC hardware layer functions.
*
* Function to read and write commands to the ESC. Used to read/write ESC
* registers and memory.
*/

#include <string.h>
#include "esc.h"
#include "cc.h"
#include "main.h"

#define MAX_READ_SIZE   128

#define ESC_CMD_READ    0x02
#define ESC_CMD_READWS  0x03
#define ESC_CMD_WRITE   0x04
#define ESC_CMD_NOP     0x00
#define ESC_TERM        0xff
#define ESC_NEXT        0x00

#define ESCREG_PDI_CONTROL         0x0140
#define ESCREG_ESC_CONFIG          0x0141
#define DC_SYNC_OUT                0x04
#define ESCREG_CYCLIC_UNIT_CONTROL 0x0980
#define SYNC_OUT_UNIT_CONTROL_MASK 0x01
#define SYNC_OUT_ECAT_CONTROL      0x00
#define SYNC_OUT_PDI_CONTROL       0x01
#define ESCREG_SYNC0_CYCLE_TIME    0x09A0
#define ESCREG_SYNC_START_TIME     0x0990

static uint8_t read_termination[MAX_READ_SIZE] = {0};

extern SPI_HandleTypeDef hspi4;

void spi_unselect(void) {
    HAL_GPIO_WritePin(ETHERCAT_NS_GPIO_Port, ETHERCAT_NS_Pin, GPIO_PIN_SET);
}

void spi_select(void) {
    HAL_GPIO_WritePin(ETHERCAT_NS_GPIO_Port, ETHERCAT_NS_Pin, GPIO_PIN_RESET);
}

static void esc_address(const uint16_t address, const uint8_t command) {
    /* Device is selected already.
     * We use 2 bytes addressing.
     */
    uint8_t data[2];

    /* address 12:5 */
    data[0] = address >> 5;
    /* address 4:0 and cmd 2:0 */
    data[1] = (address & 0x1F) << 3 | command;

    /* Write (and read AL interrupt register) */
    HAL_SPI_TransmitReceive(&hspi4, data, (uint8_t *) &ESCvar.ALevent, sizeof(data), 1000);
    ESCvar.ALevent = etohs(ESCvar.ALevent);
}

/** ESC read function used by the Slave stack.
 *
 * @param[in]   address     = address of ESC register to read
 * @param[out]  buf         = pointer to buffer to read in
 * @param[in]   len         = number of bytes to read
 */
void ESC_read(const uint16_t address, void *buf, const uint16_t len) {
    if (len > MAX_READ_SIZE) { return; }

    /* Select device. */
    spi_select();

    /* Write address and command to device. */
    esc_address(address, ESC_CMD_READ);

    /* Here we want to read data and keep MOSI low (0x00) during
     * all bytes except the last one where we want to pull it high (0xFF).
     * Read (and write termination bytes).
     */
    HAL_SPI_TransmitReceive(&hspi4, read_termination +
                                    (MAX_READ_SIZE - len), buf, len, 0xffff);

    /* Un-select device. */
    spi_unselect();
}

/** ESC write function used by the Slave stack.
 *
 * @param[in]   address     = address of ESC register to write
 * @param[out]  buf         = pointer to buffer to write from
 * @param[in]   len         = number of bytes to write
 */
void ESC_write(uint16_t address, void *buf, uint16_t len) {
    /* Select device. */
    spi_select();
    /* Write address and command to device. */
    esc_address(address, ESC_CMD_WRITE);
    /* Write data. */
    HAL_SPI_Transmit(&hspi4, buf, len, 0xffff);

    /* Un-select device. */
    spi_unselect();
}


void ESC_init(const esc_cfg_t *cfg) {
    UNUSED(cfg);
    read_termination[MAX_READ_SIZE - 1] = 0xFF;
}
