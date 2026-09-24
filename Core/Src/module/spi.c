/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * SPI bridge module implementation
 *
 * Copyright (C) 2026 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <stdint.h>
#include <stdbool.h>
#include "stm32c0xx_hal.h"
#include "stm32c0xx_ll_bus.h"
#include "stm32c0xx_ll_spi.h"
#include "module/spi.h"
#include "module/gpio.h"

// Register offsets inside the SPI module window. The transfer header
// (TX_LEN, RX_LEN, EXEC) is placed right before the buffer so a single I2C
// write can carry both the header and the data to send.
#define SPI_REG_CTRL        0x000
#define SPI_REG_STATUS      0x001
#define SPI_REG_BUF_SIZE_L  0x002
#define SPI_REG_BUF_SIZE_H  0x003
#define SPI_REG_TX_LEN_L    0x0FB
#define SPI_REG_TX_LEN_H    0x0FC
#define SPI_REG_RX_LEN_L    0x0FD
#define SPI_REG_RX_LEN_H    0x0FE
#define SPI_REG_EXEC        0x0FF
#define SPI_BUF_START       0x100

#define SPI_BUF_SIZE 512

#define CTRL_ENABLE     (1 << 0)
#define CTRL_CPOL       (1 << 1)
#define CTRL_CPHA       (1 << 2)
#define CTRL_BR_SHIFT   3
#define CTRL_BR_MASK    (0x07 << CTRL_BR_SHIFT)
#define CTRL_MASK       (CTRL_ENABLE | CTRL_CPOL | CTRL_CPHA | CTRL_BR_MASK)

#define STATUS_CS_ASSERTED  (1 << 0)
#define STATUS_ERROR        (1 << 1)

#define EXEC_TRANSFER   (1 << 0)
#define EXEC_KEEP_CS    (1 << 1)

// SPI1 pins, and the GPIO module indexes that are reserved while the bridge is enabled
#define SPI_CS_PIN      GPIO_PIN_4   // GPIO_4
#define SPI_SCK_PIN     GPIO_PIN_5   // GPIO_5
#define SPI_MISO_PIN    GPIO_PIN_11  // GPIO_9
#define SPI_MOSI_PIN    GPIO_PIN_12  // GPIO_10
#define SPI_GPIO_MASK   ((1 << 4) | (1 << 5) | (1 << 9) | (1 << 10))

// Upper bound for busy-waiting on a flag, well above one byte at the slowest baud rate
#define SPI_TIMEOUT_LOOPS 100000

static uint8_t ctrl = 0;
static uint8_t status = 0;
static uint16_t tx_len = 0;
static uint16_t rx_len = 0;
static uint8_t exec = 0;
static bool exec_pending = false;
static uint8_t buffer[SPI_BUF_SIZE];

static void cs_set(bool asserted)
{
    if (asserted) {
        HAL_GPIO_WritePin(GPIOA, SPI_CS_PIN, GPIO_PIN_RESET);
        status |= STATUS_CS_ASSERTED;
    } else {
        HAL_GPIO_WritePin(GPIOA, SPI_CS_PIN, GPIO_PIN_SET);
        status &= ~STATUS_CS_ASSERTED;
    }
}

static void configure_peripheral(void)
{
    LL_SPI_Disable(SPI1);
    LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);
    LL_SPI_SetTransferDirection(SPI1, LL_SPI_FULL_DUPLEX);
    LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);
    LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
    LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
    LL_SPI_SetClockPolarity(SPI1, (ctrl & CTRL_CPOL) ? LL_SPI_POLARITY_HIGH : LL_SPI_POLARITY_LOW);
    LL_SPI_SetClockPhase(SPI1, (ctrl & CTRL_CPHA) ? LL_SPI_PHASE_2EDGE : LL_SPI_PHASE_1EDGE);
    LL_SPI_SetBaudRatePrescaler(SPI1, ((ctrl & CTRL_BR_MASK) >> CTRL_BR_SHIFT) << SPI_CR1_BR_Pos);
    LL_SPI_Enable(SPI1);
}

static void pins_claim(void)
{
    gpio_module_reserve(SPI_GPIO_MASK);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Drive CS high before switching the pin to output to avoid a glitch
    HAL_GPIO_WritePin(GPIOA, SPI_CS_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = SPI_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    status &= ~STATUS_CS_ASSERTED;

    GPIO_InitStruct.Pin = SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void pins_release(void)
{
    LL_SPI_Disable(SPI1);
    status &= ~STATUS_CS_ASSERTED;

    // Hand the pins back to the GPIO module, which restores its configuration (inputs by default)
    gpio_module_release(SPI_GPIO_MASK);
}

static bool transfer_byte(uint8_t tx, uint8_t* rx)
{
    uint32_t loops = SPI_TIMEOUT_LOOPS;
    while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {
        if (--loops == 0) {
            return false;
        }
    }
    LL_SPI_TransmitData8(SPI1, tx);

    loops = SPI_TIMEOUT_LOOPS;
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1)) {
        if (--loops == 0) {
            return false;
        }
    }
    *rx = LL_SPI_ReceiveData8(SPI1);

    return true;
}

static void ctrl_write(uint8_t value)
{
    bool was_enabled = (ctrl & CTRL_ENABLE) != 0;
    ctrl = value & CTRL_MASK;

    if (ctrl & CTRL_ENABLE) {
        configure_peripheral();
        if (!was_enabled) {
            pins_claim();
        }
    } else if (was_enabled) {
        pins_release();
    }
}

void spi_module_init(void)
{
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);
}

uint8_t spi_module_read(uint16_t address)
{
    if (address >= SPI_BUF_START && address < SPI_BUF_START + SPI_BUF_SIZE) {
        return buffer[address - SPI_BUF_START];
    }

    switch (address) {
        case SPI_REG_CTRL:
            return ctrl;
        case SPI_REG_STATUS:
            return status;
        case SPI_REG_BUF_SIZE_L:
            return (uint8_t)(SPI_BUF_SIZE & 0xFF);
        case SPI_REG_BUF_SIZE_H:
            return (uint8_t)((SPI_BUF_SIZE >> 8) & 0xFF);
        case SPI_REG_TX_LEN_L:
            return (uint8_t)(tx_len & 0xFF);
        case SPI_REG_TX_LEN_H:
            return (uint8_t)((tx_len >> 8) & 0xFF);
        case SPI_REG_RX_LEN_L:
            return (uint8_t)(rx_len & 0xFF);
        case SPI_REG_RX_LEN_H:
            return (uint8_t)((rx_len >> 8) & 0xFF);
        case SPI_REG_EXEC:
            return exec;
        default:
            return 0x00;
    }
}

void spi_module_write(uint16_t address, uint8_t value)
{
    if (address >= SPI_BUF_START && address < SPI_BUF_START + SPI_BUF_SIZE) {
        buffer[address - SPI_BUF_START] = value;
        return;
    }

    switch (address) {
        case SPI_REG_CTRL:
            ctrl_write(value);
            break;
        case SPI_REG_TX_LEN_L:
            tx_len = (tx_len & 0xFF00) | value;
            break;
        case SPI_REG_TX_LEN_H:
            tx_len = (tx_len & 0x00FF) | ((uint16_t)value << 8);
            break;
        case SPI_REG_RX_LEN_L:
            rx_len = (rx_len & 0xFF00) | value;
            break;
        case SPI_REG_RX_LEN_H:
            rx_len = (rx_len & 0x00FF) | ((uint16_t)value << 8);
            break;
        case SPI_REG_EXEC:
            // Executed on STOP, after the data following the header has been received
            exec = value;
            exec_pending = true;
            break;
        default:
            break;
    }
}

// Runs in the I2C ISR. If the host starts a new transaction before we are done
// the I2C peripheral stretches SCL on the address match until we return.
void spi_module_on_stop(void)
{
    if (!exec_pending) {
        return;
    }
    exec_pending = false;
    status &= ~STATUS_ERROR;

    if (!(ctrl & CTRL_ENABLE)) {
        status |= STATUS_ERROR;
        return;
    }

    if (exec & EXEC_TRANSFER) {
        uint32_t total = (uint32_t)tx_len + rx_len;
        if (total > SPI_BUF_SIZE) {
            status |= STATUS_ERROR;
            cs_set(false);
            return;
        }

        cs_set(true);

        // Drop anything left in the RX FIFO
        while (LL_SPI_IsActiveFlag_RXNE(SPI1)) {
            (void)LL_SPI_ReceiveData8(SPI1);
        }

        // MISO byte i always ends up in buffer[i], bytes after tx_len are clocked out as 0xFF
        for (uint32_t i = 0; i < total; i++) {
            uint8_t tx = (i < tx_len) ? buffer[i] : 0xFF;
            if (!transfer_byte(tx, &buffer[i])) {
                status |= STATUS_ERROR;
                cs_set(false);
                return;
            }
        }
    }

    cs_set((exec & EXEC_KEEP_CS) != 0);
}
