/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * GPIO module implementation
 *
 * Copyright (C) 2025 Bitcraze AB
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
#include "stm32c0xx_ll_gpio.h"
#include "module/gpio.h"

typedef struct {
    GPIO_TypeDef * port;
    uint16_t pin;
} gpio_entry_t;

static gpio_entry_t gpio_map[] = {
    {GPIOA, GPIO_PIN_2}, // GPIOA2
    {GPIOA, GPIO_PIN_3}, // GPIOA3
    {GPIOA, GPIO_PIN_4}, // GPIOA4
    {GPIOA, GPIO_PIN_5}, // GPIOA5
    {GPIOA, GPIO_PIN_6}, // GPIOA6
    {GPIOA, GPIO_PIN_7}, // GPIOA7
    {GPIOA, GPIO_PIN_8}, // GPIOA8
    {GPIOA, GPIO_PIN_11}, // GPIOA11
    {GPIOA, GPIO_PIN_12}, // GPIOA12
    {GPIOC, GPIO_PIN_14}, // GPIOC14
    {GPIOC, GPIO_PIN_15} // GPIOC15
};

#define GPIO_DIR_START ((uint16_t)0x00)
#define GPIO_DIR_END 0x01
#define GPIO_DATA_START 0x02
#define GPIO_DATA_END 0x03

// Cache for GPIO direction (1=output, 0=input)
static uint16_t gpio_direction_cache = 0x0000; // Initialize all as inputs

void gpio_module_init(void)
{
    // GPIO clocks are already enabled in main.c
    // Initialize all GPIOs as inputs
    gpio_direction_cache = 0x0000;

    // Configure all pins as inputs
    for (uint8_t i = 0; i < sizeof(gpio_map) / sizeof(gpio_entry_t); i++) {
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = gpio_map[i].pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(gpio_map[i].port, &GPIO_InitStruct);
    }
}

uint8_t gpio_module_read(uint16_t address)
{
    address &= 0xFF;

    if (address <= GPIO_DIR_END)
    {
        // Read GPIO direction from cache
        uint8_t gpio_addr = (uint8_t)(address - GPIO_DIR_START);
        if (gpio_addr == 0) {
            return (uint8_t)(gpio_direction_cache & 0xFF); // Low byte
        } else {
            return (uint8_t)((gpio_direction_cache >> 8) & 0xFF); // High byte
        }
    }

    if (address >= GPIO_DATA_START && address <= GPIO_DATA_END)
    {
        // Read GPIO data
        uint8_t gpio_addr = (uint8_t)(address - GPIO_DATA_START);
        uint8_t gpio_index = gpio_addr == 0 ? 0 : 8; // Start at 8 if it's the high byte
        uint8_t result = 0;

        for (uint8_t i = 0; i < 8; i++) {
            if (gpio_index < sizeof(gpio_map) / sizeof(gpio_entry_t))
            {
                if (HAL_GPIO_ReadPin(gpio_map[gpio_index].port, gpio_map[gpio_index].pin) == GPIO_PIN_SET)
                {
                    result |= (1 << i);
                }
            }
            gpio_index++;
        }
        return result;
    }

    return 0x00;
}

void gpio_module_write(uint16_t address, uint8_t value)
{
    address &= 0xFF;

    if (address <= GPIO_DIR_END)
    {
        // Write to GPIO direction and update cache
        uint8_t gpio_addr = (uint8_t)(address - GPIO_DIR_START);
        uint8_t gpio_index = gpio_addr == 0 ? 0 : 8; // Start at 8 if it's the high byte

        // Update the cache
        if (gpio_addr == 0) {
            gpio_direction_cache = (gpio_direction_cache & 0xFF00) | value; // Update low byte
        } else {
            gpio_direction_cache = (gpio_direction_cache & 0x00FF) | ((uint16_t)value << 8); // Update high byte
        }

        for (uint8_t i = 0; i < 8; i++) {
            if (gpio_index < sizeof(gpio_map) / sizeof(gpio_entry_t))
            {
                uint8_t gpio_dir = (value >> i) & 0x01;

                if (gpio_dir == 1)
                {
                    GPIO_InitTypeDef GPIO_InitStruct;
                    GPIO_InitStruct.Pin = gpio_map[gpio_index].pin;
                    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
                    GPIO_InitStruct.Pull = GPIO_NOPULL;
                    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                    HAL_GPIO_Init(gpio_map[gpio_index].port, &GPIO_InitStruct);
                }
                else
                {
                    GPIO_InitTypeDef GPIO_InitStruct;
                    GPIO_InitStruct.Pin = gpio_map[gpio_index].pin;
                    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                    GPIO_InitStruct.Pull = GPIO_NOPULL;
                    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                    HAL_GPIO_Init(gpio_map[gpio_index].port, &GPIO_InitStruct);
                }
            }
            gpio_index++;
        }
    }

    if (address >= GPIO_DATA_START && address <= GPIO_DATA_END)
    {
        // Write to GPIO data
        uint8_t gpio_addr = (uint8_t)(address - GPIO_DATA_START);
        uint8_t gpio_index = gpio_addr == 0 ? 0 : 8; // Start at 8 if it's the high byte

        for (uint8_t i = 0; i < 8; i++) {
            if (gpio_index < sizeof(gpio_map) / sizeof(gpio_entry_t))
            {
                uint8_t gpio_value = (value >> i) & 0x01;

                if (gpio_value == 1)
                {
                    HAL_GPIO_WritePin(gpio_map[gpio_index].port, gpio_map[gpio_index].pin, GPIO_PIN_SET);
                }
                else
                {
                    HAL_GPIO_WritePin(gpio_map[gpio_index].port, gpio_map[gpio_index].pin, GPIO_PIN_RESET);
                }
            }
            gpio_index++;
        }
    }
}