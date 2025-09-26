/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * I2C memory implementation
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
#include "stm32c0xx_ll_i2c.h"
#include "memory.h"
#include "config.h"
#include "module/gpio.h"
#include "module/rom.h"
#include "module/cpuid.h"
#include "module/scratchpad.h"


#define ROM_MEMORY_MASK 0x0FFF
#define GPIO_MEMORY_START 0x1000
#define GPIO_MEMORY_MASK 0x00FF

#define CPUID_START 0x1900
#define CPUID_MEMORY_MASK 0x00FF

#define I2C_ADDRESS_UPDATE_REGISTER 0x1800

#define SCRATCHPAD_MEMORY_START 0x1F00
#define SCRATCHPAD_MEMORY_MASK 0x00FF

static uint8_t i2c_address = I2C_ADDR_DISCOVERY >> 1;
static bool i2c_address_updated = false;

void memory_init(void)
{
  gpio_module_init();
  rom_module_init();
  cpuid_module_init();
  scratchpad_module_init();
}

void memory_set_i2c_address(uint8_t address)
{
  i2c_address = address;
  i2c_address_updated = true;
}

uint8_t memory_get_i2c_address(void)
{
  return i2c_address;
}

bool memory_check_and_reset_i2c_address_update(void)
{
  if (i2c_address_updated)
  {
    i2c_address_updated = false;
    return true; // Indicate that the address was updated
  }
  return false; // No update
}

uint8_t memory_read(uint16_t address)
{
  // ROM module (device info) - 0x0000-0x0FFF
  if ((address & 0xF000) == 0x0000)
  {
    return rom_module_read(address & ROM_MEMORY_MASK);
  }

  // GPIO module - 0x1000-0x10FF
  if ((address & 0xFF00) == GPIO_MEMORY_START)
  {
    return gpio_module_read(address & GPIO_MEMORY_MASK);
  }

  // CPU ID - 0x1900-0x19FF
  if ((address & 0xFF00) == CPUID_START)
  {
    return cpuid_module_read(address & CPUID_MEMORY_MASK);
  }

  // Scratchpad memory - 0x1F00-0x1FFF
  if ((address & 0xFF00) == SCRATCHPAD_MEMORY_START)
  {
    return scratchpad_module_read(address & SCRATCHPAD_MEMORY_MASK);
  }

  return 0x00;
}

void memory_write(uint16_t address, uint8_t value)
{
  // GPIO module - 0x1000-0x10FF
  if ((address & 0xFF00) == GPIO_MEMORY_START)
  {
    gpio_module_write(address & GPIO_MEMORY_MASK, value);
    return;
  }

  // Scratchpad memory - 0x1F00-0x1FFF
  if ((address & 0xFF00) == SCRATCHPAD_MEMORY_START)
  {
    scratchpad_module_write(address & SCRATCHPAD_MEMORY_MASK, value);
    return;
  }

  // I2C address update
  if (address == I2C_ADDRESS_UPDATE_REGISTER)
  {
    i2c_address = value; // Store the address as 7-bit shifted to 8-bit
    DBG_PRINT("I2C address updated to: 0x%02X\n", i2c_address);
    i2c_address_updated = true;
  }
}