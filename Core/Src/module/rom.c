/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ROM module implementation
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
#include "module/rom.h"

#define INFO_PAGE_START (0x08000000 + 1024 * 30)
static volatile const uint8_t *info_memory = (const uint8_t *)INFO_PAGE_START;
// Even though the ROM is mapped in a 4K region on I2C, only the last 2K page is currently allocated to it
#define INFO_PAGE_SIZE (2*1024)

void rom_module_init(void)
{
    // ROM module doesn't need initialization
}

uint8_t rom_module_read(uint16_t address)
{
    address &= 0xFFF;

    if (address < INFO_PAGE_SIZE)
    {
        return info_memory[address];
    }
    return 0x00;
}