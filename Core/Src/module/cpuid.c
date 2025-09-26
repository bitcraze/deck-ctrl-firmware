/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * CPU ID module implementation
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
#include "module/cpuid.h"

#define UID_BASE_ADDR  0x1FFF7550UL
static volatile uint8_t *cpuid = (volatile uint8_t *)UID_BASE_ADDR;
#define CPUID_LENGTH 12

void cpuid_module_init(void)
{
}

uint8_t cpuid_module_read(uint16_t address)
{
    address &= 0xFF;

    if (address < CPUID_LENGTH)
    {
        return cpuid[address];
    }
    return 0x00;
}