/*
 * This file is part of the FreeRTOS port to Teensy boards.
 * Copyright (c) 2018 Timo Sandmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    teensy.h
 * @brief   FreeRTOS support implementations for Teensy boards
 * @author  Timo Sandmann
 * @date    26.05.2018
 */

#ifndef PORTABLE_TEENSY_H_
#define PORTABLE_TEENSY_H_

#include <cstdint>


extern "C" {
extern uint8_t* stack_top; /**< Pointer to top of initial stack, initialized in setup() */

/**
 * @brief Write every character from the null-terminated C-string str and one additional newline character '\n' to Serial
 * @param[in] str: Character C-string to be written
 */
void serial_puts(const char* str);
} // extern C

namespace freertos {
/**
 * @brief Indicate an error with the onboard LED
 * @param[in] n: Number of short LED pulses to encode the error
 */
void error_blink(const uint8_t n);

/**
 * @brief Get amount of free (heap) RAM
 * @return Free RAM on heap in byte
 */
long free_ram();

/**
 * @brief Print amount of free (heap) RAM to Serial
 */
void print_free_ram();
} // namespace freertos

#endif /* PORTABLE_TEENSY_H_ */
