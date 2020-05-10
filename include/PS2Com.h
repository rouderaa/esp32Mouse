/*!
 *  @file       PS2Com.h
 *  Project     esp32Ps2
 *  @brief      Software implementation of PS2 protocol to interface with PS2 compatible mouse.
 *  @author     Rob van der Ouderaa
 *  @date       30/04/2020
 *  @license    MIT - Copyright (c) 2020 Rob van der Ouderaa
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <stdint.h>

// Used during debugging to show timing
// #define DEBUGOUT1 32
// #define DEBUGOUT2 33

class PS2Com {
    public:
        bool available(int nrOfBytes);
        uint8_t readByte();
        void    writeByte(uint8_t v);
        uint16_t getParityErrorCount();
        bool alternate();
        void begin(uint8_t clk, uint8_t data);

    private:
        uint8_t clkPin;
        uint8_t dataPin;
        unsigned long alternateTimer = 0L;
};