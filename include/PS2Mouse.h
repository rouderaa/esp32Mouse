/*!
 *  @file       PS2Mouse.h
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
#include "PS2Com.h"

class PS2Mouse {

    public:
        void begin(uint8_t clk, uint8_t data);
        void loop();
        bool isInitialized();
        uint8_t nextByte();
        int readPacket(uint8_t *toPacket);
        uint8_t getDeviceType();

        bool getLeftButton();
        bool getMiddleButton();
        bool getRightButton();

        int32_t getXpos();
        int32_t getYpos();
        int32_t getZpos();
        int32_t getLatestXMove();
        int32_t getLatestYMove();

        void getState(char *toState);

    private:
        PS2Com ps2Com;

        uint8_t state = 0;
        uint8_t latestCommand = 0;
        uint8_t deviceType = 0; // Either 0 for regular or 3 for scrolling mouse
        uint8_t commandIndex = 0;
        unsigned long sendDelay = 0L;

        // Detected state of mouse buttons
        bool leftButton = false;
        bool middleButton = false;
        bool rightButton = false;

        // Relative movement since last measurement
        int16_t xMove = 0;
        int16_t yMove = 0;
        int16_t latestXMove = 0;
        int16_t latestYMove = 0;

        // Absolute positions
        int32_t xPos = 0;
        int32_t yPos = 0;
        int32_t zPos = 0;
};