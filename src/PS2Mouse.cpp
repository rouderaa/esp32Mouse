/*!
 *  @file       PS2Mouse.cpp
 *  Project     esp32Ps2
 *  @brief      Software implementation of PS2 protocol to interface with PS2 compatable mouse.
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

#include <Arduino.h>
#include <stdint.h>
#include <freertos/portmacro.h>
#include "PS2Mouse.h"

// Possible mouse states
#define MOUSESTARTUP 0
#define MOUSESTARTUP1 1
#define MOUSESTARTUP2 2
#define MOUSESENDCOMMAND 3
#define MOUSEACK 4
#define MOUSEALTERNATE 5
#define MOUSEDEVICETYPE 6
#define MOUSERESETACK 7
#define MOUSEWAITFORCLKLOW 8
#define MOUSEDELAYSEND 9

void PS2Mouse::begin(uint8_t clk, uint8_t data)
{
  ps2Com.begin(clk, data);

  // Wait for mouse to be initialized before continueing
  while (!isInitialized()) {
    loop();
  }
}

bool PS2Mouse::isInitialized()
{
  return (state == MOUSEALTERNATE);
}

// packet is 4 bytes for mouse of type 03
// returns number of read bytes
int PS2Mouse::readPacket(uint8_t *toPacket)
{
  if (ps2Com.available(4))
  {
    byte b;
    do
    {
      b = ps2Com.readByte();
    } while ((b & 0x08) == 0); // Test if first byte of packet
    toPacket[0] = b;
    toPacket[1] = ps2Com.readByte();
    toPacket[2] = ps2Com.readByte();
    toPacket[3] = ps2Com.readByte();

    return 4;
  }
  return 0x00;
}

uint8_t PS2Mouse::nextByte()
{
  if (ps2Com.available(1))
  {
    return ps2Com.readByte();
  }
  return 0xff;
}

uint8_t PS2Mouse::getDeviceType()
{
  return deviceType;
}

// Note: the setting of several sample rates is necessary for the mouse to goto 'intellimouse' mode
static uint8_t initCommands[] = {
    0xF3, // Set sample Rate
    0xC8, // decimal 200
    0xF3, // Set sample Rate
    0x64, // decimal 100
    0xF3, // Set sample Rate
    0x50, // decimal 80
    0xF2, // Read device type

    0xE8, // Set resolution
    0x03, // 8 counts/mm
    0xE6, // Scaling 1:1
    0xF3, // Sample rate
    0x28, // decimal 40
    0xF4, // Enable device
    0x00  // End marker
};

void PS2Mouse::loop()
{
  switch (state)
  {
  case MOUSESTARTUP:
    // digitalWrite(DEBUGOUT2, LOW); // DEBUG

    // Send reset
    ps2Com.writeByte(0xFF);
    // Start initialisation again
    commandIndex = 0;
    state = MOUSERESETACK;
    break;
  case MOUSESTARTUP1:
    ps2Com.alternate();
    if (ps2Com.available(1))
    {
      uint8_t b = ps2Com.readByte();
      if (b == 0xAA)
        state = MOUSESTARTUP2;
    }
    break;
  case MOUSESTARTUP2:
    ps2Com.alternate();
    if (ps2Com.available(1))
    {
      uint8_t b = ps2Com.readByte();
      if (b == 0x00)
      {
        state = MOUSEWAITFORCLKLOW;
      }
    }
    break;
  case MOUSEWAITFORCLKLOW:

    if (ps2Com.alternate() == LOW)
    {
      sendDelay = millis() + 12; // 12ms delay low clk
      state = MOUSEDELAYSEND;
    }
    break;
  case MOUSEDELAYSEND:
    // wait 12 ms before sending data
    if (sendDelay < millis())
    {
      commandIndex = 0;
      state = MOUSESENDCOMMAND;
    }
    break;
  case MOUSESENDCOMMAND:
    if (initCommands[commandIndex] == 0)
    {
      state = MOUSEALTERNATE;
    }
    else
    {
      latestCommand = initCommands[commandIndex];
      ps2Com.writeByte(latestCommand);
      state = MOUSEACK;
    }
    break;
  case MOUSEACK:
    if (ps2Com.available(1))
    {
      uint8_t b = ps2Com.readByte();
      if (b == 0xFA) // ACK
      {
        // Select next command
        commandIndex++;
        if (latestCommand == 0xF2)
        {
          // Read device type
          state = MOUSEDEVICETYPE;
        }
        else
        {
          state = MOUSESENDCOMMAND;
        }
      }
      if (b == 0xFE) // Resend
      {
        state = MOUSESENDCOMMAND;
      }
      if (b == 0xFC) // Error
      {
        // Send reset
        ps2Com.writeByte(0xFF);
        // Start initialisation again
        commandIndex = 0;
        state = MOUSERESETACK;
      }
    }
    break;
  case MOUSERESETACK:
    if (ps2Com.available(1))
    {
      uint8_t b = ps2Com.readByte();
      if (b == 0xFA) // ACK
      {
        state = MOUSESTARTUP1;
      }
    }
    break;
  case MOUSEDEVICETYPE:
    if (ps2Com.available(1))
    {
      uint8_t b = ps2Com.readByte();
      deviceType = b;
      state = MOUSESENDCOMMAND;
    }
    break;
  case MOUSEALTERNATE:
    ps2Com.alternate();
    break;
  }

  // Parse collected bytes into mouse status
  uint8_t packet[4];
  if (readPacket(packet) != 0)
  {
    // Got packet, examine it into values
    uint8_t b;
    b = packet[0];
    leftButton = (b & 0x1);
    middleButton = (b & 0x4);
    rightButton = (b & 0x2);

    xMove = (int8_t)packet[1];
    if (xMove != 0)
      latestXMove = xMove;
    xPos += xMove;
    yMove = (int8_t)packet[2];
    if (yMove != 0)
      latestYMove = yMove;
    yPos += yMove;
    zPos += (int8_t)packet[3];
  }
}

bool PS2Mouse::getLeftButton()
{
  return leftButton;
}

bool PS2Mouse::getMiddleButton()
{
  return middleButton;
}

bool PS2Mouse::getRightButton()
{
  return rightButton;
}

int32_t PS2Mouse::getXpos()
{
  return xPos;
}

int32_t PS2Mouse::getYpos()
{
  return yPos;
}

int32_t PS2Mouse::getZpos()
{
  return zPos;
}

int32_t PS2Mouse::getLatestXMove()
{
  return latestXMove;
}

int32_t PS2Mouse::getLatestYMove()
{
  return latestYMove;
}

// Returns readable status of mouse for debugging
void PS2Mouse::getState(char *toState)
{
  sprintf(
      toState,
      "l:%d m:%d r:%d x:%d y:%d z:%d",
      getLeftButton(),
      getMiddleButton(),
      getRightButton(),
      getXpos(),
      getYpos(),
      getZpos());
}