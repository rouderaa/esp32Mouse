/*!
 *  @file       PS2Com.cpp
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

#include <Arduino.h>
#include <stdint.h>
#include <freertos/portmacro.h>
#include "PS2Com.h"
#include "RingBuf.h"

// PS2 read states
#define PS2IDLE 0
#define PS2DATA 1
#define PS2PARITY 2
#define PS2STOP 3

static portMUX_TYPE intMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint8_t data;
static volatile uint8_t bitcount;
static volatile uint8_t localClk;
static volatile uint8_t localData;
static volatile uint8_t clkValue = 0;
static volatile uint8_t PS2State = PS2IDLE;
static RingBuf<uint8_t, 16> ringbuffer;

// Called on each falling edge of the CLK line
static void IRAM_ATTR onPS2ClkDown()
{
  // Glitch detection
  int PS2Clk = digitalRead(localClk);
  if (PS2Clk != 0)
    return;

  // digitalWrite(DEBUGOUT1, LOW);

  int PS2Data = digitalRead(localData);
  switch (PS2State)
  {
  case PS2IDLE:
    data = 0;
    bitcount = 0;
    if (PS2Data == 0)
    {
      // Must be 0 for startbit
      PS2State = PS2DATA;
    }
    break;
  case PS2DATA:
    data = data >> 1;
    if (PS2Data == 1)
      data = (data | 0x80);
    bitcount++;
    if (bitcount == 8)
    {
      PS2State = PS2PARITY;
    }
    break;
  case PS2PARITY:
    // Odd parity check
    uint8_t p;
    p = data ^ (data >> 1);
    p = p ^ (p >> 2);
    p = p ^ (p >> 4);
    if (!((p & 1) != PS2Data)) {
      // invalid parity
      data = 0xFC; // return error
    }

    PS2State = PS2STOP;
    break;
  case PS2STOP:
    portENTER_CRITICAL_ISR(&intMux);
    ringbuffer.push(data);
    portEXIT_CRITICAL_ISR(&intMux);

    PS2State = PS2IDLE;
    break;
  }
  // digitalWrite(DEBUGOUT1, HIGH);
}

// Alternate clock line with 15 ms interval in rest period
#define MOUSEALTERNATE 15
bool PS2Com::alternate() {
  if (alternateTimer < millis()) {
    alternateTimer = millis() + MOUSEALTERNATE;
    clkValue = !clkValue;
    digitalWrite(localClk, clkValue);
  }
  return clkValue;
}

bool PS2Com::available(int nrOfBytes)
{
  portENTER_CRITICAL_ISR(&intMux);
  bool filled = (ringbuffer.size() >= nrOfBytes);
  portEXIT_CRITICAL_ISR(&intMux);

  return filled;
}

uint8_t PS2Com::readByte()
{
  portENTER_CRITICAL_ISR(&intMux);
  uint8_t val;
  ringbuffer.pop(val);
  portEXIT_CRITICAL_ISR(&intMux);

  // Serial.printf("+%02X", val);

  return val;
}

void PS2Com::writeByte(uint8_t v)
{
  // digitalWrite(DEBUGOUT2, LOW); // for dsview debugging

  delayMicroseconds(140); // Give previous ack handling ample time

  // No read interrupts during writing
  portENTER_CRITICAL_ISR(&intMux); // disables interrupts
  detachInterrupt(clkPin);

  // Calculate parity
  bool parity = true;
  for (uint8_t i = 0; i < 8; i++)
  {
    if ((v & (1 << i)) != 0)
      parity = !parity;
  }

  uint16_t shift = v | (0x200) | (0x100 * parity);

  pinMode(dataPin, OUTPUT_OPEN_DRAIN | PULLUP);
  digitalWrite(dataPin, LOW);
 
  pinMode(clkPin, OUTPUT_OPEN_DRAIN | PULLUP);
  digitalWrite(clkPin, LOW);

  delayMicroseconds(140);

  // Make clk pin input since clock pulses now come from device
  pinMode(clkPin, INPUT_PULLUP);

  // digitalWrite(DEBUGOUT2, HIGH); // for dsview debugging

  // Write bits in the shift variable
  for (uint8_t i = 0; i < 10; i++)
  {
    while (!digitalRead(clkPin))
      ;
    while (digitalRead(clkPin))
      ;
    digitalWrite(dataPin, shift & 1);
    shift >>= 1;
  }
  digitalWrite(dataPin, HIGH);

  // Handle stop bit
  while (!digitalRead(clkPin))
    ;
  while (digitalRead(clkPin))
    ;

  // Set data line back to allow for reading
  pinMode(dataPin, INPUT_PULLUP);

  // release control of clock line, waiting for ACK
  pinMode(clkPin, INPUT | PULLUP);

  /* mode: if set to 0: GPIO interrupt disable  if set to 1: rising edge trigger  if set to 2: falling edge trigger  if set to 3: any edge trigger  if set to 4: low level trigger  if set to 5: high level trigger*/
  int mode = 2; // Falling edge trigger
  attachInterrupt(clkPin, onPS2ClkDown, mode);  

  portEXIT_CRITICAL_ISR(&intMux);

  // Serial.printf("-%02X", v);
}

void PS2Com::begin(uint8_t clk, uint8_t data)
{
  clkPin = clk;
  localClk = clk;
  dataPin = data;
  localData = data;

  pinMode(clk, OUTPUT_OPEN_DRAIN | PULLUP);
  pinMode(data, INPUT_PULLUP);

  // digitalWrite(data, HIGH);
  digitalWrite(clk, HIGH);

  // pinMode(DEBUGOUT1, OUTPUT);
  // pinMode(DEBUGOUT2, OUTPUT);
  // digitalWrite(DEBUGOUT1, HIGH);
  // digitalWrite(DEBUGOUT2, HIGH);

  /* mode: if set to 0: GPIO interrupt disable  if set to 1: rising edge trigger  if set to 2: falling edge trigger  if set to 3: any edge trigger  if set to 4: low level trigger  if set to 5: high level trigger*/
  int mode = 2; // Falling edge trigger
  attachInterrupt(clkPin, onPS2ClkDown, mode);  
}
