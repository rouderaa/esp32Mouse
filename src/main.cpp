#include <Arduino.h>
#include "PS2Mouse.h"

// Your two pins attached to the PS2 mouse and power lines
// NOTE: warning: 
// measure with voltage meter that RED is really your +5V of your mouse !!
                   // BLACK USB A, pin 4, is GND
#define PS2CLK 25  // GREEN USB A, pin 3, is PS2CLK
#define PS2DATA 26 // WHITE USB A, pin 2, is PS2DATA
                   // RED   USB A, pin 1, is +5V

static PS2Mouse ps2Mouse;

void setup()
{
  Serial.begin(115200);

  ps2Mouse.begin(PS2CLK, PS2DATA);

  Serial.printf("Activated\n\r");
}

void loop()
{
  ps2Mouse.loop();

  // Show state of mouse, just for debugging and demo
  char state[128];
  ps2Mouse.getState(state);
  Serial.printf("%s\n\r", state);
}