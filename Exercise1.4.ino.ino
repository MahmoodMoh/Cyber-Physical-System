#include "sam.h"
#define redLed 10   // PB10 (D4)

int delayT2 = 1000;

void setup() {
  // Enable clock for PORT peripheral
  PM->APBBMASK.reg |= PM_APBBMASK_PORT;
  // Set PB10 as output (D4)
  PORT->Group[1].DIRSET.reg = (1u << redLed);
}

void loop() {

  // Toggle PB
  PORT->Group[1].OUTSET.reg = (1u << redLed);
  delay(delayT2);
  PORT->Group[1].OUTCLR.reg = (1u << redLed);
  delay(delayT2);
}