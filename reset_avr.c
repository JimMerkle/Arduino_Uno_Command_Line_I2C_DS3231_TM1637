// Example reboot code from:
// https://forum.arduino.cc/t/soft-reset-and-arduino/367284/6

#include <avr/wdt.h>

void reboot(void) {
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}