// Demo of config() and fast toggle() function.
#include <DigitalIO.h>

// Class with compile time pin number.
DigitalPin<13> pin13;

void setup() {
  // Set mode to OUTPUT and level LOW.
  pin13.config(OUTPUT, LOW);
}
void loop() {
  pin13.toggle();
  pin13.toggle();
  delay(1);
}