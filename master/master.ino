#include <smart_greenhouse_mcu.h>
#include "master.h"

void setup() {
  // Setup hardware serial
  Serial.begin(BAUD_RATE);
  while(!Serial);
  Serial.flush();
}

void loop() {

}