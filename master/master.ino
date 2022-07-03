#include "master.h"

// Initialize communication protocol and serial handling
SmartGreenHouseMCU sghMCU;

void setup() {
    // Setup hardware and bluetooth serial
    sghMCU.setupHardwareSerial();
    sghMCU.setupBTSerial();
    Serial.println("Master MCU is ready!");
}

void loop() {
    // uint16_t reading1 = touchRead(T0);
    // uint16_t reading2 = touchRead(T3);
    
    // String onn = "0|1";
    // String off = "0|0";
    
    // if (Serial2.available())
    //     Serial.write(Serial2.read());

    // Serial2.println(onn);
    // delay(1000);
    // Serial2.println(off);
    // delay(1000);
    // Serial.println("TEST");
}