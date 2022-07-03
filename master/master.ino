#include "master.h"

// Initialize communication protocol and serial handling
SmartGreenHouseMCU sghMCU;

void setup() {
    // Setup hardware and bluetooth serial
    sghMCU.setupHardwareSerial();
    sghMCU.setupBTSerial();
    Serial.println("Master MCU is ready!");

    // Set sleep timer
    // esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

void loop() {
    uint16_t reading1 = touchRead(T0);
    uint16_t reading2 = touchRead(T3);
    
    if (sghMCU.hasMessage()) {
        char msg[SERIAL_BUFFER_SIZE] = {0};
        sghMCU.receive(msg);
        Serial.print(msg);
    }

    sghMCU.send("0|1");
    delay(1000);
    sghMCU.send("0|0");
    delay(1000);

    // Serial.println("Going to sleep now");
    // esp_deep_sleep_start();
}