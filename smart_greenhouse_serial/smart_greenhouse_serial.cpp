#include "smart_greenhouse_serial.h"

/*
 Determine The serial that is going to be used
 as the bluetooth serial according to the type
 of the MCU and compile it.
*/
#if defined(ESP_PLATFORM)
    #define BTSerial Serial2 // Set BTSerial as Serial2 alias
#elif defined(__AVR_ATmega328P__)
    #include <SoftwareSerial.h>
    SoftwareSerial BTSerial(SLAVE_BT_RX, SLAVE_BT_TX); // Set software serial as BTSerial
#endif

// Setup the hardware serial (for debugging)
void SmartGreenHouseSerial::setupHardwareSerial(void) {
    // Setup hardware serial
    Serial.begin(BAUD_RATE);
    while(!Serial);
    Serial.flush();
}

// Setup the bluetooth serial
void SmartGreenHouseSerial::setupBTSerial(void) {
// Compile bluetooth serial setup, according to MCU type
#if defined(ESP_PLATFORM)
    BTSerial.begin(BT_BAUD_RATE, SERIAL_8N1, MASTER_BT_RX, MASTER_BT_TX);
#elif defined(__AVR_ATmega328P__)
    BTSerial.begin(BT_BAUD_RATE);
#endif
    while(!BTSerial);
    BTSerial.flush();
}

// Check if bluetooth serial is available (pending message)
bool SmartGreenHouseSerial::hasMessage(void) {
    return BTSerial.available();
}

// Receive message from bluetooth
String SmartGreenHouseSerial::receive(void) {
    return BTSerial.readString();
}

// Send a message via bluetooth
void SmartGreenHouseSerial::send(String msg) {
    BTSerial.println(msg);
}
