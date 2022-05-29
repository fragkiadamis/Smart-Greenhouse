#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include "smart_greenhouse_mcu.h"

SoftwareSerial BTSerial(BT_RX, BT_TX);

void SmartGreenHouseMCU::begin(void) {
    BTSerial.begin(BT_BAUD_RATE);
    while(!BTSerial);
    BTSerial.flush();
}

bool SmartGreenHouseMCU::hasMessage(void) {
    return BTSerial.available();
}
    
void SmartGreenHouseMCU::receive(char *buffer) {
    uint8_t size = 0;
    unsigned long time = micros();

    while(micros() - time <= SERIAL_TIMEOUT) {
        if (BTSerial.available()) {
        char c = BTSerial.read();
        
        // If the buffer is almost full or end of line, stop
        if(size == (SERIAL_BUFFER_SIZE - 1) || c == '\n')
            break;
            
            buffer[size++] = c;
            time = micros(); // Every time a new character arrives update the timeout
        }
    }
    buffer[size] = '\0';
}

void SmartGreenHouseMCU::send(float value) {
    BTSerial.println(value);
}

void SmartGreenHouseMCU::mcu_sleep(void) {
    sleep_enable(); // Enabling sleep mode
    set_sleep_mode(SLEEP_MODE_ADC); // Setting the sleep mode.
    sleep_cpu(); // Activating sleep mode
    sleep_disable();
}
