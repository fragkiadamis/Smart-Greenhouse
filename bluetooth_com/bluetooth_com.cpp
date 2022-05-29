#include <SoftwareSerial.h>
#include "bluetooth_com.h"

SoftwareSerial BTSerial(BT_RX, BT_TX);

void BluetoothCom::begin() {
    BTSerial.begin(BT_BAUD_RATE);
    while(!BTSerial);
    BTSerial.flush();
}

bool BluetoothCom::hasMessage(void) {
    return BTSerial.available();
}
    
void BluetoothCom::receive(char *buffer) {
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

void BluetoothCom::send(float value) {
    BTSerial.println(value);
}
