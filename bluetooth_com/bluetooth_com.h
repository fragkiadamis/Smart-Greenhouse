// BluetoothCom.h
#ifndef BLUETOOTH_COM_H
#define BLUETOOTH_COM_H

#define SERIAL_BUFFER_SIZE 16
#define SERIAL_TIMEOUT 10000
#define BT_BAUD_RATE 38400
#define BT_RX 10
#define BT_TX 11

#include "Arduino.h"

class BluetoothCom {
    private:
    public:
        void begin();
        bool hasMessage(void);
        void receive(char *buffer);
        void send(float value);
};

#endif