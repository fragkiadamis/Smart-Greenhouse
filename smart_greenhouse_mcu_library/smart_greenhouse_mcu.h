// BluetoothCom.h
#ifndef SMART_GREENHOUSE_MCU_H
#define SMART_GREENHOUSE_MCU_H

#define SERIAL_BUFFER_SIZE 16
#define SERIAL_TIMEOUT 10000
#define BT_BAUD_RATE 38400
#define BT_RX 10
#define BT_TX 11

#include "Arduino.h"

class SmartGreenHouseMCU {
    private:
    public:
        void begin(void);
        bool hasMessage(void);
        void receive(char *buffer);
        void send(float value);
        void mcu_sleep(void);
};

#endif