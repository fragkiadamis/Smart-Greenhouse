// BluetoothCom.h

#include "Arduino.h"
#include <SoftwareSerial.h>
#include <avr/sleep.h>

#ifndef SMART_GREENHOUSE_MCU_H
#define SMART_GREENHOUSE_MCU_H

#define SERIAL_BUFFER_SIZE 16
#define SERIAL_TIMEOUT 10000
#define BT_BAUD_RATE 38400
#define BT_RX 10
#define BT_TX 11

class SmartGreenHouseMCU {
    private:
    public:
        // DHT sensors enumeration
        typedef enum {
            HUMIDITY,
            INNER_TEMP
        } DHT_SENSOR;

        // Action enumeration
        typedef enum {
            BZ,         // Buzzer
            DHT_SENS,   // DHT Humidity
            IRG,        // Irrigation
            OUTER_TEMP, // Outer temperature
            LUM         // Inner lumination
        } ACTION;

        // Methods
        void begin(void);
        bool hasMessage(void);
        void receive(char *buffer);
        void send(float value);
        void mcu_sleep(void);
};

#endif