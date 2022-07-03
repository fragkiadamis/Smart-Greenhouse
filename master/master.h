// Include libraries
#include <smart_greenhouse_mcu.h>

// Serials
#define BAUD_RATE 115200
#define BTSerial Serial2

// Bluetooth
#define SLAVE_ADDR 98d3:32:3165fe

// Shutter
#define UPPER_END T0
#define LOWER_END T3
#define TOUCH_THRESHOLD 20
#define SHUTTER_MOTOR_PIN1 26
#define SHUTTER_MOTOR_PIN2 27
