// Include libraries
#include <smart_greenhouse_mcu.h>
#include <EEPROM.h>

// Sleep
#define uS_TO_S_FACTOR 1000000  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  5        // Time ESP32 will go to sleep (in seconds)

// LEDs
#define AC_COOLING 22
#define AC_HEATING 23
#define ARTIFICIAL_LIGHTING 32

// Shutter
#define UPPER_END T0
#define MIDDLE T4
#define LOWER_END T3
#define TOUCH_THRESHOLD 30
#define SHUTTER_MOTOR_PIN1 6
#define SHUTTER_MOTOR_PIN2 7

// Atmospheric Thresholds
#define LOW_OUT_TEMP_THRESHOLD 22
#define HIGH_OUT_TEMP_THRESHOLD 31
#define LOW_LUMINOSITY_THRESHOLD 40
#define HIGH_LUMINOSITY_THRESHOLD 80

// Air-Condition
#define AC_MOTOR_PIN1 8
#define AC_MOTOR_PIN2 9

#define RES_TIMEOUT 5000
