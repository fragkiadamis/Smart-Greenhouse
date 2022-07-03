// Include libraries
#include <smart_greenhouse_mcu.h>

// Sleep
#define uS_TO_S_FACTOR 1000000  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  5        // Time ESP32 will go to sleep (in seconds)

// Shutter
#define UPPER_END T0
#define LOWER_END T3
#define TOUCH_THRESHOLD 20
#define SHUTTER_MOTOR_PIN1 26
#define SHUTTER_MOTOR_PIN2 27
