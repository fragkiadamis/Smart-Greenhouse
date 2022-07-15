// Include libraries
#include <smart_greenhouse_serial.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <PubSubClient.h>

// MQTT
#define SSID "COSMOTE-304503"
#define PWD "gm8un268me928dht"
#define MQTT_SERVER "192.168.1.4"
#define PORT 1883

// EEPROM
// #define INITIALIZE_EEPROM // BE CAREFUL!!!! Uncomment this the first time that the MCU is running or when you want to reset the entirety of EEPROM to zero's.
#define EEPROM_SIZE 512
#define MODE_ADDRESS 0
#define AC_MODE_COOLING_ADDRESS 1
#define AC_HUMIDIFY_ADDRESS 2

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
#define LOW_IN_TEMP_THRESHOLD 23
#define HIGH_OUT_TEMP_THRESHOLD 31
#define HIGH_IN_TEMP_THRESHOLD 28
#define AC_LOW_STOP_TEMP 25
#define AC_HIGH_STOP_TEMP 28
#define LOW_LUMINOSITY_THRESHOLD 40
#define HIGH_LUMINOSITY_THRESHOLD 80
#define LOW_HUMIDITY_THRESHOLD 50
#define HIGH_HUMIDITY_THRESHOLD 70

// Wate Tank
#define WATER_TANK T8
#define MAX_LEVEL 500
#define WT_LOW_THRESHOLD 25
#define WT_STOP_THRESHOLD 80

// Air-Condition
#define AC_MOTOR_PIN1 8
#define AC_MOTOR_PIN2 9
#define AC_HUMIDIFICATION 34
#define AC_DEHUMIDIFICATION 35

// MQTT Topics
#define MODE "esp32/mode"
#define AC "esp32/ac"
#define IRR "esp32/irrigation"
#define SHTR "esp32/shutter"
