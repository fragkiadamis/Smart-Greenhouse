// Include libraries
#include <smart_greenhouse_serial.h>
#include <avr/sleep.h>
#include <Servo.h>
#include "DHT.h"

// Actuators
#define BUZZER_PIN 3
#define BUZZER_FREQ 1000
#define VALVE_PIN 5

// Sensors
#define DHT_PIN 4
#define DHT_TYPE DHT11
#define LM35_PIN A5
#define INNER_LDR_PIN A3
#define OUTTER_LDR_PIN A4

// Interrupt
#define INTERRUPT_PIN 2
