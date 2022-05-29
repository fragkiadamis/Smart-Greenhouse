// Serials
#define BAUD_RATE 57600

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