// Serials
#define BAUD_RATE 57600
#define BT_BAUD_RATE 38400
#define SERIAL_BUFFER_SIZE 16
#define SERIAL_TIMEOUT 10000
#define BT_RX 10
#define BT_TX 11

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