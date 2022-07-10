#include "master.h"

// Initialize communication protocol and serial handling
SmartGreenHouseMCU sghMCU;

String sendCmdAndGetRes(String cmd) {
    sghMCU.send(cmd);
    while (!sghMCU.hasMessage());
    char res[SERIAL_BUFFER_SIZE] = {0};
    sghMCU.receive(res);
    return String(res);
}

// Check atmospheric temperature.
void checkTemperatures(float *innerTemperature, float *outerTemperature) {
    String cmd = "";

    // Read inner and outer temperature.
    cmd = String(sghMCU.DHT_SENS) + '|' + String(sghMCU.INNER_TEMP);    // Create command,
    *innerTemperature = sendCmdAndGetRes(cmd).toFloat();                // And send it.
    cmd = String(String(sghMCU.OUTER_TEMP));
    *outerTemperature = sendCmdAndGetRes(cmd).toFloat();
}

void checkLuminosityDif(float *luminosityDif) {
    String cmd = "";

    // Read inner and outer temperature.
    cmd = String(String(sghMCU.LUM));                   // Create command,
    *luminosityDif = sendCmdAndGetRes(cmd).toFloat();   // And send it.
}

void setup() {
    // Setup hardware and bluetooth serial
    sghMCU.setupHardwareSerial();
    sghMCU.setupBTSerial();

    // Set sleep timer
    // esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

    delay(1000);
    Serial.println("Master MCU is ready!");
}

void loop() {
    // Initialize variables for atmospheric values
    float innerTemperature = 0, outerTemperature = 0;
    float luminosityDif = 0;

    // Check atmosphere and handle it.
    checkTemperatures(&innerTemperature, &outerTemperature);
    Serial.println("Inner Temp: " + String(innerTemperature));
    Serial.println("Outer Temp: " + String(outerTemperature));

    checkLuminosityDif(&luminosityDif);
    Serial.println("Luminosity Percentage: " + String(luminosityDif));

    // uint16_t reading1 = touchRead(T0);
    // uint16_t reading2 = touchRead(T4);
    // uint16_t reading3 = touchRead(T2);

    // Serial.println("GO UP");
    // digitalWrite(SHUTTER_MOTOR_PIN1, HIGH);
    // digitalWrite(SHUTTER_MOTOR_PIN2, LOW);
    // delay(2000);

    // Serial.println("GO DOWN");
    // digitalWrite(SHUTTER_MOTOR_PIN1, LOW);
    // digitalWrite(SHUTTER_MOTOR_PIN2, HIGH);
    // delay(2000);

    // sghMCU.send("1|0");
    // delay(1000);

    Serial.println("Going to sleep now");
    delay(1000);
    // esp_light_sleep_start();
}