#include "master.h"

// Initialize communication protocol and serial handling.
SmartGreenHouseSerial sghSerial;

// State flags.
bool acIsEnabled = false;
bool coolingIsEnabled = false;
bool heatingIsEnabled = false;
bool openWindowIsAllowed = true;

// Send message via bluetooth and wait for a response.
String sendCmdAndGetRes(String cmd) {
    sghSerial.send(cmd);
    while (!sghSerial.hasMessage());
    char res[SERIAL_BUFFER_SIZE] = {0};
    sghSerial.receive(res);
    return String(res);
}

// Request the inside and the outside temperature.
void requestTemperatures(float *innerTemperature, float *outerTemperature) {
    String cmd = "";

    // Read inner and outer temperature.
    cmd = String(sghSerial.DHT_SENS) + '|' + String(sghSerial.INNER_TEMP);    // Create command,
    *innerTemperature = sendCmdAndGetRes(cmd).toFloat();                // And send it.
    cmd = String(String(sghSerial.OUTER_TEMP));
    *outerTemperature = sendCmdAndGetRes(cmd).toFloat();
}

// Request the inside / outside luminosity ratio of the greenhouse.
void requestLuminosity(float *luminosityRatio) {
    String cmd = "";

    // Read inner and outer temperature.
    cmd = String(String(sghSerial.LUM));                   // Create command,
    *luminosityRatio = sendCmdAndGetRes(cmd).toFloat();   // And send it.
}

// Determine window position by reading the capacitance sensors.
char determineWindowPosition(void) {
    if (touchRead(UPPER_END) < TOUCH_THRESHOLD)
        return 'u';
    else if (touchRead(MIDDLE) < TOUCH_THRESHOLD)
        return 'm';
    else if (touchRead(LOWER_END) < TOUCH_THRESHOLD)
        return 'l';

    return 'e';
}

// Control the window shutter.
void controlShutter(uint8_t sensor, uint8_t pin1Val, uint8_t pin2Val) {
    digitalWrite(SHUTTER_MOTOR_PIN1, pin1Val);
    digitalWrite(SHUTTER_MOTOR_PIN2, pin2Val);

    // Wait until the window touches the middle or the upper sensor (whatever is defined)
    while (touchRead(sensor) > TOUCH_THRESHOLD);
    // And stop the shutter
    digitalWrite(SHUTTER_MOTOR_PIN1, LOW);
    digitalWrite(SHUTTER_MOTOR_PIN2, LOW);
}

// Start air-condition.
void enableAircondition(bool cooling) {
    // Setup or change mode (cooling or heating).
    if (cooling) {
        digitalWrite(AC_COOLING, HIGH);
        coolingIsEnabled = true;
        heatingIsEnabled = false;
    } else {
        digitalWrite(AC_HEATING, HIGH);
        coolingIsEnabled = false;
        heatingIsEnabled = true;
    }

    // If it is not already on, then turn it on...
    if (!acIsEnabled) {
        digitalWrite(AC_MOTOR_PIN1, HIGH);
        digitalWrite(AC_MOTOR_PIN2, LOW);
        acIsEnabled = true;
    }
}

// Stop the aircondition.
void stopAircondition(void) {
    digitalWrite(AC_COOLING, LOW);
    digitalWrite(AC_HEATING, LOW);

    digitalWrite(AC_MOTOR_PIN1, LOW);
    digitalWrite(AC_MOTOR_PIN2, LOW);
    
    acIsEnabled = false;
    coolingIsEnabled = false;
    heatingIsEnabled = false;
}

// Check the temperature and handle it if needed.
void handleTemperature(void) {
    // Initialize variables for atmospheric values.
    float innerTemperature = 0, outerTemperature = 0;
    // Get temperatures.
    requestTemperatures(&innerTemperature, &outerTemperature);

    // Initialze window position and determine window position.
    char windowPos = determineWindowPosition();

    // Stop air-condition if it is on and the atmosphere is OK.
    if (acIsEnabled) {
        bool coolingIsOK = coolingIsEnabled && innerTemperature < AC_LOW_STOP_TEMP;
        bool heatingIsOK = heatingIsEnabled && innerTemperature > AC_HIGH_STOP_TEMP;
        if (coolingIsOK || heatingIsOK)
            stopAircondition();
    }

    // When the temperature is inside this span, handle temperature by opening / closing the window.
    if ((LOW_OUT_TEMP_THRESHOLD <= outerTemperature <= HIGH_OUT_TEMP_THRESHOLD) && !acIsEnabled) {
        // In case where the inner temperature is up to 1 degree
        // higher from the external temperature, open the window.
        if ((outerTemperature < innerTemperature <= outerTemperature + 1) && windowPos != 'u') {
            controlShutter(UPPER_END, HIGH, LOW);
            // Allow window opening
            openWindowIsAllowed = true;

        // In case where the inner temperature is lower more
        // than 1 degree from the external temperature, close the window.
        } else if ((outerTemperature - 1 < innerTemperature <= outerTemperature) && windowPos != 'l')
            controlShutter(LOWER_END, LOW, HIGH);

    // Else handle temperature using the air-condition with the window closed.
    } else {
        if (windowPos != 'l')
            controlShutter(LOWER_END, LOW, HIGH);
        
        // Enable air-condition according the inner temperature.
        if ((innerTemperature > HIGH_IN_TEMP_THRESHOLD) && !acIsEnabled)
            enableAircondition(true);
        else if ((innerTemperature < LOW_IN_TEMP_THRESHOLD) && !acIsEnabled)
            enableAircondition(false);
        
        // Do not allow window opening
        openWindowIsAllowed = false;
    }
}

// Check luminosity ratio and handle it if needed.
void handleLuminosity(void) {
    float luminosityRatio = 0;
    // Get luminosity ratio
    requestLuminosity(&luminosityRatio);

    // Re-new the values that represent the window position
    char windowPos = determineWindowPosition();
    
    // When the light intensity is inside this span, re-adjust window position for optimal light intensity
    if ((LOW_LUMINOSITY_THRESHOLD <= luminosityRatio <= HIGH_LUMINOSITY_THRESHOLD) && openWindowIsAllowed) {
        uint8_t middleThreshold = (HIGH_LUMINOSITY_THRESHOLD + LOW_LUMINOSITY_THRESHOLD) / 2;

        if ((LOW_LUMINOSITY_THRESHOLD <= luminosityRatio < middleThreshold) && windowPos != 'u')
            controlShutter(UPPER_END, HIGH, LOW); // Fully open the window
        else if (middleThreshold <= luminosityRatio <= HIGH_LUMINOSITY_THRESHOLD) {
            if (windowPos != 'm')
                // Set the window in the middle position by half-opening it or half-closing it
                if (windowPos == 'l')
                    controlShutter(MIDDLE, HIGH, LOW); // Half-open
                else if (windowPos == 'u')
                    controlShutter(MIDDLE, LOW, HIGH); // Half-close
        }
    } else {
        if (luminosityRatio < LOW_LUMINOSITY_THRESHOLD)
            digitalWrite(ARTIFICIAL_LIGHTING, HIGH);
        else if (luminosityRatio > HIGH_LUMINOSITY_THRESHOLD)
            digitalWrite(ARTIFICIAL_LIGHTING, LOW);
    }
}

// Check the water tank level and send to slave to enable buzzer if needed.
void handleWaterTank(void) {

}

void setup() {
    // Setup hardware and bluetooth serial
    sghSerial.setupHardwareSerial();
    sghSerial.setupBTSerial();

    // Set sleep timer
    // esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

    delay(1000);
    Serial.println("Master MCU is ready!");
}

void loop() {
    handleTemperature();
    handleLuminosity();
    handleWaterTank();

    Serial.println("Going to sleep now");
    delay(1000);
    // esp_light_sleep_start();
}
