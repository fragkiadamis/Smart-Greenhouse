#include "master.h"

// Initialize communication protocol and serial handling.
SmartGreenHouseSerial sghSerial;

// Initialize WiFi MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// State flags.
bool mode = false;
bool acIsEnabled = false;
bool coolingIsEnabled = false;
bool heatingIsEnabled = false;
bool humControlIsOn = false;
bool openWindowIsAllowed = true;

// Send message via bluetooth and wait for a response.
String sendCmdAndGetRes(String cmd) {
    sghSerial.send(cmd);
    while (!sghSerial.hasMessage());
    return sghSerial.receive();
}

// Request the inside and the outside temperature.
void requestTemperatures(float *innerTemperature, float *outerTemperature) {
    String cmd = "";

    // Read inner and outer temperature.
    cmd = String(sghSerial.DHT_SENS) + '|' + String(sghSerial.INNER_TEMP);    // Create command,
    *innerTemperature = sendCmdAndGetRes(cmd).toFloat();                      // And send it.
    cmd = String(String(sghSerial.OUTER_TEMP));
    *outerTemperature = sendCmdAndGetRes(cmd).toFloat();
}

// Request the inside / outside luminosity ratio of the greenhouse.
void requestLuminosity(float *luminosityRatio) {
    String cmd = "";

    // Read inner and outer temperature.
    cmd = String(String(sghSerial.LUM));                   // Create command,
    *luminosityRatio = sendCmdAndGetRes(cmd).toFloat();    // And send it.
}

// Request the humidity level inside the greenhouse.
void requestHumidity(float *humidity) {
    String cmd = "";

    // Read inner and outer temperature.
    cmd = String(sghSerial.DHT_SENS) + '|' + String(sghSerial.HUMIDITY);    // Create command,
    *humidity = sendCmdAndGetRes(cmd).toFloat();                            // And send it.
}

// Toggle irrigation.
void toggleIrrigation(bool state) {
    String cmd = "";

    // Read inner and outer temperature.
    cmd = String(sghSerial.IRG) + '|' + String(state);    // Create command,
    String res = sendCmdAndGetRes(cmd);                   // And send it.
}

// Toggle buzzer.
void toggleBuzzer(bool state) {
    String cmd = "";

    // Read inner and outer temperature.
    cmd = String(sghSerial.BZ) + '|' + String(state);     // Create command,
    String res = sendCmdAndGetRes(cmd);                   // And send it.
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

// Enable humidification / dehumidification process.
void enableHumidityControl(bool humidify) {
    if (humidify)
        digitalWrite(AC_HUMIDIFICATION, HIGH);
    else
        digitalWrite(AC_DEHUMIDIFICATION, HIGH);
}

// Stop humidification / dehumidification process.
void stopHumidityControl(void) {
    digitalWrite(AC_HUMIDIFICATION, LOW);
    digitalWrite(AC_DEHUMIDIFICATION, LOW);
}

// Check the temperature and handle it if needed.
void handleTemperature(void) {
    // Initialize variables for atmospheric values.
    float innerTemperature = 0, outerTemperature = 0;
    // Get temperatures.
    requestTemperatures(&innerTemperature, &outerTemperature);
    #ifdef DEBUG
    Serial.println("Inner temperature ratio: " + String(innerTemperature));
    Serial.println("Outer temperature ratio: " + String(outerTemperature));
    #endif

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
    #ifdef DEBUG
    Serial.println("Luminosity ratio: " + String(luminosityRatio));
    #endif

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

// Check the humidity level and handle it if needed.
void handleHumidity(void) {
    float humidity = 0;
    // Get humidity level
    requestHumidity(&humidity);
    #ifdef DEBUG
    Serial.println("Humidity: " + String(humidity));
    #endif

    if ((LOW_HUMIDITY_THRESHOLD <= humidity <= HIGH_HUMIDITY_THRESHOLD) && humControlIsOn)
        stopHumidityControl();
    else if ((humidity < LOW_HUMIDITY_THRESHOLD) && !humControlIsOn)
        enableHumidityControl(true);
    else if (humidity > HIGH_HUMIDITY_THRESHOLD && !humControlIsOn)
        enableHumidityControl(false);
}

// Check the water tank level and handle it if needed.
void handleWaterTank(void) {
    uint8_t reading = touchRead(WATER_TANK);
    float mls = 9.646 * pow(10, -6) * reading + 2.9110 * pow(10, -4);
    float percentage = (mls / (float)MAX_LEVEL) * 100.0;

    if (percentage < WT_LOW_THRESHOLD)
        toggleBuzzer(true);
    else if (percentage >= WT_STOP_THRESHOLD)
        toggleBuzzer(false);
}

// Setup WiFi.
void setupWiFi(void) {
    #ifdef DEBUG
    Serial.print("Connecting to ");
    Serial.println(SSID);
    #endif

    WiFi.begin(SSID, PWD);
    while (WiFi.status() != WL_CONNECTED) {
        #ifdef DEBUG
        delay(500);
        Serial.print("Connecting");
        Serial.print(".");
        #endif
    }

    #ifdef DEBUG
    Serial.println();
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    #endif
}

// Reconnect to MQTT server.
void reconnect(void) {
    // Loop until we're reconnected
    while (!client.connected()) {
        #ifdef DEBUG
        Serial.println("Attempting MQTT connection...");
        #endif
        // Attempt to connect
        if (client.connect("ESP32Client")) {
            #ifdef DEBUG
            Serial.println("connected");
            #endif
            // Subscribe
            client.subscribe(MODE);
            client.subscribe(AC_TEMP);
            client.subscribe(AC_HUM);
            client.subscribe(IRR);
            client.subscribe(SHTR);
        } else {
            #ifdef DEBUG
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            #endif
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

// Write byte to EEPROM specified address.
void writeByteToEEPROM(uint16_t address, bool byte) {
    EEPROM.write(address, byte);
    EEPROM.commit();
}

#ifdef INITIALIZE_EEPROM
// Initialize EEPROM to zero's
void initializeEEPROM(void) {
    for (uint16_t i = 0; i < EEPROM_SIZE; i++)
        EEPROM.write(i, 0);
    EEPROM.commit();
}
#endif

// MQTT callback function.
void executeMQTT(char* topic, byte* message, unsigned int length) {
    #ifdef DEBUG
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    #endif

    // Get command string value.
    String cmd;
    for (int i = 0; i < length; i++) {
        cmd += (char)message[i];
        #ifdef DEBUG
        Serial.print((char)message[i]);
        #endif
    }
    #ifdef DEBUG
    Serial.println();
    #endif

    // Handle the message by topic.
    if (String(topic) == MODE)
        writeByteToEEPROM(MODE_ADDRESS, (cmd == "true") ? 1 : 0);
    else if (String(topic) == AC_TEMP) {
        Serial.println("temptest");
        if (cmd == TEMP_OFF)
            stopAircondition();
        else
            enableAircondition((cmd == COOL_MODE) ? 1 : 0);
    } else if (String(topic) == AC_HUM) {
        Serial.println("humtest");
        if (cmd == HUM_OFF)
            stopHumidityControl();
        else
            enableHumidityControl((cmd == HUM_MODE) ? 1 : 0);
    } else if (String(topic) == IRR)
        toggleIrrigation((cmd == "true") ? 1 : 0);
    else if (String(topic) == SHTR) {
        // Initialze window position and determine window position.
        char windowPos = determineWindowPosition();
        // If window is already in the desired position, do nothing.
        if ((cmd == "close" && windowPos == 'l') || (cmd == "mid" && windowPos == 'm')  || (cmd == "open" && windowPos == 'u'))
            return;
        
        if (cmd == "close")
            controlShutter(LOWER_END, LOW, HIGH);
        else if (cmd == "mid")
            if (windowPos == 'u')
                controlShutter(MIDDLE, LOW, HIGH);
            else if (windowPos == 'l')
                controlShutter(MIDDLE, HIGH, LOW);
        else if (cmd == "open")
            controlShutter(UPPER_END, HIGH, LOW);
    }
}

void setup() {
    // Setup hardware and bluetooth serial.
    #ifdef DEBUG
    sghSerial.setupHardwareSerial();
    #endif
    sghSerial.setupBTSerial();

    #ifdef INITIALIZE_EEPROM
    initializeEEPROM();
    #endif
  
    // initialize EEPROM with predefined size.
    EEPROM.begin(EEPROM_SIZE);
    // Read the mode that the master works.
    mode = EEPROM.read(MODE_ADDRESS);

    setupWiFi();
    client.setServer(MQTT_SERVER, PORT);
    client.setCallback(executeMQTT);

    // Set sleep timer.
    // esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

    #ifdef DEBUG
    Serial.println("Master MCU is ready!");
    String startupMode = (mode) ? "automatic" : "manual";
    Serial.println("Working in " + startupMode + " mode");
    #endif
}

void loop() {
    // Check for MQTT messages
    if (!client.connected())
        reconnect();
    client.loop();

    // If the automatic mode is on, handle the greenhouse according to the automation rules.
    if (mode) {
        // handleTemperature();
        // handleLuminosity();
        // handleHumidity();
        // handleWaterTank();
    }
    
    // Fall asleep
    // esp_light_sleep_start();
}
