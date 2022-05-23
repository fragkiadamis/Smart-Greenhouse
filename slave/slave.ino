// Include nescessary files
#include <SoftwareSerial.h>
#include <avr/sleep.h>
#include "slave.h"

// Setup software serial pins
SoftwareSerial BTSerial(BT_RX, BT_TX);

void startSerials(void) {
  // Setup hardware serial & bluetooth serial
  Serial.begin(BAUD_RATE);
  while(!Serial);
  BTSerial.begin(BT_BAUD_RATE);
  while(!BTSerial);
}

// Arduino falls into deep sleep
void deepSleep(void) {
  sleep_enable();//Enabling sleep mode
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), wakeUp, CHANGE);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);//Setting the sleep mode, in our case full sleep
  sleep_cpu();//activating sleep mode
}

// Read the data from a given serial
void readSerialData(char *buffer) {
  uint8_t size = 0;
  unsigned long time = micros();

  while(micros() - time <= SERIAL_TIMEOUT) {
    if (BTSerial.available()) {
      char c = BTSerial.read();
      
      // If the buffer is almost full or end of line, stop
      if(size == (SERIAL_BUFFER_SIZE - 1) || c == '\n')
        break;
        
      buffer[size++] = c;
      time = micros(); // Every time a new character arrives update the timeout
    }
  }
  buffer[size] = '\0';
}

// Toggle buzzer (turn ON or OFF)
void toggleBuzzer(uint8_t pin, bool buzz) {
  if (buzz)
    return tone(pin, BUZZER_FREQ);

  noTone(pin);
}

// Write to pins
void writeToPin(char pinQuality, char pin, uint8_t value) {
  // Analog pin
  if (pinQuality == 'A')
    analogWrite(pin, map(value, 0, 255, 0, 1024));
  
  // Digital pin
  else if (pinQuality == 'D') {
    if (pin == BUZZER_PIN) // If it's the buzzer then buzzzz
      return toggleBuzzer(pin, value);

    // Else change pin state
    digitalWrite(pin, value = (value) ? HIGH : LOW); // In digital we only care for 0 and 1
  }
}

// Extract the command from the string and execute it
void executeCommand(char *buffer) {
  // Extract command (READ or WRITE | DIGITAL or ANALOG | PIN | VALUE)
  char action = (String(buffer).substring(0, 1)).charAt(0);
  char pinQuality = (String(buffer).substring(1, 2)).charAt(0);
  uint8_t pin = (String(buffer).substring(2, 4)).toInt();
  uint8_t value = (String(buffer).substring(5)).toInt();


  // Execute command
  if (action == 'W')
    writeToPin(pinQuality, pin, value);
  // else if (action == 'R')
  //   return readFromPin(pinQuality, pin);
}

void setup() {
  startSerials();
}

void loop() {
  // Fall into deep sleep
  // deepSleep();
  // delay(200);

  // If BTSerial has available data
  if (BTSerial.available()) {
    char cmd[SERIAL_BUFFER_SIZE] = {0};
    readSerialData(cmd);
    executeCommand(cmd);
  }
}

// Interrupt from sleep
void wakeUp(void) {
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
  startSerials();
}
