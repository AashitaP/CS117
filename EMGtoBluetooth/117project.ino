
#include <SPI.h>
#include "Adafruit_BLE_UART.h"

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 13

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

const int analogInPin = 9;  // Analog input pin that the potentiometer is attached to
int sensorValue = 0;        // value read from the pot

uint8_t sendBuffer[50];
int count = 0;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  while (!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));

  BTLEserial.setDeviceName("kpbdick"); /* 7 characters max! */

  BTLEserial.begin();
}

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

uint8_t s;

void loop() {
  // read the analog in value:
  sensorValue = analogRead(analogInPin);

  // print the results to the Serial Monitor:
  //Serial.print("sensor = ");
  //Serial.println(sensorValue);

  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
      Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
      Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
      Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  s = sensorValue;

  sendBuffer[count] = s;

  count++;

  Serial.print("Loop: "); Serial.println(count);

  //if buffer reaches 50 values and device connected, send data and reset buffer
  if (count == 50 && status == ACI_EVT_CONNECTED) {
    Serial.setTimeout(100); // 100 millisecond timeout

    // write the data
    BTLEserial.write(sendBuffer, 50);

    for (int i = 0; i < count; i++) {
      Serial.println(sendBuffer[i]);
    }

    count = 0;
    for (int i = 0; i < count; i++) {
      sendBuffer[i] = 0;
    }

  }


  //reset buffer if reaches max value of 50 and device still isn't connected
  if (count == 50 && status != ACI_EVT_CONNECTED) {
    count = 0;

    for (int i = 0; i < count; i++) {
      sendBuffer[i] = 0;
    }
  }

  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
  delay(200);
}
