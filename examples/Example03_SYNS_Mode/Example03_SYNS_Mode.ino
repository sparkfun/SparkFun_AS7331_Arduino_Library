/*
  Using the AMS AS7331 Spectral UV Sensor in Synchronous Start (SYNS) Mode.

  This example shows how operate the AS7331 in SYNS mode. This uses the active 
  low SYN pin to start the conversion and relies on an interrupt to signal the
  end of conversion.

  By: Alex Brudner
  SparkFun Electronics
  Date: 2023/11/27
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> AS7331
  QWIIC --> QWIIC
  26  --> INT
  27  --> SYN

  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/23517 - Qwiic 1x1
  https://www.sparkfun.com/products/23518 - Qwiic Mini
*/

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_AS7331.h>

SfeAS7331ArdI2C myUVSensor;

const uint8_t synPin = 27;

const uint8_t interruptPin = 26;
volatile bool newDataReady = false;

void setup() {
  Serial.begin(115200);
  while(!Serial){delay(100);};
  Serial.println("AS7331 UV A/B/C Synchronous Start mode example.");

  Wire.begin();

  // Configure SYN pin.
  pinMode(synPin, OUTPUT);
  digitalWrite(synPin, HIGH); // Active low, so start high.

  // Configure Interrupt.
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), dataReadyInterrupt, RISING);

  // Initialize sensor and run default setup.
  if(myUVSensor.begin() == false) {
    Serial.println("Sensor failed to begin. Please check your wiring!");
    Serial.println("Halting...");
    while(1);
  }

  Serial.println("Sensor began.");

  // Set measurement mode and change device operating mode to measure.
  if(myUVSensor.prepareMeasurement(MEAS_MODE_SYNS) == false) {
    Serial.println("Sensor did not get set properly.");
    Serial.println("Halting...");
    while(1);
  }

  Serial.println("Set mode to synchronous start (SYNS). Starting measurement...");

  // Set device to be ready to measure.
  if(kSTkErrOk != myUVSensor.setStartState(true))
    Serial.println("Error starting reading!");
    
  // Send start toggle.
  digitalWrite(synPin, LOW);
  delay(1);
  digitalWrite(synPin, HIGH);

}

void loop() {

  // If an interrupt has been generated...
  if(newDataReady) {
    newDataReady = false;

    if(kSTkErrOk != myUVSensor.readAllUV())
      Serial.println("Error reading UV.");

    // Start next measurement
    digitalWrite(synPin, LOW);
    delay(1);
    digitalWrite(synPin, HIGH);
  
    Serial.print("UVA:");
    Serial.print(myUVSensor.getUVA());
    Serial.print(" UVB:");
    Serial.print(myUVSensor.getUVB());
    Serial.print(" UVC:");
    Serial.println(myUVSensor.getUVC());
  }

}

void dataReadyInterrupt() {
  newDataReady = true;
}