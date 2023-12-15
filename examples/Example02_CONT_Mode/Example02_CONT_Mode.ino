/*
  Using the AMS AS7331 Spectral UV Sensor in Continuous (CONT) Mode.

  This example shows how operate the AS7331 in CONT mode. The break time
  register sets the delay between measurements so that the processor can read
  out the results without interfering with the ADC.

  By: Alex Brudner
  SparkFun Electronics
  Date: 2023/11/27
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> AS7331
  QWIIC --> QWIIC
  26  --> INT

  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/23517 - Qwiic 1x1
  https://www.sparkfun.com/products/23518 - Qwiic Mini
*/

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_AS7331.h>

SfeAS7331ArdI2C myUVSensor;

const uint8_t interruptPin = 26;
volatile bool newDataReady = false;

void setup() {
  Serial.begin(115200);
  while(!Serial){delay(100);};
  Serial.println("AS7331 UV A/B/C Continuous mode example.");

  Wire.begin();

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

  // Set the delay between measurements so that the processor can read out the 
  // results without interfering with the ADC.
  // Set break time to 900us (112 * 8us) to account for the time it takes to poll data.
  if(kSTkErrOk != myUVSensor.setBreakTime(112)) {
    Serial.println("Sensor did not set break time properly.");
    Serial.println("Halting...");
    while(1);
  }

  // Set measurement mode and change device operating mode to measure.
  if(myUVSensor.prepareMeasurement(MEAS_MODE_CONT) == false) {
    Serial.println("Sensor did not get set properly.");
    Serial.println("Spinning...");
    while(1);
  }

  Serial.println("Set mode to continuous. Starting measurement...");

  // Begin measurement.
  if(kSTkErrOk != myUVSensor.setStartState(true))
    Serial.println("Error starting reading!");

}

void loop() {

  // If an interrupt has been generated...
  if(newDataReady) {
    newDataReady = false;

    if(kSTkErrOk != myUVSensor.readAllUV())
      Serial.println("Error reading UV.");
  
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