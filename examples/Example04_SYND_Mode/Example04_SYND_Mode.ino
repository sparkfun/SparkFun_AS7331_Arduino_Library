/*
  Using the AMS AS7331 Spectral UV Sensor in Synchronous Start and Stop (SYND) Mode.

  This example shows how operate the AS7331 in SYND mode. This uses the active 
  low SYN pin to both start and stop the conversion. The conversion time is
  calculated and stored in the `measures.outputConversionTime` field in units 
  of number of clock cycles.

  By: Alex Brudner
  SparkFun Electronics
  Date: 2023/11/28
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> AS7331
  QWIIC --> QWIIC
  27  --> SYN

  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/23517 - Qwiic 1x1
  https://www.sparkfun.com/products/23518 - Qwiic Mini
*/

#include "SparkFun_AS7331.h"

SfeAS7331ArdI2C myUVSensor;

int8_t result = SFE_BUS_OK;

const uint8_t synPin = 27;

void setup() {
  Serial.begin(115200);
  while(!Serial){delay(100);};
  Serial.println("AS7331 UV A/B/C Synchronous Start and End (SYND) mode example.");

  // Configure SYN pin.
  pinMode(synPin, OUTPUT);
  digitalWrite(synPin, HIGH); // Active low, so start high.

  // Initialize sensor and run default setup.
  if(myUVSensor.begin() == false) {
    Serial.println("Sensor failed to begin. Please check your wiring!");
    Serial.println("Spinning...");
    while(1);
  }

  Serial.println("Sensor began.");

  // Set measurement mode and change device operating mode to measure.
  if(myUVSensor.startMeasurement(MEAS_MODE_SYND) == false) {
    Serial.println("Sensor did not get set properly.");
    Serial.println("Spinning...");
    while(1);
  }

  Serial.println("Set mode to synchronous start/end (SYND). Starting measurement...");

  // Set device to be ready to measure.
  if(SFE_BUS_OK != myUVSensor.setStartStateMode(START_STATE_ENABLED))
    Serial.println("Error starting reading!");
    
  // Send start toggle.
  digitalWrite(synPin, LOW);
  delay(1);
  digitalWrite(synPin, HIGH);

}

void loop() {
  // Delay a random period of time from 64ms (minimum for full read) and 300ms.  
  delay(random(64,300));

  // End measurement.
  digitalWrite(synPin, LOW);
  delay(1);
  digitalWrite(synPin, HIGH);

  if(SFE_BUS_OK != myUVSensor.readAllUV())
    Serial.println("Error reading UV.");

  Serial.print("UVA:");
  Serial.print(myUVSensor.measures.uva);
  Serial.print(" UVB:");
  Serial.print(myUVSensor.measures.uvb);
  Serial.print(" UVC:");
  Serial.println(myUVSensor.measures.uvc);

  // Start next measurement.
  digitalWrite(synPin, LOW);
  delay(1);
  digitalWrite(synPin, HIGH);
}