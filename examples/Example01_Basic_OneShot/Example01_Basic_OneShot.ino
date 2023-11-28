/*
  Using the AMS AS7331 Spectral UV Sensor in Command/One Shot (CMD) Mode.

  This example shows how operate the AS7331 in the default CMD mode. The start 
  command is sent, then delays until the conversion time has passed before
  reading out the UV values.

  By: Alex Brudner
  SparkFun Electronics
  Date: 2023/11/17
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> AS7331
  QWIIC --> QWIIC

  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/23517 - Qwiic 1x1
  https://www.sparkfun.com/products/23518 - Qwiic Mini
*/

#include "SparkFun_AS7331.h"

SfeAS7331ArdI2C myUVSensor;

int8_t result = SFE_BUS_OK;

void setup() {
  Serial.begin(115200);
  while(!Serial){delay(100);};
  Serial.println("UV LED Example.");

  // Initialize sensor and run default setup.
  if(myUVSensor.begin() == false) {
    Serial.println("Sensor failed to begin. Please check your wiring!");
    Serial.println("Spinning...");
    while(1);
  }

  Serial.println("Sensor began.");

  // Set measurement mode and change device operating mode to measure.
  if(myUVSensor.startMeasurement(MEAS_MODE_CMD) == false) {
    Serial.println("Sensor did not get set properly.");
    Serial.println("Spinning...");
    while(1);
  }

  Serial.println("Set mode to command.");

}

void loop() {
  
  // Send a start measurement command.
  if(SFE_BUS_OK != myUVSensor.setStartStateMode(START_STATE_ENABLED))
    Serial.println("Error starting reading!");
  
  // Wait for a bit longer than the conversion time.
  delay(2+(1 << myUVSensor.getConfigConversionTime()));

  // Read UV values.
  if(SFE_BUS_OK != myUVSensor.readAllUV())
    Serial.println("Error reading UV.");

  Serial.print("UVA:");
  Serial.print(myUVSensor.measures.uva);
  Serial.print(" UVB:");
  Serial.print(myUVSensor.measures.uvb);
  Serial.print(" UVC:");
  Serial.println(myUVSensor.measures.uvc);

  delay(2000);

};
