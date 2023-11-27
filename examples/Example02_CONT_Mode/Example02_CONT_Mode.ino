#include "SparkFun_AS7331.h"

SfeAS7331ArdI2C myUVSensor;

int8_t result = SFE_BUS_OK;

const uint8_t interruptPin = 26;
volatile bool newDataReady = false;

void setup() {
  Serial.begin(115200);
  while(!Serial){delay(100);};
  Serial.println("AS7331 UV A/B/C Continuous mode example.");

  // Configure Interrupt.
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), dataReadyInterrupt, RISING);

  // Initialize sensor and run default setup.
  if(myUVSensor.begin() == false) {
    Serial.println("Sensor failed to begin. Please check your wiring!");
    Serial.println("Spinning...");
    while(1);
  }

  Serial.println("Sensor began.");

  // Set break time to 900us (112 * 8us) to account for the time it takes to poll data.
  if(SFE_BUS_OK != myUVSensor.setBreakTime(112)) {
    Serial.println("Sensor did not set break time properly.");
    Serial.println("Spinning...");
    while(1);
  }

  // Set measurement mode and change device operating mode to measure.
  if(myUVSensor.startMeasurement(MEAS_MODE_CONT) == false) {
    Serial.println("Sensor did not get set properly.");
    Serial.println("Spinning...");
    while(1);
  }

  Serial.println("Set mode to continuous. Starting measurement...");

  // Begin measurement.
  if(SFE_BUS_OK != myUVSensor.setStartStateMode(START_STATE_ENABLED))
    Serial.println("Error starting reading!");

}

void loop() {

  if(newDataReady) {
    newDataReady = false;

    if(SFE_BUS_OK != myUVSensor.readAllUV())
      Serial.println("Error reading UV.");
  
    Serial.print("UVA:");
    Serial.print(myUVSensor.measures.uva);
    Serial.print(" UVB:");
    Serial.print(myUVSensor.measures.uvb);
    Serial.print(" UVC:");
    Serial.println(myUVSensor.measures.uvc);
  }

}

void dataReadyInterrupt() {
  newDataReady = true;
}