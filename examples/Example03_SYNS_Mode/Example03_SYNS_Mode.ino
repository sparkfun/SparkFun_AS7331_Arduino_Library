#include "SparkFun_AS7331.h"

SfeAS7331ArdI2C myUVSensor;

int8_t result = SFE_BUS_OK;

const uint8_t synPin = 27;

const uint8_t interruptPin = 26;
volatile bool newDataReady = false;

void setup() {
  Serial.begin(115200);
  while(!Serial){delay(100);};
  Serial.println("AS7331 UV A/B/C Synchronous Start mode example.");

  // Configure SYN pin.
  pinMode(synPin, OUTPUT);
  digitalWrite(synPin, HIGH); // Active low, so start high.

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

  // Set measurement mode and change device operating mode to measure.
  if(myUVSensor.startMeasurement(MEAS_MODE_SYNS) == false) {
    Serial.println("Sensor did not get set properly.");
    Serial.println("Spinning...");
    while(1);
  }

  Serial.println("Set mode to synchronous. Starting measurement...");

  // Begin measurement.
  if(SFE_BUS_OK != myUVSensor.setStartStateMode(START_STATE_ENABLED))
    Serial.println("Error starting reading!");
    
  digitalWrite(synPin, LOW);
  delay(1);
  digitalWrite(synPin, HIGH);

}

void loop() {

  if(newDataReady) {
    newDataReady = false;

    if(SFE_BUS_OK != myUVSensor.readAllUV())
      Serial.println("Error reading UV.");

    digitalWrite(synPin, LOW);
    delay(1);
    digitalWrite(synPin, HIGH);
  
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