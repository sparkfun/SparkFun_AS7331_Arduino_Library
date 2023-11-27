#include "SparkFun_AS7331.h"

SfeAS7331ArdI2C myUVSensor;

int8_t result = SFE_BUS_OK;

void setup() {
  Serial.begin(115200);
  while(!Serial){delay(100);};
  Serial.println("UV LED Example.");

  if(myUVSensor.begin() == false) {
    Serial.println("Sensor failed to begin. Please check your wiring!");
    Serial.println("Spinning...");
    while(1);
  }

  Serial.println("Sensor began.");

  if(myUVSensor.startMeasurement(MEAS_MODE_CMD) == false) {
    Serial.println("Sensor did not get set properly.");
    Serial.println("Spinning...");
    while(1);
  }

  Serial.println("Set mode to command.");

}

void loop() {
  
  if(SFE_BUS_OK != myUVSensor.setStartStateMode(START_STATE_ENABLED))
    Serial.println("Error starting reading!");
  
  delay(2+(1 << myUVSensor.getConfigConversionTime()));

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
