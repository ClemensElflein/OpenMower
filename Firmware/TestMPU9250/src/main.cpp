/**
 * This software is used to check MPU9250 modules if they are fake or real.
 * If real, LED will stay lit, if fake, the LED will blink.
 * Pins:
 * VCC=5V
 * GND=GND
 * SCL=GP18
 * SDA=GP19
 * AD0=GP16
 * NCS=GP17
 */

#include "Arduino.h"
#include <MPU9250.h>

MPU9250 IMU(SPI, 17);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
}

bool on = false;

void loop()
{
    
  int status = IMU.begin();
  if (status < 0)
  {
    on = !on;
    Serial.println("Fake");
  }
  else
  {
    on = true;
      digitalWrite(LED_BUILTIN, on);

    Serial.println("Real");
    for(int i = 0; i < 100; i++) {
        IMU.readSensor();
  // display the data
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.print("\t");
  Serial.println(IMU.getTemperature_C(),6);
  delay(100);
    }
  }
  digitalWrite(LED_BUILTIN, on);
  delay(100);
}