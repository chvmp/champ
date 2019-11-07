/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This example shows how to use the stability classifier:
  Are we on a table, stationary, stable, or moving?

  It takes about 1ms at 400kHz I2C to read a record from the sensor, but we are polling the sensor continually
  between updates from the sensor. Use the interrupt pin on the BNO080 breakout to avoid polling.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 9600 baud to serial monitor.
*/

#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 myIMU;

void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.println("BNO080 Read Example");

  Wire.begin();

  myIMU.begin();

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableStabilityClassifier(50); //Send data update every 50ms

  //To enable stationary classifier we need to enable gyro calibration. Page 70.
  myIMU.calibrateGyro();

  Serial.println(F("Stability Classifier enabled"));
}

void loop()
{
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    byte classification = myIMU.getStabilityClassifier();

    if(classification == 0) Serial.print(F("Unknown classification"));
    else if(classification == 1) Serial.print(F("On table"));
    else if(classification == 2) Serial.print(F("Stationary"));
    else if(classification == 3) Serial.print(F("Stable"));
    else if(classification == 4) Serial.print(F("Motion"));
    else if(classification == 5) Serial.print(F("[Reserved]"));

    Serial.println();
  }
}
