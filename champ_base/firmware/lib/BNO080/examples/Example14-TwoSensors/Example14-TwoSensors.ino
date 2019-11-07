/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This example shows how to output the i/j/k/real parts of the rotation vector.
  https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation

  It takes about 1ms at 400kHz I2C to read a record from the sensor, but we are polling the sensor continually
  between updates from the sensor. Use the interrupt pin on the BNO080 breakout to avoid polling.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 9600 baud to serial monitor.
*/

#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 myIMU1; //Open I2C ADR jumper - goes to address 0x4B
BNO080 myIMU2; //Closed I2C ADR jumper - goes to address 0x4A

void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.println("BNO080 Read Example");

  Wire.begin();
  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  //When a large amount of time has passed since we last polled the sensors
  //they can freeze up. To un-freeze it is easiest to power cycle the sensor.

  //Start 2 sensors
  if (myIMU1.begin(0x4B) == false)
  {
    Serial.println("First BNO080 not detected with I2C ADR jumper open. Check your jumpers and the hookup guide. Freezing...");
    while(1);
  }

  if (myIMU2.begin(0x4A) == false)
  {
    Serial.println("Second BNO080 not detected with I2C ADR jumper closed. Check your jumpers and the hookup guide. Freezing...");
    while(1);
  }

  myIMU1.enableRotationVector(50); //Send data update every 50ms
  myIMU2.enableRotationVector(50); //Send data update every 50ms

  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form i, j, k, real, accuracy"));
}

void loop()
{
  //Look for reports from the IMU
  if (myIMU1.dataAvailable() == true)
  {
    float quatI = myIMU1.getQuatI();
    float quatJ = myIMU1.getQuatJ();
    float quatK = myIMU1.getQuatK();
    float quatReal = myIMU1.getQuatReal();
    float quatRadianAccuracy = myIMU1.getQuatRadianAccuracy();

    //Serial.print("First:");
    Serial.print(quatI, 2);
    Serial.print(F(","));
    Serial.print(quatJ, 2);
    Serial.print(F(","));
    Serial.print(quatK, 2);
    Serial.print(F(","));
    Serial.print(quatReal, 2);
    Serial.print(F(","));
    Serial.print(quatRadianAccuracy, 2);
    Serial.print(F(","));

    //Serial.println();
  }

  if (myIMU2.dataAvailable() == true)
  {
    float quatI = myIMU2.getQuatI();
    float quatJ = myIMU2.getQuatJ();
    float quatK = myIMU2.getQuatK();
    float quatReal = myIMU2.getQuatReal();
    float quatRadianAccuracy = myIMU2.getQuatRadianAccuracy();

    //Serial.print("Second:");
    Serial.print(quatI, 2);
    Serial.print(F(","));
    Serial.print(quatJ, 2);
    Serial.print(F(","));
    Serial.print(quatK, 2);
    Serial.print(F(","));
    Serial.print(quatReal, 2);
    Serial.print(F(","));
    Serial.print(quatRadianAccuracy, 2);
    Serial.print(F(","));

    Serial.println();
  }
}
