/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: July 27th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14686

  This example shows how to use the SPI interface and print two records at the same time:
  Accel and Quat. 

  This example shows how to output the i/j/k/real parts of the rotation vector.
  https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation

  Hardware modifications:
  The PS1 jumper must be closed
  The PS0 jumper must be open. PS0/WAKE is connected and the WAK pin is used to bring the IC out of sleep.
  The I2C pull up jumper must be cleared/open

  Hardware Connections:
  Don't hook the BNO080 to a normal 5V Uno! Either use the Qwiic system or use a
  microcontroller that runs at 3.3V.
  Arduino 13 = BNO080 SCK
  12 = SO
  11 = SI
  10 = !CS
  9 = WAK
  8 = !INT
  7 = !RST
  3.3V = 3V3
  GND = GND
*/

#include <SPI.h>

#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 myIMU;

//These pins can be any GPIO
byte imuCSPin = 10;
byte imuWAKPin = 9;
byte imuINTPin = 8;
byte imuRSTPin = 7;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO080 SPI Read Example");

  myIMU.enableDebugging(Serial); //Pipe debug messages to Serial port

  if (myIMU.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin) == false)
  {
    Serial.println("BNO080 over SPI not detected. Are you sure you have all 6 connections? Freezing...");
    while (1);
  }

  //You can also call begin with SPI clock speed and SPI port hardware
  //myIMU.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin, 1000000);
  //myIMU.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin, 1000000, SPI1);

  //The IMU is now connected over SPI
  //Please see the other examples for library functions that you can call

  myIMU.enableAccelerometer(10); //Send data update at 100Hz
  myIMU.enableRotationVector(10); //Send data update at 100Hz

  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form i, j, k, real, accuracy"));
}

void loop()
{
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    float x = myIMU.getAccelX();
    float y = myIMU.getAccelY();
    float z = myIMU.getAccelZ();

    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

    Serial.print(x, 2);
    Serial.print(F(","));
    Serial.print(y, 2);
    Serial.print(F(","));
    Serial.print(z, 2);

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
