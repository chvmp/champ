/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This example shows how to read the Q values and other metadata for the accelerometer.

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
  Serial.println("BNO080 Reading Metadata Example");

  Wire.begin();

  myIMU.begin();

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  //Get Q1 for rotation vector and accelerometer
  int rotationVector_Q1 = myIMU.getQ1(FRS_RECORDID_ROTATION_VECTOR);
  int accelerometer_Q1 = myIMU.getQ1(FRS_RECORDID_ACCELEROMETER);

  Serial.println();
  Serial.println("For rotation vector");
  Serial.print("Range: ");
  Serial.println(myIMU.getRange(FRS_RECORDID_ROTATION_VECTOR), 4);
  Serial.print("Resolution: ");
  Serial.println(myIMU.getResolution(FRS_RECORDID_ROTATION_VECTOR), 10);
  Serial.print("Q1: ");
  Serial.println(myIMU.getQ1(FRS_RECORDID_ROTATION_VECTOR));
  Serial.print("Q2: ");
  Serial.println(myIMU.getQ2(FRS_RECORDID_ROTATION_VECTOR));
  Serial.print("Q3: ");
  Serial.println(myIMU.getQ3(FRS_RECORDID_ROTATION_VECTOR));

  Serial.println();
  Serial.println("For accelerometer");
  Serial.print("Range: ");
  Serial.println(myIMU.getRange(FRS_RECORDID_ACCELEROMETER), 4);
  Serial.print("Resolution: ");
  Serial.println(myIMU.getResolution(FRS_RECORDID_ACCELEROMETER), 4);
  Serial.print("Q1: ");
  Serial.println(myIMU.getQ1(FRS_RECORDID_ACCELEROMETER));
  Serial.print("Q2: ");
  Serial.println(myIMU.getQ2(FRS_RECORDID_ACCELEROMETER));
  Serial.print("Q3: ");
  Serial.println(myIMU.getQ3(FRS_RECORDID_ACCELEROMETER));

  //Example of reading meta data manually
  //See page 30 of the reference manual
  Serial.println();
  uint16_t accelerometer_power = myIMU.readFRSword(FRS_RECORDID_ACCELEROMETER, 3) & 0xFFFF; //Get word 3, lower 16 bits
  float accel_power = myIMU.qToFloat(accelerometer_power, 10); //Q point is 10 for power
  Serial.print("Accelerometer power: ");
  Serial.print(accel_power, 5);
  Serial.println(" (mA)");
}

void loop()
{
}






//Pretty prints the contents of the current shtp packet
/*void printPacket(void)
  {
  uint16_t packetLength = (uint16_t)myIMU.shtpHeader[1] << 8 | myIMU.shtpHeader[0];

  if (packetLength & 1 << 15)
  {
    Serial.println(F("Continued packet"));
    packetLength &= ~(1 << 15);
  }

  Serial.print(F("Packet Length: "));
  Serial.println(packetLength);

  //Attempt to pre-print known events
  if (myIMU.shtpHeader[2] == CHANNEL_EXECUTABLE && myIMU.shtpData[0] == 0x01) {
    Serial.println(F("Known: Reset complete"));
    return;
  }
  if (myIMU.shtpHeader[2] == CHANNEL_CONTROL && myIMU.shtpData[0] == 0xF1) {
    Serial.println(F("Known: Command response"));
  }
  if (myIMU.shtpHeader[2] == CHANNEL_COMMAND && myIMU.shtpData[0] == 0x00) {
    Serial.println(F("Known: Init Advertisement"));
    return;
  }

  if (myIMU.shtpHeader[2] == CHANNEL_REPORTS && myIMU.shtpData[0] == 0xFB) {
    Serial.println(F("Known: Data report"));
    printDataReport();
    return;
  }

  Serial.print(F("Channel: "));
  if (myIMU.shtpHeader[2] == 0) Serial.println(F("Command"));
  else if (myIMU.shtpHeader[2] == 1) Serial.println(F("Executable"));
  else if (myIMU.shtpHeader[2] == 2) Serial.println(F("Control"));
  else Serial.println(myIMU.shtpHeader[2]);
  Serial.print("SeqNum: ");
  Serial.println(myIMU.shtpHeader[3]);

  int dataLength = packetLength - 4;
  if (dataLength > 10) dataLength = 10; //Arbitrary limiter. We don't want the phonebook.
  for (int i = 0 ; i < dataLength ; i++)
  {
    Serial.print(i);
    Serial.print(F(") 0x"));
    if (myIMU.shtpData[i] < 0x10) Serial.print("0");
    Serial.print(myIMU.shtpData[i], HEX);
    Serial.println();
  }
  }*/
