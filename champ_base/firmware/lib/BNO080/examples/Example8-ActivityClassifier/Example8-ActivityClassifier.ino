/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This is a fun one! The BNO080 can guess at what activity you are doing:
  In vehicle
  On bicycle
  On foot
  Still
  Tilting
  Walking
  Running
  On stairs
  This example shows how to read the confidence levels of each activity

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

byte activityConfidences[9]; //This array will be filled with the confidence levels of each possible activity

void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.println("BNO080 Read Example");

  Wire.begin();

  myIMU.begin();

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  //See page 73 of reference manual. There is a 32 bit word where each
  //bit enables a different possible activity. Currently there are only 8
  //possible. Let's enable all of them!
  uint32_t enableActivities = 0x1F; //Enable all 9 possible activities including Unknown

  //Send data update every 50ms, with sensor specific config word
  //Pass in a pointer to our activityConfidences array as well
  myIMU.enableActivityClassifier(50, enableActivities, activityConfidences); 

  Serial.println(F("Activity Classifier enabled"));
}

void loop()
{
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    //getActivityClassifier will modify our activityConfidences array
    //It will return the most likely activity as well.
    byte mostLikelyActivity = myIMU.getActivityClassifier(); 

    Serial.print("Most likely activity: ");
    printActivityName(mostLikelyActivity);
    Serial.println();

    Serial.println("Confidence levels:");
    for(int x = 0 ; x < 9 ; x++)
    {
      printActivityName(x);
      Serial.print(F(") "));
      Serial.print(activityConfidences[x]);
      Serial.print(F("%"));
      Serial.println();
    }

    Serial.println();
  }
}

//Given a number between 0 and 8, print the name of the activity
//See page 73 of reference manual for activity list
void printActivityName(byte activityNumber)
{
  if(activityNumber == 0) Serial.print("Unknown");
  else if(activityNumber == 1) Serial.print("In vehicle");
  else if(activityNumber == 2) Serial.print("On bicycle");
  else if(activityNumber == 3) Serial.print("On foot");
  else if(activityNumber == 4) Serial.print("Still");
  else if(activityNumber == 5) Serial.print("Tilting");
  else if(activityNumber == 6) Serial.print("Walking");
  else if(activityNumber == 7) Serial.print("Running");
  else if(activityNumber == 8) Serial.print("On stairs");
}
