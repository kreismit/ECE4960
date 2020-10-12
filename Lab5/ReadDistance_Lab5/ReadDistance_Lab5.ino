#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>

/*
  Reading distance from the laser based VL53L1X
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 4th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14667

  This example prints the distance to an object.

  Are you getting weird readings? Be sure the vacuum tape has been removed from the sensor.
*/

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

void setup(void)
{
  Wire.begin();

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  
  //distanceSensor.setDistanceModeShort();
  //distanceSensor.setTimingBudgetInMs(60);       // ideal for SR
  distanceSensor.setDistanceModeLong();           // default
  distanceSensor.setTimingBudgetInMs(180);        // ideal for LR
  //distanceSensor.setIntermeasurementPeriod(100);  // default
  Serial.println("Sensor online!");
  Serial.printf("%d ms between measurements.\n",distanceSensor.getIntermeasurementPeriod());
  Serial.printf("Signal intensity threshold: %5.4f \n",distanceSensor.getSignalThreshold());
  Serial.printf("Standard deviation threshold: %5.4f \n",distanceSensor.getSigmaThreshold());
  distanceSensor.setOffset(37);    // Set ranging offset from calibration (mm)
}

void loop(void)
{
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  uint32_t rangingStart = micros();             // Log initial time to measure ranging time
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  int distance = distanceSensor.getDistance();  //Get the result of the measurement from the sensor
  int rangingTime = micros()-rangingStart;      // Record time required to get measurement
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  Serial.print("Distance(mm): ");
  Serial.print(distance);

  float distanceInches = distance * 0.0393701;
  float distanceFeet = distanceInches / 12.0;

  Serial.print("\tDistance(ft): ");
  Serial.print(distanceFeet, 2);

  Serial.print("\tRanging time (us): ");
  Serial.print(rangingTime);

  Serial.println();
}
