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
#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

//Global variables needed for PDM library
#define pdmDataBufferSize 4096 //Default is array of 4096 * 32bit
uint16_t pdmDataBuffer[pdmDataBufferSize];

//Global variables needed for the FFT in this sketch
float g_fPDMTimeDomain[pdmDataBufferSize * 2];
float g_fPDMFrequencyDomain[pdmDataBufferSize * 2];
float g_fPDMMagnitudes[pdmDataBufferSize * 2];
uint32_t sampleFreq;

//Enable these defines for additional debug printing
#define PRINT_PDM_DATA 0
#define PRINT_FFT_DATA 0
#include <PDM.h>        // PDM library included with Arduino_Apollo3 core (for mic)
AP3_PDM myPDM;   //Create instance of PDM class

//Math library needed for FFT
#define ARM_MATH_CM4
#include <arm_math.h>

#define LEDPIN 13

SCMD myMotorDriver; // This creates the main object of one motor driver and connected slaves.


// Global variables for driving (and detecting sound)
bool whistle;             // Does the board hear a whistling frequency?
int right=0; int left=1;  // addresses for R and L drive banks
int power;                // power to send to motors
int powerTurn = 255;      // power to send to motors when spinning in place
float m=0.08;              // proportionality constant: y (power) = mx+b where x is the distance sensed
int b=20;                 // b in the linear proportionality relation
int brake;
int offset = 4;           // left side gets (offset) more power than right side
float calib = 1.08;        // left side gets (calib) times more power than right
int t;                    // for open-loop timing

void setup(void)
{
  Wire.begin();

  //Serial.begin(115200);

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    //Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  
  //distanceSensor.setDistanceModeShort();
  //distanceSensor.setTimingBudgetInMs(60);       // ideal for SR
  distanceSensor.setDistanceModeLong();           // default
  distanceSensor.setTimingBudgetInMs(180);        // ideal for LR
  //distanceSensor.setIntermeasurementPeriod(100);  // default
  //Serial.println("Sensor online!");
  //Serial.printf("%d ms between measurements.\n",distanceSensor.getIntermeasurementPeriod());
  //Serial.printf("Signal intensity threshold: %5.4f \n",distanceSensor.getSignalThreshold());
  //Serial.printf("Standard deviation threshold: %5.4f \n",distanceSensor.getSigmaThreshold());
  distanceSensor.setOffset(37);    // Set ranging offset from calibration (mm)

  //////// Initialize PDM so we can control this by whistling! (TODO)
  
  if (myPDM.begin() == false) // Turn on PDM with default settings, start interrupts
  {
    //Serial.println("PDM Init failed. Are you sure these pins are PDM capable?");
    while (1);  // halt the code here
  }
  else
    //Serial.println("PDM initialized successfully.");
  //printPDMConfig();

  //////// Initialize motor driver so we can drive!
  
  //***** Configure the Motor Driver's Settings *****//
  //  .commInter face can be I2C_MODE or SPI_MODE
  myMotorDriver.settings.commInterface = I2C_MODE;
  //myMotorDriver.settings.commInterface = SPI_MODE;

  //  set address if I2C configuration selected with the config jumpers
  myMotorDriver.settings.I2CAddress = 0x5D; //config pattern "0101" on board for address 0x5A

  //  set chip select if SPI selected with the config jumpers
  myMotorDriver.settings.chipSelectPin = 10;

  //*****initialize the driver get wait for idle*****//
  while ( myMotorDriver.begin() != 0xA9 ) //Wait until a valid ID word is returned
  {
    //Serial.println( "ID mismatch, trying again" );
    delay(500);
  }
  //Serial.println( "ID matches 0xA9" );

  //  Check to make sure the driver is done looking for slaves before beginning
  //Serial.print("Waiting for enumeration...");
  while ( myMotorDriver.ready() == false );

  //*****Set application settings and enable driver*****//

  while ( myMotorDriver.busy() );
  myMotorDriver.enable();
}

void loop(void)
{
  // Update current range reading
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    myPDM.getData(pdmDataBuffer, pdmDataBufferSize); // keep checking PDM
    whistle = loudestFreq();                         // do we hear a whistle?    
  }
  int distance = distanceSensor.getDistance();  //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
  power = ((float) m*distance)+b; // calculate power proportional to distance from obstacle
  brake = power/2;                // this is imprecise (integer divide) but it doesn't need to be precise

  // Go to Deep Sleep until the PDM ISR or other ISR wakes us.
  //am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
  //***** Operate the Motor Driver *****//
  //  It uses .setDrive( motorNum, direction, level ) to drive the motors.

  if(whistle) {                                         // Stop avoiding obstacles when the whistle blows
    while (whistle){
      myPDM.getData(pdmDataBuffer, pdmDataBufferSize); // keep checking PDM
      whistle = loudestFreq();                         // do we hear a whistle?
      myMotorDriver.setDrive( right, 1, 0);             // stop
      myMotorDriver.setDrive( left, 1, 0);
    } // OK, the user has stopped whistling.
    // Don't exit this loop until the user whistles again
    while (!whistle){
      myPDM.getData(pdmDataBuffer, pdmDataBufferSize); // keep checking PDM
      whistle = loudestFreq();                         // do we hear a whistle?
      myMotorDriver.setDrive( right, 1, 0);             // stop
      myMotorDriver.setDrive( left, 1, 0);
    }
  }
  if(distance < 200){
    t = millis();
    // Hard stop
    myMotorDriver.setDrive( right, 0, brake);
    myMotorDriver.setDrive( left, 1, brake);
    t = millis();
    while (millis()-t < 150){ // Wait for a specified time, but keep updating the PDM data
      myPDM.getData(pdmDataBuffer, pdmDataBufferSize);
    }
    // Spin in place
    myMotorDriver.setDrive( left, 1, powerTurn);             // 1 for reverse
    //myMotorDriver.setDrive( right, 1, powerTurn); // 1 for forward
    // Actually, drive in a backward arc. This is better for not running the wheels into things.
    myMotorDriver.setDrive( right, 1, 0);                     
    
    while (millis()-t < 200){ // Wait for a specified time, but keep updating the PDM data
      myPDM.getData(pdmDataBuffer, pdmDataBufferSize);
    }
  }
  else{
    // Drive straight forward
    myMotorDriver.setDrive( left, 0, power);             // 1 for forward
    myMotorDriver.setDrive( right, 1, calib*power+offset); // left side is reversed
  }
}

//// Functions

bool loudestFreq()    // Function returns true if the loudest freq. is within whistling range
// Written mostly by Sparkfun in void printLoudest(void) in the microphone test code.
{
  float fMaxValue;
  uint32_t ui32MaxIndex;
  int16_t *pi16PDMData = (int16_t *)pdmDataBuffer;
  uint32_t ui32LoudestFrequency;

  //
  // Convert the PDM samples to floats, and arrange them in the format
  // required by the FFT function.
  //
  for (uint32_t i = 0; i < pdmDataBufferSize; i++)
  {
    g_fPDMTimeDomain[2 * i] = pi16PDMData[i] / 1.0;
    g_fPDMTimeDomain[2 * i + 1] = 0.0;
  }

  //
  // Perform the FFT.
  //
  arm_cfft_radix4_instance_f32 S;
  arm_cfft_radix4_init_f32(&S, pdmDataBufferSize, 0, 1);
  arm_cfft_radix4_f32(&S, g_fPDMTimeDomain);
  arm_cmplx_mag_f32(g_fPDMTimeDomain, g_fPDMMagnitudes, pdmDataBufferSize);

  //
  // Find the frequency bin with the largest magnitude.
  //
  arm_max_f32(g_fPDMMagnitudes, pdmDataBufferSize / 2, &fMaxValue, &ui32MaxIndex);

  ui32LoudestFrequency = (sampleFreq * ui32MaxIndex) / pdmDataBufferSize;
  if(ui32LoudestFrequency>800 && ui32LoudestFrequency<1200){
    //blink();
    return true;
  }
  else
    return false;
}

//*****************************************************************************
//
// Print PDM configuration data.
//
//*****************************************************************************
void printPDMConfig(void)
{
  uint32_t PDMClk;
  uint32_t MClkDiv;
  float frequencyUnits;

  //
  // Read the config structure to figure out what our internal clock is set
  // to.
  //
  switch (myPDM.getClockDivider())
  {
  case AM_HAL_PDM_MCLKDIV_4:
    MClkDiv = 4;
    break;
  case AM_HAL_PDM_MCLKDIV_3:
    MClkDiv = 3;
    break;
  case AM_HAL_PDM_MCLKDIV_2:
    MClkDiv = 2;
    break;
  case AM_HAL_PDM_MCLKDIV_1:
    MClkDiv = 1;
    break;

  default:
    MClkDiv = 0;
  }

  switch (myPDM.getClockSpeed())
  {
  case AM_HAL_PDM_CLK_12MHZ:
    PDMClk = 12000000;
    break;
  case AM_HAL_PDM_CLK_6MHZ:
    PDMClk = 6000000;
    break;
  case AM_HAL_PDM_CLK_3MHZ:
    PDMClk = 3000000;
    break;
  case AM_HAL_PDM_CLK_1_5MHZ:
    PDMClk = 1500000;
    break;
  case AM_HAL_PDM_CLK_750KHZ:
    PDMClk = 750000;
    break;
  case AM_HAL_PDM_CLK_375KHZ:
    PDMClk = 375000;
    break;
  case AM_HAL_PDM_CLK_187KHZ:
    PDMClk = 187000;
    break;

  default:
    PDMClk = 0;
  }

  //
  // Record the effective sample frequency. We'll need it later to print the
  // loudest frequency from the sample.
  //
  sampleFreq = (PDMClk / (MClkDiv * 2 * myPDM.getDecimationRate()));

  frequencyUnits = (float)sampleFreq / (float)pdmDataBufferSize;

//  Serial.printf("Settings:\n");
//  Serial.printf("PDM Clock (Hz):         %12d\n", PDMClk);
//  Serial.printf("Decimation Rate:        %12d\n", myPDM.getDecimationRate());
//  Serial.printf("Effective Sample Freq.: %12d\n", sampleFreq);
//  Serial.printf("FFT Length:             %12d\n\n", pdmDataBufferSize);
//  Serial.printf("FFT Resolution: %15.3f Hz\n", frequencyUnits);
}
