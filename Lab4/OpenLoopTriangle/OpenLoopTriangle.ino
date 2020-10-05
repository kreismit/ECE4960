/******************************************************************************
  MotorTest.ino
  Serial Controlled Motor Driver
  Marshall Taylor @ SparkFun Electronics
  Sept 15, 2016
  https://github.com/sparkfun/Serial_Controlled_Motor_Driver
  https://github.com/sparkfun/SparkFun_Serial_Controlled_Motor_Driver_Arduino_Library

  Resources:
  Uses Wire.h for i2c operation
  Uses SPI.h for SPI operation

  Development environment specifics:
  Arduino IDE 1.6.7
  Teensy loader 1.27

  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
  Please review the LICENSE.md file included with this example. If you have any questions
  or concerns with licensing, please contact techsupport@sparkfun.com.
  Distributed as-is; no warranty is given.
******************************************************************************/
//This example steps through all motor positions moving them forward, then backwards.
//To use, connect a redboard to the user port, as many slaves as desired on the expansion
//port, and run the sketch.
//
// Notes:
//    While using SPI, the default LEDPIN will not toggle
//    This steps through all 34 motor positions, which takes a few seconds to loop.

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"

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

void setup()
{
  //Serial.begin(9600);
  //pinMode(LEDPIN, OUTPUT);
  //Serial.println("Starting sketch.");

  if (myPDM.begin() == false) // Turn on PDM with default settings, start interrupts
  {
    //Serial.println("PDM Init failed. Are you sure these pins are PDM capable?");
    while (1);  // halt the code here
  }
  else
    //Serial.println("PDM initialized successfully.");
  printPDMConfig();
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

  //Serial.println("Ready");
  //Serial.println();

}

void loop()
{
  bool whistle;             // Does the board hear a whistling frequency?
  int left=0; int right=1;  // addresses for L and R drive banks
  int power = 200;          // power to send to motors
  int brake = 100;
  int offset = 4;           // left side gets (offset) more power than right side
  float calib = 1.08;        // left side gets (calib) times more power than right
  if (myPDM.available())
  {
    myPDM.getData(pdmDataBuffer, pdmDataBufferSize);

    whistle = loudestFreq();
  }

  // Go to Deep Sleep until the PDM ISR or other ISR wakes us.
  //am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
  //***** Operate the Motor Driver *****//
  //  It uses .setDrive( motorNum, direction, level ) to drive the motors.
  // The right motor is 

  if(whistle) {
    //Serial.println("Heard whistle.");      
    // Drive straight forward
    for(int i=0; i<3; i++){
      myMotorDriver.setDrive( right, 1, power);             // 1 for forward
      myMotorDriver.setDrive( left, 0, calib*power+offset); // left side is reversed
      int t = millis();
      while (millis()-t < 350){ // Wait for a specified time, but keep updating the PDM data
        myPDM.getData(pdmDataBuffer, pdmDataBufferSize);
      }
      // Coast to a stop
      myMotorDriver.setDrive( left, 1, 0);                  // Set both sides to zero
      myMotorDriver.setDrive( right, 1, 0);
      t = millis();
      while (millis()-t < 500){ // Wait for a specified time, but keep updating the PDM data
        myPDM.getData(pdmDataBuffer, pdmDataBufferSize);
      }
      // Spin in place
      myMotorDriver.setDrive( right, 1, power);             // 1 for forward
      myMotorDriver.setDrive( left, 1, calib*power+offset); // left side is NOT reversed since we're spinning!
      t = millis();
      while (millis()-t < 290){ // Wait for a specified time, but keep updating the PDM data
        myPDM.getData(pdmDataBuffer, pdmDataBufferSize);
      }
      // Coast to a stop
      myMotorDriver.setDrive( left, 1, 0);                  // Set both sides to zero
      myMotorDriver.setDrive( right, 1, 0);
      t = millis();
      while (millis()-t < 500){ // Wait for a specified time, but keep updating the PDM data
        myPDM.getData(pdmDataBuffer, pdmDataBufferSize);
      }
    }
  }
  else{ // Otherwise, just stop the motors
      //Serial.println("Heard nothing");
      myMotorDriver.setDrive( left, 1, 0);
      myMotorDriver.setDrive( right, 1, 0);
  }
  // Wait some time (here, 1/4 sec) so the robot has time to stop before it hears its motors!
}

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
