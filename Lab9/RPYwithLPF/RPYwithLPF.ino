/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo 
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 * 
 * This code is beerware; if you see me (or any other SparkFun employee) at the
 * local, and you've found our code helpful, please buy us a round!
 * 
 * Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <math.h>       // for trig functions

//#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI    // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2        // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL   1     // The value of the last bit of the I2C address. 
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                        // the ADR jumper is closed the value becomes 0

#ifdef USE_SPI
  ICM_20948_SPI myICM;  // If using SPI create an ICM_20948_SPI object
#else
  ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
#endif

// Global variables
float roll, pitch, yaw;     // Angles of tilt about X, Y, and Z axes, respectively (deg)
float rollXL, pitchXL;      // Angles of tilt about X and Y axes, measured by accelerometer (deg)
float compFactorRoll = 1; // Compensation factors for the slight errors in the XL angles
float compFactorPitch = 1;
float rad2deg = 180/M_PI;   // Convert degrees to radians using this multiplier
float aX, aY, aZ, a;        // Acceleration components in X,Y,Z; and then total accel.
float aXLast, aYLast, aZLast;// Previous acceleration components in X,Y,Z
float omX, omY, omZ;        // Rotation rates about X, Y, and Z, as measured by gyro (deg/s)
float thX, thY, thZ;        // Angles about X, Y, and Z, as calculated by gyro (deg)
float omXCal, omYCal, omZCal = 0; //Calibration offsets for omX, omY, and omZ
float alpha = 0.5;          // Complementary filter parameter
float alpha2 = 0.1;         // Lag filter (LPF) parameter for accelerometer readings
int tNow, tLast;            // current and previous times (microseconds)
float dt;                   // change in time (sec)
int tWait = 10;             // Milliseconds to wait between each data update
float calTime = 5;          // how many seconds to calibrate

void setup() {

  SERIAL_PORT.begin(115200);
  while(!SERIAL_PORT){};

#ifdef USE_SPI
    SPI_PORT.begin();
#else
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
#endif
  
  bool initialized = false;
  while( !initialized ){

#ifdef USE_SPI
    myICM.begin( CS_PIN, SPI_PORT ); 
#else
    myICM.begin( WIRE_PORT, AD0_VAL );
#endif

    //SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
    //SERIAL_PORT.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok ){
      //SERIAL_PORT.println( "Trying again..." );
      delay(500);
    }else{
      initialized = true;
    }
  }
  
  //// Calibrate gyroscope angles
  //Serial.println("Calibrating gyro...");
  int tCal = calTime*1000000;       // 1 sec = 1000000 microseconds
  float thXCal = 0, thYCal = 0, thZCal = 0; // angles to measure during calibration while the gyro is stationary
  tNow = micros();
  aX = myICM.accX(); aY = myICM.accY(); aZ = myICM.accZ(); // initialize XL readings
  aXLast = aX; aYLast = aY; aZLast = aZ; // initialize "previous" numbers
  int tStartCal = tNow;             // starting calibration now!
  while(tNow-tStartCal < tCal){     // run for tCal microseconds (plus one loop)
    tLast = tNow;                   // what was new has grown old
    delay(tWait);
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
    tNow = micros();                // update current time
    dt = (float) (tNow-tLast)*0.000001; // calculate change in time in seconds
    omX = myICM.gyrX(); omY = myICM.gyrY(); omZ = myICM.gyrZ(); // omega's (angular velocities) about all 3 axes
    aX = myICM.accX(); aY = myICM.accY(); aZ = myICM.accZ();
    thXCal = thXCal + omX*dt;                 // update rotation angles about X (roll)
    thYCal = thYCal + omY*dt;                 // Y (pitch)
    thZCal = thZCal + omZ*dt;                 // and Z (yaw)
  }
  // and we want the average rotation rate in deg/s after that loop
  omXCal = thXCal / calTime;
  omYCal = thYCal / calTime;
  omZCal = thZCal / calTime;
  //Serial.printf("Calibration offsets: X %6.4f, Y %6.4f, Z %6.4f degrees per second\n",omXCal,omYCal,omZCal);
}

void loop() {

  if( myICM.dataReady() ){
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
    tLast = tNow;                   // the new has grown old
    aXLast = aX; aYLast = aY; aZLast = aZ;
    tNow = micros();
    dt = (float) (tNow-tLast)*0.000001; // calculate change in time in seconds
    // printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    // printScaledAGMT( myICM.agmt);   // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    aX = alpha2*myICM.accX()+ (1-alpha2)*aXLast;
    aY = alpha2*myICM.accY() + (1-alpha2)*aYLast;
    aZ = alpha2*myICM.accZ() + (1-alpha2)*aZLast;
    omX = myICM.gyrX()-omXCal;          // omega's (angular velocities) about all 3 axes
    omY = myICM.gyrY()-omYCal;
    omZ = myICM.gyrZ()-omZCal;
    thX = thX + omX*dt;                 // update rotation angles about X (roll)
    thY = thY + omY*dt;                 // Y (pitch)
    thZ = thZ + omZ*dt;                 // and Z (yaw)
    a = sqrt(pow(aX,2)+pow(aY,2)+pow(aZ,2));  // calculate total acceleration
    rollXL = asin(aY/a)*compFactorRoll*rad2deg;
    pitchXL = asin(aX/a)*compFactorPitch*rad2deg;
    roll = alpha*rollXL + (1-alpha)*thX;  // complementary filter
    pitch = alpha*pitchXL + (1-alpha)*thY;
    thX = roll; thY = pitch;              // overwrite any gyro drift
    yaw = thZ;                            // still waiting for the magnetometer
    Serial.printf("%4.2f,%4.2f,%4.2f\n",roll,pitch,yaw); // and print them
    delay(tWait);
  }else{
    //Serial.println("Waiting for data");
//    Serial.println("AccX,AccY,AccZ,GyX,GyY,GyZ,MagX,MagY,MagZ,Temp");
    delay(500);
  }
  
}


// Below here are some helper functions to print the data nicely!

void printPaddedInt16b( int16_t val ){
  if(val > 0){
    SERIAL_PORT.print(" ");
    if(val < 10000){ SERIAL_PORT.print("0"); }
    if(val < 1000 ){ SERIAL_PORT.print("0"); }
    if(val < 100  ){ SERIAL_PORT.print("0"); }
    if(val < 10   ){ SERIAL_PORT.print("0"); }
  }else{
    SERIAL_PORT.print("-");
    if(abs(val) < 10000){ SERIAL_PORT.print("0"); }
    if(abs(val) < 1000 ){ SERIAL_PORT.print("0"); }
    if(abs(val) < 100  ){ SERIAL_PORT.print("0"); }
    if(abs(val) < 10   ){ SERIAL_PORT.print("0"); }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT( ICM_20948_AGMT_t agmt){
  //SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b( agmt.acc.axes.x );
  SERIAL_PORT.print(",");
  printPaddedInt16b( agmt.acc.axes.y );
  SERIAL_PORT.print(",");
  printPaddedInt16b( agmt.acc.axes.z );
  //SERIAL_PORT.print(" ]");
  SERIAL_PORT.print(",");
  //SERIAL_PORT.print(" Gyr [ ");
  printPaddedInt16b( agmt.gyr.axes.x );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.gyr.axes.y );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.gyr.axes.z );
  //SERIAL_PORT.print(" ]");
  SERIAL_PORT.print(",");
  //SERIAL_PORT.print(" Mag [ ");
  printPaddedInt16b( agmt.mag.axes.x );
  SERIAL_PORT.print(",");
  printPaddedInt16b( agmt.mag.axes.y );
  SERIAL_PORT.print(",");
  printPaddedInt16b( agmt.mag.axes.z );
  //SERIAL_PORT.print(" ]");
  SERIAL_PORT.print(",");
  //SERIAL_PORT.print(" Tmp [ ");
  printPaddedInt16b( agmt.tmp.val );
  //SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}


void printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if(val < 0){
    SERIAL_PORT.print("-");
  }else{
    SERIAL_PORT.print(" ");
  }
  for( uint8_t indi = 0; indi < leading; indi++ ){
    uint32_t tenpow = 0;
    if( indi < (leading-1) ){
      tenpow = 1;
    }
    for(uint8_t c = 0; c < (leading-1-indi); c++){
      tenpow *= 10;
    }
    if( aval < tenpow){
      SERIAL_PORT.print("0");
    }else{
      break;
    }
  }
  if(val < 0){
    SERIAL_PORT.print(-val, decimals);
  }else{
    SERIAL_PORT.print(val, decimals);
  }
}

void printScaledAGMT( ICM_20948_AGMT_t agmt){
  //SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat( myICM.accX(), 5, 2 );
  SERIAL_PORT.print(",");
  printFormattedFloat( myICM.accY(), 5, 2 );
  SERIAL_PORT.print(",");
  printFormattedFloat( myICM.accZ(), 5, 2 );
  //SERIAL_PORT.print(" ]");
  SERIAL_PORT.print(",");
  //SERIAL_PORT.print(" Gyr (DPS) [ ");
  printFormattedFloat( myICM.gyrX(), 5, 2 );
  SERIAL_PORT.print(",");
  printFormattedFloat( myICM.gyrY(), 5, 2 );
  SERIAL_PORT.print(",");
  printFormattedFloat( myICM.gyrZ(), 5, 2 );
  //SERIAL_PORT.print(" ]");
  SERIAL_PORT.print(",");
  //SERIAL_PORT.print(" Mag (uT) [ ");
  printFormattedFloat( myICM.magX(), 5, 2 );
  SERIAL_PORT.print(",");
  printFormattedFloat( myICM.magY(), 5, 2 );
  SERIAL_PORT.print(",");
  printFormattedFloat( myICM.magZ(), 5, 2 );
  //SERIAL_PORT.print(" ]");
  SERIAL_PORT.print(",");
  //SERIAL_PORT.print(" Tmp (C) [ ");
  printFormattedFloat( myICM.temp(), 5, 2 );
  //SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}
