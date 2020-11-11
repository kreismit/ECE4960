/* This code reads roll, pitch, and yaw angles and transmits them continuously over Bluetooth
   in CSV format to a dedicated Python watcher.
   Based on examples from SparkFun and on the ECE_4960_robot sketch by Alex Coy Tailor. */

/* Includes and Definitions                                                      */
// Bluetooth and ECE4960_robot
#include "BLE_example.h"
#include "commands.h"
#include "related_funcs.h"
// IMU and related calculations
#include "ICM_20948.h"  // Latest version: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <math.h>       // for trig functions
// Serially Controlled Motor Driver
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
// IR ToF sensor
#include <Wire.h>
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>

SCMD scmd; // This creates the main object of one motor driver and connected slaves.

ICM_20948_I2C IMU;  // Create an ICM_20948_I2C object. Used to get data from the IMU.

SFEVL53L1X TOF;  // Create a VL53L1X object. Used to get data from the ToF.

#define AD0_VAL   1     // The value of the last bit of the I2C address. 
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                        // the ADR jumper is closed the value becomes 0

// maximum length of reply / data message
#define MAXREPLY 100
//#define SERIAL_DEBUG    // Uncomment this to enable general serial debugging.
//#define SERIAL_TOF      // Uncomment this to print ToF values to serial.
//#define SERIAL_PID      // Uncomment this to print PID values to serial.
#define SERIAL_PORT Serial  // formerly for SparkFun library purposes
#define FLIPPED 0         // 0 if curved side up; 1 if flat side up


/* Global variables                                               */
// For calculating angles...
float roll, pitch, yaw;     // Angles of tilt about X, Y, and Z axes, respectively (deg)
float rollRad, pitchRad, yawRad; // Angles of tilt about X, Y, and Z axes, respectively (rad)
float rollXL, pitchXL;      // Angles of tilt about X and Y axes, measured by accelerometer (deg)
float compFactorRoll = 1;   // Compensation factors for the slight errors in the XL angles
float compFactorPitch = 1;
float rad2deg = 180/M_PI;   // Convert degrees to radians using this multiplier
float deg2rad = M_PI/180;   // Convert radians to degrees using this multiplier
float omX, omY, omZ;        // Rotation rates about X, Y, and Z, as measured by gyro (deg/s)
float thX, thY, thZ;        // Angles about X, Y, and Z, as calculated by gyro (deg)
float omXCal = 0, omYCal = 0, omZCal = 0; //Calibration offsets for omX, omY, and omZ
float aX, aY, aZ, a;        // Acceleration components in X,Y,Z; and then total accel.
float aCX, aCY, aCZ;           // Accelerometer readings (compensated) along X, Y, Z
float aXCal = 0, aYCal = 0, aZCal = 0; // Calibration offsets for aX, aY, aZ
float v = 0, X = 0, Y = 0;  // velocity and positions for odometry
float mTotal;               // total in-plane magnetometer reading
int mCount = 0;             // count the number of magnetometer readings for averaging
float alpha = 0.3;          // Complementary filter parameter
uint32_t tNow, tLast;       // current and previous times (microseconds)
float dt;                   // change in time (sec)
int tWait = 10;             // Milliseconds to wait between each data update
float calTime = 5;          // how many seconds to calibrate

// For communicating via Bluetooth...

// buffer to reply to client
uint8_t val[MAXREPLY];
uint8_t *val_data = &val[2]; // start of optional data area
uint8_t *val_len = &val[1];  // store length of optional data

String s_Rev = "Rev 1.0";
String s_Rcvd = "100"; //KHE declare extern in BLE_example_funcs.cpp to accept messages,
                      // if 100 don't bother checking case statements
uint16_t l_Rcvd = 0;
uint8_t *m_Rcvd = NULL;
String s_AdvName = "MyRobot"; //KHE 2 0 TOTAL CHARACHTERS ONLY!!  any more will be dropped

cmd_t empty_cmd = {NOT_A_COMMAND, 1, {0}};
cmd_t *cmd = &empty_cmd;
cmd_t *res_cmd = &empty_cmd;
bt_debug_msg_t *bt_debug_head = NULL;
bt_debug_msg_t *bt_debug_tail = NULL;

present_t presentSensors = {
  .motorDriver = 1,
  .ToF_sensor = 1,
  .prox_sensor = 0,
  .IMU = 1
};
int bytestream_active = 0;    // Device starts streaming when this is nonzero. can be 4 or 8 (# bytes)
unsigned long start;          // stores the initial time, in microseconds, when the bytestream starts
unsigned long finish = 0;     // stores the time, in microseconds, when the bytestream ends
uint32_t bytestreamCount = 0; // counts the number of bytestream messages sent
uint64_t bytestreamCount64;   // and the 64-bit version
float data32[4];              // array of 32-bit floating-point numbers to send
uint64_t data64[4];           // array of unsigned long's / double's to send
cmd_type_e typeOld;                  // previous command type

// For motor control
bool address, direction;    // address bit and forward/backward bit for each motor
uint8_t power[2];           // power levels to send to each motor
byte right = 1;             // Addresses for R and L drive motors.
byte left = 0;              // One byte is plenty of room for these.
byte rFwd = 1;              // These are for sanity and error avoidance:
byte rRev = 0;              // so I don't have to remember which way is forward.
byte lFwd = 0;
byte lRev = 1;
int offset = 4;             // left side gets (offset) more power than right side
float calib = 1.08;         // left side gets (calib) times more power than right
// For PID controller: subscripts linear = 0, angular = 1
bool pid = false;           // are we using the PID controller to run the motors?
float r[2];                 // setpoint for angular velocity controller
float kp[2], ki[2], kd[2];  // P, I, and D gains for controller; set by master via Bluetooth
float e[2];                 // current error values
float eLast[2];             // previous error value
float inte[2];              // integral of error
float de;                   // change in error (differential)
float alphaE[2];            // lag filter parameter
float output[2];            // output of PID to send to motors

bool TOFStarted = 0;

void setup()
{
  // Initialize PID constants
  e[0] = 0; e[1] = 0;
  inte[0] = 0; inte[1] = 0;
  alphaE[0] = 0.1; alphaE[1] = 0.5; // lag filter parameters - tweak me!
  Serial.begin(115200);
  
  #ifdef AM_DEBUG_PRINTF
    //
    // Enable printing to the console.
    //
    enable_print_interface();
  #endif
  
  #ifdef SERIAL_DEBUG
    //Serial.printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    Serial.print("Revision = ");
    Serial.print(s_Rev);
    Serial.printf("  ECE 4960 Robot Compiled: %s   %s\n", __DATE__, __TIME__);
    //Serial.printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  #endif
  
  analogWriteResolution(16); //Set AnalogWrite resolution to 16 bit = 0 - 65535
  // but make max 64k or trouble
  
  Wire.begin();
  Wire.setClock(400000);

  /* Set up ToF sensor */
  if (TOF.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("ToF failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  TOF.setDistanceModeShort();
  TOF.setTimingBudgetInMs(60);       // ideal for SR
  //TOF.setDistanceModeLong();           // default
  //TOF.setTimingBudgetInMs(180);        // ideal for LR
  TOF.setIntermeasurementPeriod(15);  // default is 100
  TOF.setOffset(37);
  
  bool initialized = false;
  while( !initialized ){
    // Wait for the serial port to open (if serial output is turned on)
    #ifdef SERIAL_DEBUG
        Serial.begin(115200);
        while(!Serial){};
    #endif
    
    IMU.begin( Wire, AD0_VAL );  // use I2C
    #ifdef SERIAL_DEBUG
        Serial.printf("Initialization of the IMU returned: ");
        Serial.println( IMU.statusString() );
    #endif
    
    if( IMU.status != ICM_20948_Stat_Ok ){
      #ifdef SERIAL_DEBUG
        Serial.println( "Trying again..." );
      #endif
      delay(500);
    }else{
      initialized = true;
    }
  }

  // Set full scale ranges for both acc and gyr (thanks SparkFun for the example code!)
  ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain
  // values for all configurable sensors
  
  myFSS.a = gpm2;         // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                          // gpm2
                          // gpm4
                          // gpm8
                          // gpm16
                          
  myFSS.g = dps1000;      // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                          // dps250
                          // dps500
                          // dps1000
                          // dps2000
  // Chose 1000 dps (about 4 rot/s = 240 rpm) based on testing / trial & error.
                          
  IMU.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS );
  
  //// Calibrate gyroscope and magnetometer readings
  // For magnetometer calibration to work, it must begin pointing north.
//  #ifdef SERIAL_DEBUG
//    Serial.println("Point me north!");
//    delay(2000);
//    Serial.println("Calibrating...");
//  #endif
  
  int tCal = calTime*1000000;       // 1 sec = 1000000 microseconds
  float thXCal = 0, thYCal = 0, thZCal = 0; // angles to measure during calibration
  tNow = micros();
  int tStartCal = tNow;             // starting calibration now!
  while(tNow-tStartCal < tCal){     // run for tCal microseconds (plus one loop)
    if( IMU.dataReady() ){
      tLast = tNow;                   // what was new has grown old
      delay(tWait);
      IMU.getAGMT();                // The values are only updated when you call 'getAGMT'
      tNow = micros();                // update current time
      dt = (float) (tNow-tLast)*0.000001; // calculate change in time in seconds
      omX = IMU.gyrX(); omY = IMU.gyrY(); omZ = IMU.gyrZ(); // angular velocities about X, Y, & Z
      aX = IMU.accX(); aY = IMU.accY(); aZ = IMU.accZ();    // accelerations in X, Y, & Z
      thXCal = thXCal + omX*dt;                 // update rotation angles about X (roll)
      thYCal = thYCal + omY*dt;                 // Y (pitch)
      thZCal = thZCal + omZ*dt;                 // and Z (yaw)
      aXCal = aXCal + aX*dt;
      aYCal = aYCal + aY*dt;
      aZCal = aZCal + aZ*dt;
    }
  }
  // and we want the average rotation rate in deg/s after that loop
  omXCal = thXCal / calTime;
  omYCal = thYCal / calTime;
  omZCal = thZCal / calTime;
  aXCal = aXCal / calTime;  // same with acceleration
  aYCal = aYCal / calTime;
  aZCal = aZCal / calTime;
  
  #ifdef SERIAL_DEBUG
    Serial.printf("Calibration offsets: ");
    Serial.printf("X %6.4f, Y %6.4f, Z %6.4f degrees per second\n",omXCal,omYCal,omZCal);
  #endif
  
  //// Initialize motor driver so we can drive!
  
  //***** Configure the Motor Driver's Settings *****//
  //  .commInter face can be I2C_MODE or SPI_MODE
  scmd.settings.commInterface = I2C_MODE;
  //scmd.settings.commInterface = SPI_MODE;

  //  set address if I2C configuration selected with the config jumpers
  scmd.settings.I2CAddress = 0x5D; //config pattern "0101" on board for address 0x5A

  //  set chip select if SPI selected with the config jumpers
  scmd.settings.chipSelectPin = 10;

  //*****initialize the driver get wait for idle*****//
  while ( scmd.begin() != 0xA9 ) //Wait until a valid ID word is returned
  {
    #ifdef SERIAL_DEBUG
      Serial.println( "ID mismatch, trying again" );
    #endif
    delay(500);
  }
  //Serial.println( "ID matches 0xA9" );

  //  Check to make sure the driver is done looking for slaves before beginning
  //Serial.print("Waiting for enumeration...");
  while ( scmd.ready() == false );

  //*****Set application settings and enable driver*****//

  while ( scmd.busy() );
  scmd.enable();


  /************************************************************************************************
                  Set Advertising name:  uses global string s_AdvName set above.
   ************************************************************************************************/
  set_Adv_Name(); //BLE_example_funcs.cpp near end of file, fn declared extern in BLE_example.h

  /************************************************************************************************
                   Boot the radio
                    SDK/third_party/exactle/sw/hci/apollo3/hci_drv_apollo3.c
                    = huge program to handle the ble radio stuff in this file
   ***********************************************************************************************/
  HciDrvRadioBoot(0);

  /************************************************************************************************
        Initialize the main ExactLE stack: BLE_example_funcs.cpp
        - One time timer
        - timer for handler
        - dynamic memory buffer
        - security
        - HCI host conroller interface
        - DM device manager software
        - L2CAP data transfer management
        - ATT - Low Energy handlers
        - SMP - Low Energy security
        - APP - application handlers..global settings..etc
        - NUS - nordic location services

   ************************************************************************************************/
  exactle_stack_init();

  /*************************************************************************************************
      Set the power level to it's maximum of 15 decimal...defined in hci_drv_apollo3.h as 0xF
      needs to come after the HCI stack is initialized in previous line
        - poss. levels = 0x03=-20,0x04=-10,0x05=-5,0x08=0,0x0F=4 but have to use definitions,
        not these ints.
          extremes make a difference of about 10 at 1 foot.
   **********************************************************************************************/
  HciVsA3_SetRfPowerLevelEx(TX_POWER_LEVEL_PLUS_3P0_dBm);
  //= 15 decimal = max power WORKS..default = 0

  /***********************************************************************************************
      Start the "Amdtp" (AmbiqMicro Data Transfer Protocol) profile. Function in amdtp_main.c

       Register for stack callbacks
       - Register callback with DM for scan and advertising events with security
       - Register callback with Connection Manager with client id
       - Register callback with ATT low energy handlers
       - Register callback with ATT low enerty connection handlers
       - Register callback with ATT CCC = client charachteristic configuration array
       - Register for app framework discovery callbacks
       - Initialize attribute server database
       - Reset the device

   *********************************************************************************************/
  AmdtpStart();

  /**********************************************************************************************
     On first boot after upload and boot from battery, pwm on pin 14 not working
      need to reset nano board several times with battery power applied to get
      working.  Delay 5 seconds works..haven't tried lesser values.
   *********************************************************************************************/
  //delay(5000);

  /**********************************************************************************************
      Arduino device GPIO control setup.
        Place after board BLE setup stuff happens.  ie.:
          could not get A14 to PWM untill I moved the set_stop() call from the
          beginning of setup to this location...then works great.
   *********************************************************************************************/

  pinMode(LED_BUILTIN, OUTPUT);

  //Set a starting point...for motors, servo, and LED_BUILTIN
  //delay(1000);

  // Bluetooth will start after blinking
  for (int i = 0; i < 20; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }

  //setupwdt();

  pinMode(blinkPin, OUTPUT);
  digitalWrite(blinkPin, LOW);

  // Configure the watchdog.
  //setupTimerA(myTimer, 31); // timerNum, period
  //moved to BLE_example_funcs.cpp scheduler_timer_init
  setupWdt();
  am_hal_wdt_init(&g_sWatchdogConfig);
  //NVIC_EnableIRQ(CTIMER_IRQn); // Enable CTIMER interrupt in nested vector interrupt controller.
  NVIC_EnableIRQ(WDT_IRQn); // Enable WDT interrupt in nested vector interrupt controller.

  uint8_t a = 0;
  m_Rcvd = &a;
  //Serial.printf("Size of command: %d", sizeof(cmd_t));
  am_hal_interrupt_master_enable();
  //interrupts(); // Enable interrupt operation. Equivalent to am_hal_rtc_int_enable().
  //am_hal_wdt_start();
  //am_hal_wdt_int_enable(); - freezes boot

} /*** END setup FCN ***/

void loop()
{
  //Serial.println("Loop...."); //KHE Loops constantly....no delays

  if(!TOFStarted){
    TOF.startRanging(); // initiate measurement the first time
    TOFStarted = true;
  }
  if (l_Rcvd > 1) //Check if we have a new message from amdtps_main.c through BLE_example_funcs.cpp
  {

    cmd = (cmd_t *)m_Rcvd;
    
    #ifdef SERIAL_DEBUG
      Serial.print("Message Buffer: ");
      for (int i = 0; i < l_Rcvd; i++)
        Serial.printf("%d ", m_Rcvd[i]);
      Serial.println();
      Serial.printf("Got command: 0x%x Length: 0x%x Data: ", cmd->command_type, cmd->length);

      for (int i = 0; i < cmd->length; i++)
      {
        Serial.printf("0x%x ", cmd->data[i]);
      }
      Serial.println();
    #endif  /* SERIAL_DEBUG */

    switch (cmd->command_type)
    {
      case SET_MOTORS:
      {
        // Receiving a byte array in this format:
        // [commandValue, length, leftPower, rightPower]
        //      0           1       2           3
        // The last two are "data", so data = [leftPower, rightPower].
        for(byte i=0; i<2; i++){ // run this twice
          address = i;                  // left is 0; right is 1
          power[i] = cmd->data[i];
          scmd.writeRegister(SCMD_MA_DRIVE+i,power[i]);  // the driver accepts a value 0-255
        }
        //break;  // flow right into GET_MOTORS so the computer's records update automatically
      }
      case GET_MOTORS:
      {
        typeOld = res_cmd->command_type;  // save old value, so it can be reset after this finishes
        res_cmd->command_type = GET_MOTORS;
        res_cmd->length = 4;  // 2 uint8_t's for type and length, and two more for data
        for(byte i=0; i<2; i++){
          address = i;
          power[i] = scmd.readRegister(SCMD_MA_DRIVE+i);
          //((uint8_t*)res_cmd->data)[i] = power[i]; // already stored in memory
        }
        amdtpsSendData((uint8_t *)res_cmd, res_cmd->length);
        res_cmd->command_type = typeOld;
        break;
      }
      case SET_VEL:
      {
        for(byte i=0; i<2; i++){
          r[i] = ((float*)cmd->data)[i];
          Serial.printf("r[%d] = %3.2f\n",i,r[i]);
        }
        if (r[0] == 0 && r[1] == 0){ // Special case: stopping robot completely
          pid = false;
          scmd.setDrive(left,0,0);
          scmd.setDrive(right,0,0);
          Serial.println("Stopping robot.");
        }
        else{
          pid = true;
          Serial.println("Starting PID-controlled movement.");
        }
        break;
      }
      case GET_VEL:
      {
        Serial.println("Going to send current linear and angular rates");
        res_cmd->command_type = GET_VEL;
        bytestream_active = 4;              // the angles are 32-bit floats
        ((uint32_t *)res_cmd->data)[0] = 0;
        break;
      }
      case SET_GAINS:
      {
        // Sets PID gains for linear (0) and angular (1) velocity control
        for(byte i=0; i<2; i++){
          kp[i] = ((float*)cmd->data)[3*i];
          ki[i] = ((float*)cmd->data)[3*i+1];
          kd[i] = ((float*)cmd->data)[3*i+2];
        }
        Serial.printf("Linear: Kp = %3.2f, Ki = %3.2f, Kd = %3.2f\n",kp[0],ki[0],kd[0]);
        Serial.printf("Angular: Kp = %3.2f, Ki = %3.2f, Kd = %3.2f\n",kp[1],ki[1],kd[1]);
        break;
      }
      case SER_RX:
      {
        Serial.println("Got a serial message");
        pushMessage((char *)&cmd->data, cmd->length);
        break;
      }
      case GET_ODOM:
      {
        #ifdef SERIAL_DEBUG
          Serial.println("Going to send current R, P, and Y angles");
        #endif
        res_cmd->command_type = GET_ODOM;
        //res_cmd->length = 6;
        //((float *) res_cmd->data)[0] = yaw; // send current yaw (a float)
        //amdtpsSendData((uint8_t *)res_cmd, res_cmd->length);
        bytestream_active = 4;              // the angles are 32-bit floats
        ((uint32_t *)res_cmd->data)[0] = 0;
        break;
      }
      case PING:
      {
        cmd->command_type = PONG;
        amdtpsSendData(m_Rcvd, l_Rcvd);
        #ifdef SERIAL_DEBUG
          Serial.printf("Returning ping after %3.1f ms\n", (micros()-finish)*0.001);
          finish=micros();
        #endif
        break;
      }
      case START_BYTESTREAM_TX:
      {
        bytestream_active = (int)cmd->data[0];
        #ifdef SERIAL_DEBUG
          Serial.printf("Start bytestream with active %d \n", bytestream_active);
        #endif
        ((uint32_t *)res_cmd->data)[0] = 0;
        res_cmd->command_type = BYTESTREAM_TX;
        break;
      }
      case STOP_BYTESTREAM_TX:
      {
        bytestream_active = 0;
        break;
      }
      default:
      {
        Serial.printf("Unsupported Command 0x%x \n", cmd->command_type);
        break;
      }
    }

    l_Rcvd = 0;
    am_hal_wdt_restart();
    free(m_Rcvd);
  } //End if s_Rcvd != 100
  else if ((s_Rcvd[0] == '6' && s_Rcvd[1] == '7'))
  {
    s_Rcvd[0] = 0;
    digitalWrite(LED_BUILTIN, HIGH);
    //Serial.printf("Connected, length was %d", l_Rcvd);
  }
  else if ((s_Rcvd[0] == '6' && s_Rcvd[1] == '8'))
  {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("disconnected");
    //Decimal value of D for Disconnect
    //Serial.println("got disconnect from case in ino file - set_Stop");
    digitalWrite(LED_BUILTIN, LOW);
    //amdtps_conn_close();
    DmDevReset();
  }

  if (availableMessage())
  {
    Serial.println("Bluetooth Message:");
    Serial.println(pullMessage());
    printOverBluetooth("Message Received.");
  }

  if (bytestream_active)
  {
    //res_cmd->command_type = BYTESTREAM_TX; // let the command switch take care of this now
    // Numbers to be packed will be in an array data32 or data64 (depending on size)
    uint32_t *p32;            // pointers to the appropriate data types
    uint64_t *p64;
    switch(bytestream_active){
      case 4:                 //asked for a 4-byte number!
      {
        int size0 = sizeof(data32[0]);
        int len = sizeof(data32)/size0;          // how many numbers will fit in this stream
        bytestreamCount++;        // we are sending one bytestream now
        // Two bytes for command_type and for length;
        res_cmd->length=2+bytestream_active*len; // then 4 bytes per uint32_t or 8 per uint64_t
        p32=(uint32_t*) res_cmd->data;
        for(int i=0; i<len; i++){
          memcpy(p32, &data32[i], size0);
          p32++;                      // move 4 bytes down the array
        }
        //bytestreamCount = (float) bytestreamCount;
        //memcpy(p32, &bytestreamCount, sizeof(bytestreamCount));
        //p32++;
        //memcpy(p32, &start, sizeof(start));
        #ifdef SERIAL_DEBUG
          //Serial.printf("Stream %d", bytestream_active);
          //Serial.printf(" after %3.1f ms latency\n", (micros() - finish)*0.001);
          //start=micros();
          Serial.printf("Sending 32-bit numbers [ ");
          for(int i=0; i<len; i++){
            Serial.printf("%4.2f ",data32[i]);
          }
          Serial.printf("]\n");
        #endif
        break;
      }
      case 8:               //asked for an 8-byte number!
      //default:              // the default is to send a 64-bit (8-byte) array
      {
        int size0 = sizeof(data64[0]);
        int len = sizeof(data64)/size0;          // how many numbers will fit in this stream
        //bytestreamCount++;        // we are sending one bytestream now
        // Two bytes for command_type and for length;
        res_cmd->length=2+bytestream_active*len; // then 4 bytes per uint32_t, 8 per uint64_t
        p64=(uint64_t*) res_cmd->data;
        for(int i=0; i<len; i++){
          memcpy(p64, &data64[i], size0);
          p64++;                      // move 8 bytes down the array
        }
        //bytestreamCount64 = (uint64_t) bytestreamCount;
        //memcpy(p64, &bytestreamCount64, sizeof(bytestreamCount));
        //p64++;
        //memcpy(p64, &start, sizeof(start));
        #ifdef SERIAL_DEBUG
          Serial.printf("Sending 64-bit numbers [ ");
          for(int i=0; i<len; i++){
            Serial.printf("%4.2f ",data64[i]);
          }
          Serial.printf("]\n");
        #endif
        break;
      }
    }
    amdtpsSendData((uint8_t *)res_cmd, res_cmd->length);
//    #ifdef SERIAL_DEBUG
//      finish=micros();
//      Serial.printf("Finished sending bytestream after %u microseconds\n",finish-start);
//    #endif
  }

  trigger_timers();

  // Disable interrupts.

  /*
    //Uncomment this if you want the board to go to sleep and be woken up
    //by Timer2. Not good for instruction streaming!

    am_hal_interrupt_master_disable();

    //
    // Check to see if the WSF routines are ready to go to sleep.
    //
    if (wsfOsReadyToSleep())
    {
      am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
    // Loop stops here on sleep and wakes on Timer2 interrupt,
    // runs about 30 loops, then sleeps again.
    // An interrupt woke us up so now enable them and take it.
    am_hal_interrupt_master_enable();
  */

  // Update current range reading
  // Don't wait for data to be ready; instead, keep running the main loop and read data
  // when it's ready. The ToF sensor updates fairly slowly.
  if (TOF.checkForDataReady()) // Ready!
  {
    data32[3] = TOF.getDistance();  //Get the result of the measurement from the sensor
    // and put it in the data array at the end
    #ifdef SERIAL_TOF
      Serial.print("ToF reading: ");
      Serial.printf("%4.1f mm\n", data32[3]);
    #endif
    TOF.clearInterrupt();
    //TOF.stopRanging();
    TOF.startRanging(); //Write configuration bytes to initiate measurement
  }
  
  if( IMU.dataReady() ){
    IMU.getAGMT();                // The values are only updated when you call 'getAGMT'
    tLast = tNow;                   // the new has grown old
    tNow = micros();
    dt = (float) (tNow-tLast)*0.000001; // calculate change in time in seconds
    
    aX = IMU.accX(); aY = IMU.accY(); aZ = IMU.accZ();
    omX = IMU.gyrX()-omXCal;          // omega's (angular velocities) about all 3 axes
    omY = IMU.gyrY()-omYCal;
    if (FLIPPED)
      omZ = omZCal-IMU.gyrZ();          // invert the gyro reading if flipped
    else
      omZ = IMU.gyrZ()-omZCal;          // not flipped; use positive reading
    thX = thX + omX*dt;                 // update rotation angles about X (roll)
    thY = thY + omY*dt;                 // Y (pitch)
    thZ = thZ + omZ*dt;                 // and Z (yaw)
    a = sqrt(pow(aX,2)+pow(aY,2)+pow(aZ,2));  // calculate total acceleration
    rollXL = asin(aY/a)*compFactorRoll*rad2deg;
    pitchXL = asin(aX/a)*compFactorPitch*rad2deg;
    roll = alpha*rollXL + (1-alpha)*thX;  // complementary filter
    pitch = alpha*pitchXL + (1-alpha)*thY;
    thX = roll; thY = pitch;              // overwrite any gyro drift
    yaw = thZ;                            // not doing complementary filter yet
    rollRad = roll*deg2rad; pitchRad = pitch*deg2rad;
    // Odometry with accelerometer
    //aCX = 10*(cos(pitchRad)*(aX-aXCal)+sin(pitchRad)*(aZ-aZCal));  // acceleration, calibrated and rotated
    aCX = 10*(aX-aXCal);                               // acceleration, calibrated but ignoring rotation
    v = v + aCX*dt; // integrate to get velocity...
    X = X + v*cos(yaw*deg2rad)*dt;    // and integrate again (in polar coordinates) to get position.
    Y = Y + v*sin(yaw*deg2rad)*dt;
    //Serial.printf("%4.2f,%4.2f,%4.2f\n",roll,pitch,yaw); // and print the angles to serial
    if (res_cmd->command_type==GET_ODOM){
      data32[0] = X;                   //update Bluetooth data
      data32[1] = Y;
      data32[2] = yaw;
    }
    else if (res_cmd->command_type==GET_VEL){
      // Give raw values (but with calibration offset)
      data32[0] = v;  // Raw X velocity
      data32[1] = 0;  // Y velocity (sideways) assumed zero
      data32[2] = omZ;
    }
  }else{
    Serial.println("Waiting for data from IMU");
    delay(500);
  }
  // Handle PID command
  if(pid){ // As long as we're running the controller,
    for(int i=0; i<2; i++){ // run two separate controllers for linear (0) and angular (1)
      eLast[i] = e[i];  // Update error (but don't update time - this is already done in the gyro section)
      //tLast = t;
      //t = micros();
      //dt = (t - tLast)*0.000001;
      if(i){ // if this is the angular controller
        e[i] = alphaE[i]*(r[i]-omZ)+(1-alphaE[i])*eLast[i]; // lag filter on error
      }else{ // if this is the linear controller
        e[i] = alphaE[i]*(r[i]-v)+(1-alphaE[i])*eLast[i];  // lag filter on error
      }
      inte[i] = inte[i] + e[i]*dt;                          // integral term
      de = e[i] - eLast[i];                                 // numerator of deriv. term
      if (inte[i] > 127)                                    // anti-windup control for integral
        inte[i] = 127;
      else if (inte[i] < -127)
        inte[i] = -127;
      output[i] = kp[i]*e[i]+ki[i]*inte[i]+kd[i]*(de/dt);   // calculate output
    }
//    #ifdef SERIAL_PID
//      Serial.printf("P = %3.1f, I = %3.1f, D = %3.1f\n",kp[0]*e[0], ki[0]*inte[0], kd[0]*((e[0]-eLast[0])/dt));
//    #endif
    /* Remember: clockwise is direction 1. 
       Right goes full power forward for 255 and backward for 0.
       Left goes full power backward for 255 and forward for 0.
       When the robot is flipped, right becomes left and vice versa, so the opposite is true.
       However, for spinning, the robot actually spins in the same direction, from the floor's perspective, regardless of
       which way is up.                                                                             */
    if (r[0] !=0){ // If a nonzero linear velocity is requested,
      for(int i=0; i<2; i++){ // combine outputs from linear and angular PID controllers
        int sign = pow(-1, (float) i+FLIPPED); // -1^(i+1) so -1 for i=0; 1 for i=1
        // I couldn't make PID control with the accelerometer work, so I'm using open-loop.
        // Proportional gain is the multiplier (H) for the reference input.
        power[i] = sign*kp[0]*r[0] + output[1] + 127; // flip x output as appropriate
      }
    }
    else{ // spinning in place; ignore linear velocity
      for(int i=0; i<2; i++){
        power[i] = output[1] + 127; // forward on both sides, and the bot spins CCW
      }
      v = 0;   // and reset odometry velocity reading
    }
    for(int i=0; i<2; i++){    // update motor powers
      if (power[i] > 255)
        power[i] = 255;
      else if(power[i] < 0)
        power[i] = 0;
      else
        power[i] = power[i];
      scmd.writeRegister(SCMD_MA_DRIVE+i,power[i]);  // the driver accepts a value 0-255
    }
  }
  else{ // controller isn't running; motors are stopped
    // and reset odometry velocity reading
    v = 0;
  }
  delay(tWait); // regardless of what else runs, delay a bit before the next loop iteration
} //END LOOP
