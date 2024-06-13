#include <Arduino.h>
#include <SPI.h>
#include "io_config.h"
#include "MPU9250SPI.h"

uint8_t Gscale = GFS_250DPS, Ascale = AFS_2G, Mscale = MFS_16BITS, Mmode = M_100Hz, sampleRate = 0x04;         
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float motion = 0; // check on linear acceleration to determine motion
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
bool wakeup;

bool intFlag = false;
bool newMagData = false;

int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float   magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float   temperature;    // Stores the MPU9250 internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// These can be measured once and entered here or can be calculated each time the device is powered on
float   gyroBias[3] = {0.96, -0.21, 0.12}, accelBias[3] = {0.00299, -0.00916, 0.00952};
float   magBias[3] = {71.04, 122.43, -36.90}, magScale[3]  = {1.01, 1.03, 0.96}; // Bias corrections for gyro and accelerometer


uint32_t delt_t = 0;                      // used to control display output rate
uint32_t count = 0, sumCount = 0;         // used to control display output rate
float pitch, yaw, roll;                   // absolute orientation
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method


// MPU9250 Register Addresses

#define SerialDebug true   
#define WHO_AM_I 0x75
#define REG_WHO_AM_I 0x71 


#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
SPIClass spi = SPIClass(SPI1_MOSI_PIN, SPI1_MISO_PIN, SPI1_SCL_PIN);
// MOSI, SCK
MPU9250SPI MPU9250(SPI1_MOSI_PIN,SPI1_SCL_PIN, &spi);

void setup() {
  Serial.begin(8000);
  MPU9250.begin(SPI1_CS_PIN);
  Serial.println("MPU9250 9-axis motion sensor...");
  uint8_t c = MPU9250.getMPU9250ID();
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  delay(1000);
  
  if (c == 0x71 ) // WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255 
  {  
    Serial.println("MPU9250 is online...");
    
    MPU9250.resetMPU9250(); // start by resetting MPU9250
    
    MPU9250.SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
    delay(1000);

  // get sensor resolutions, only need to do this once
  aRes = MPU9250.getAres(Ascale);
  gRes = MPU9250.getGres(Gscale);
  mRes = MPU9250.getMres(Mscale);

 // Comment out if using pre-measured, pre-stored offset biases
  MPU9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
  Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
  Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
  delay(1000); 
  
  MPU9250.initMPU9250(Ascale, Gscale, sampleRate); 
  Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  
  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
  byte d = MPU9250.getAK8963CID();  // Read WHO_AM_I register for AK8963
  Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
  delay(1000); 
  
  // Get magnetometer calibration from AK8963 ROM
  MPU9250.initAK8963Slave(Mscale, Mmode, magCalibration); Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

 // Comment out if using pre-measured, pre-stored offset biases
//  MPU9250.magcalMPU9250(magBias, magScale);
  Serial.println("AK8963 mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]); 
  Serial.println("AK8963 mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]); 
  delay(2000); // add delay to see results before serial spew of data
  
  if(SerialDebug) {
  Serial.println("Calibration values: ");
  Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
  Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
  Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);

  attachInterrupt(SPI1_CS_PIN, myinthandler, RISING);  // define interrupt for intPin output of MPU9250
  }
  
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

  
}

void loop() {
  if(intFlag == true) {   // On interrupt, read data
      intFlag = false;     // reset newData flag
      
     MPU9250.readMPU9250Data(MPU9250Data); // INT cleared on any read
   
    // Now we'll calculate the accleration value into actual g's
     ax = (float)MPU9250Data[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
     ay = (float)MPU9250Data[1]*aRes - accelBias[1];   
     az = (float)MPU9250Data[2]*aRes - accelBias[2];  

    // Calculate the gyro value into actual degrees per second
     gx = (float)MPU9250Data[4]*gRes;  // get actual gyro value, this depends on scale being set
     gy = (float)MPU9250Data[5]*gRes;  
     gz = (float)MPU9250Data[6]*gRes; 
  
//    if( MPU9250.checkNewMagData() == true) { // wait for magnetometer data ready bit to be set
      MPU9250.readMagData(magCount);  // Read the x/y/z adc values
  
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
      mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
      my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
      mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];  
      mx *= magScale[0];
      my *= magScale[1];
      mz *= magScale[2]; 
//    }
   
  
    for(uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;

    /* end of MPU9250 interrupt handling */
    }
    }
}
//===================================================================================================================
//====== Set of useful functions
//===================================================================================================================

void myinthandler()
{
  intFlag = true;
}