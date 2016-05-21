#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

//  Function decarations
void initMPU9250(void);
void accGyroRead(void);
void magRead(void);

// Declaration of global variables
struct timespec ts;		//  delaytime for nanosleep function

int accGyroHandle;		//  i2c variables
int magHandle;
float aRes = 0;				//  resolution accelerometer
float gRes = 0;				//  resolution gyro
float mRes = 1.5;			//  resolution magnetometer 1,5 mG = 0,15 microT
double magFactoryCal[3] = {0, 0, 0};	//Factory calibration of magnetometer read from mpu
uint16_t axRead = 0;
uint16_t ayRead = 0;
uint16_t azRead = 0;
uint16_t gxRead = 0;
uint16_t gyRead = 0;
uint16_t gzRead = 0;
int16_t axRaw = 0;
int16_t ayRaw = 0;
int16_t azRaw = 0;
int16_t gxRaw = 0;
int16_t gyRaw = 0;
int16_t gzRaw = 0;
uint16_t mxRead = 0;
uint16_t myRead = 0;
uint16_t mzRead = 0;
int16_t mxRaw = 0;
int16_t myRaw = 0;
int16_t mzRaw = 0;
double ax = 0;
double ay = 0;
double az = 0;
double gx = 0;		//  All gyro measurements in deg per second
double gy = 0;
double gz = 0;
double mx = 0;		//  All gyro measurements in deg per second
double my = 0;
double mz = 0;

unsigned long start = 0;
unsigned long lastT = 0;
unsigned long tMag = 0;
unsigned long tAccGyro = 0;

//  Struct for return values
struct fusionResult {
	double yaw;
	double w;
	double wdot;
};

bool lastTime = false;
