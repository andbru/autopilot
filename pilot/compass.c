
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <pthread.h>
#include <unistd.h>

//#include "global.h"
#include "madgwick.h"
#include "cavallo.h"
#include "conversion.h"

#include "compass.h"

extern int accGyroCount;
extern int magCount;
extern FILE *fp;	
extern pthread_mutex_t mutex1;
extern int counter;
extern struct fusionResult cavalloG;
extern struct fusionResult madgwickG;
extern double gpsCourseG;
extern double gpsSpeedG;

// Declaration of file-global variables
int accGyroHandle;			//  i2c variables
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
uint16_t mxRead = 0;
uint16_t myRead = 0;
uint16_t mzRead = 0;

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

void *compass() {

	struct fusionResult madgwick;
	madgwick.yaw = 0;
	madgwick.w = 0;
	madgwick.wdot = 0;
	struct fusionResult cavallo;
	cavallo.yaw = 0;
	cavallo.w = 0;
	cavallo.wdot = 0;
	//extern pthread_mutex_t mutex1;
	
	int16_t axRaw = 0;
	int16_t ayRaw = 0;
	int16_t azRaw = 0;
	int16_t gxRaw = 0;
	int16_t gyRaw = 0;
	int16_t gzRaw = 0;
	int16_t mxRaw = 0;
	int16_t myRaw = 0;
	int16_t mzRaw = 0;
	double ax = 0;
	double ay = 0;
	double az = 0;
	double gx = 0;		//  All gyro measurements in deg per second
	double gy = 0;
	double gz = 0;
	double mx = 0;
	double my = 0;
	double mz = 0;
	
	initMPU9250();
	sleep(2);		// Wait for mpu to stabilize
	
	int regValue;
	bool newData = false;
	wiringPiI2CWriteReg8(magHandle, 0x0A, 0x11);		//  Initiate mag reading 16-bit output
	start = micros();
	lastT = start;
	tAccGyro = micros();
	tMag = micros();
	
	for(;;) {
		// Check for new AccGyro data
		regValue = wiringPiI2CReadReg8(accGyroHandle, 58);
		if(regValue & 0x01) {
			//printf("%lu   %#x  ",  micros() - tAccGyro, ii);
			tAccGyro = micros();
			unsigned long t = micros();
			accGyroRead();
			t = t;		//  Avoid warning when print below is not used
			//printf("%lu\n", micros() - t);
			pthread_mutex_lock(&mutex1);
				accGyroCount++;
			pthread_mutex_unlock(&mutex1);
			newData = true;
		}
		
		// Check for new Mag data
		regValue = wiringPiI2CReadReg8(magHandle, 0x02);
		if(regValue & 0x01) {
		//printf("%lu              %lu  %#x   ",  micros() - tAccGyro, micros() - tMag, ii);
			tMag = micros();
			unsigned long t = micros();
			magRead();
			wiringPiI2CWriteReg8(magHandle, 0x0A, 0x11);		//  Initiate new mag reading
			t = t;		//  Avoid warning when print below is not used			
			//printf(" %lu\n", micros() - t);										//  16 bit output
			pthread_mutex_lock(&mutex1);
				magCount++;
			pthread_mutex_unlock(&mutex1);
			newData = true;
		}		
		
		if(newData) {		// If either accgyro or mag has new data convert to physical units and call fusion routines
			newData = false;

			axRaw = (axRead & 0x00FF) * 256 + (axRead >> 8);	// I2CReadReg16 swaps the bytes, swap back
			ayRaw = (ayRead & 0x00FF) * 256 + (ayRead >> 8);	//  and convert to signed int
			azRaw = (azRead & 0x00FF) * 256 + (azRead >> 8);
			//printf("%d  %d  %d \n", axRaw, ayRaw, azRaw);
			gxRaw = (gxRead & 0x00FF) * 256 + (gxRead >> 8);
			gyRaw = (gyRead & 0x00FF) * 256 + (gyRead >> 8);
			gzRaw = (gzRead & 0x00FF) * 256 + (gzRead >> 8);
			//printf("%d  %d  %d \n", gxRaw, gyRaw, gzRaw);	
			mxRaw = mxRead;						//  Magnetometer bytes are already swaped, just copy to signed int
			myRaw = myRead;
			mzRaw = mzRead;
			//printf("%d  %d  %d \n", mxRaw, myRaw, mzRaw);
			double ax0 = (double)axRaw * aRes;
			double ay0 = (double)ayRaw * aRes;
			double az0 = (double)azRaw * aRes;
			//printf("%f  %f  %f \n", ax, ay, az);
			gx = (double)gxRaw * gRes;		//  All gyro measurements in deg per second
			gy = (double)gyRaw * gRes;
			gz = (double)gzRaw * gRes;
			//printf("%f  %f  %f \n", gx, gy, gz);
			double mx0 = (double)mxRaw * mRes * magFactoryCal[0];	
			double my0 = (double)myRaw * mRes * magFactoryCal[1];
			double mz0 = (double)mzRaw * mRes * magFactoryCal[2];
			//printf("%f  %f  %f \n", mx, my, mz);
			
			// Adjust for calibration and normalize w=U*(v-c)
			double Uacc11 = 1.1322;
			double Uacc12 = 0.025;
			double Uacc13 = -0.0155;
			double Uacc22 = 1.1651;
			double Uacc23 = 0.0422;
			double Uacc33 = 1.1151;
			double cAcc1 = 0.0069;
			double cAcc2 = -0.0062;
			double cAcc3 = -0.0277;
			ax = Uacc11 * (ax0 -cAcc1) + Uacc12 * (ay0 -cAcc2) + Uacc13 * (az0 -cAcc3);
			ay = Uacc22 * (ay0 -cAcc2) + Uacc23 * (az0 -cAcc3);
			az = Uacc33 * (az0 -cAcc3);
			double Umag11 = 0.0020;
			double Umag12 = -0.0001;
			double Umag13 = 0;
			double Umag22 = 0.0020;
			double Umag23 = 0;
			double Umag33 = 0.0020;
			double cMag1 = 116.5123;
			double cMag2 = 74.4579;
			double cMag3 = -197.6982;
			mx = Umag11 * (mx0 -cMag1) + Umag12 * (my0 -cMag2) + Umag13 * (mz0 -cMag3);
			my = Umag22 * (my0 -cMag2) + Umag23 * (mz0 -cMag3);
			mz = Umag33 * (mz0 -cMag3);			
			
			/*
			// log of data for calibration (not thread safe)
			static int prCount = 100;
			prCount--;
			if(prCount <= 0) {
				fprintf(fp, "%f  %f  %f  %f  %f %f \n", ax , ay, az, mx, my, mz);
				printf("%f  %f  %f  %f  %f %f \n", ax , ay, az, mx, my, mz);
				prCount = 100;
			}
			*/

			double deltaT = (double)((micros() - lastT) / 1000000.0);
			lastT = micros();
			
			static int cCount = 1000;		// wait for mpu to stabilize to avoid nan return values
			if(cCount <= 0) {
				cavallo = updateCavallo(ax, -ay, -az, gx, -gy, -gz, -mx, my, mz, deltaT);		// filter 9 sep

				madgwick = updateMadgwick(ax, ay, az, gx, gy, gz, my, mx, -mz, deltaT);		// filter

				cCount = 0;		
			}
			cCount--;		
		}
		
		// Transfer values to global variables thread safe and get gps values
			
		pthread_mutex_lock(&mutex1);
			counter++;		
			cavalloG = cavallo;
			madgwickG = madgwick;
			double gpsCourse = gpsCourseG;
			double gpsSpeed = gpsSpeedG;
		pthread_mutex_unlock(&mutex1);		
		gpsCourse = gpsCourse;		// Silence compiler warnings
		gpsSpeed = gpsSpeed;
		
		// Do something with gps values later on, might be better to transfer correction values
	}
	return 0;
}


void accGyroRead(void) {
	axRead =wiringPiI2CReadReg16(accGyroHandle, 59);
	ayRead =wiringPiI2CReadReg16(accGyroHandle, 61);
	azRead =wiringPiI2CReadReg16(accGyroHandle, 63);
	gxRead =wiringPiI2CReadReg16(accGyroHandle, 67);
	gyRead =wiringPiI2CReadReg16(accGyroHandle, 69);
	gzRead =wiringPiI2CReadReg16(accGyroHandle, 71);
}

void magRead(void) {
	mxRead = wiringPiI2CReadReg16(magHandle, 0x03);
	myRead = wiringPiI2CReadReg16(magHandle, 0x05);
	mzRead = wiringPiI2CReadReg16(magHandle, 0x07);
}

void initMPU9250(void) {
	
	struct timespec ts;		//  delaytime for nanosleep function

	printf("\n*******************   Start initMPU9250   *********************\n\n");
	
	accGyroHandle = wiringPiI2CSetup(104);								// First, init accGyro
	printf("accGyroHandle = %d \n",  accGyroHandle);
	
	int i =wiringPiI2CReadReg8(accGyroHandle, 117);
	printf("Who_am_I should be 0x71: %#x \n",  i);

	i = wiringPiI2CWriteReg8(accGyroHandle, 0x6B, 0x00);		// PWR_MGMT_1 Clear sleep mode and enable all sensors
	printf("PWR_MGMT_1   %#x Clear sleep mode, enable all sensors\n", i);
	ts.tv_sec = 0;							//  Delay 100 ms
	ts.tv_nsec = 100000000;
	nanosleep(&ts, NULL);
	printf("Hej!!!\n");
	
	i = wiringPiI2CWriteReg8(accGyroHandle, 0x6B, 0x01);			// PWR_MGMT_1 PLL as clocksource if possible
	printf("PWR_MGMT_1   %#x PLL as clocksource if possible\n", i);
	ts.tv_sec = 0;							//  Delay 200 ms
	ts.tv_nsec = 200000000;
	nanosleep(&ts, NULL);
	/*
	//  Configure Gyro and Accelerometer, reg 0x1A, disable FSYNC, DLPF_CFG 2:0 = 010 = 0x2 gives delay
	//  3.9 ms ~250 Hz reading frequency, 1kHz saple rate
	i = wiringPiI2CWriteReg8(accGyroHandle, 0x1A, 0x02);	
	printf("CONFIG       %#x ~250 Hz reading frequency\n", i);
	i = wiringPiI2CWriteReg8(accGyroHandle, 0x19, 0x03);				//  SMPLRT_DIV 1 kHz / (1 + 3) => 0x03
	printf("SMPLRT_DIV   %#x ~250 Hz reading frequency\n", i);
	*/
	
	//  Configure Gyro and Accelerometer, reg 0x1A, disable FSYNC, DLPF_CFG 2:0 = 000 = 0x0 gives delay
	//  0,97 ms ~1000 Hz reading frequency, 1kHz saple rate
	i = wiringPiI2CWriteReg8(accGyroHandle, 0x1A, 0x00);	
	printf("CONFIG       %#x ~1000 Hz reading frequency\n", i);
	i = wiringPiI2CWriteReg8(accGyroHandle, 0x19, 0x00);				//  SMPLRT_DIV 8 kHz / (1 + 0) => 0x00
	printf("SMPLRT_DIV   %#x ~1000 Hz reading frequency\n", i);
	
	uint8_t c = wiringPiI2CReadReg8(accGyroHandle, 0x1B);			//  GYRO_CONFIG, clear selftest, Full Scale 250 deg/s
	i = wiringPiI2CWriteReg8(accGyroHandle, 0x1B, c & 0x04);
	printf("GYRO_CONFIG  %#x clear selftest, Full Scale 250 deg/s\n", i);
	gRes = 250 / 32768.0;
	
	c = wiringPiI2CReadReg8(accGyroHandle, 0x1C);						//  ACCEL_CONFIG, clear selftest, Full Scale 2g
	i = wiringPiI2CWriteReg8(accGyroHandle, 0x1C, c & 0x07);
	printf("ACCEL_CONFIG %#x clear selftest, Full Scale 2g\n", i);
	aRes = 2 / 37268.0;

	i = wiringPiI2CWriteReg8(accGyroHandle, 0x37, 0x22);			// INT_Pin, i2c bypass enable
	printf("INT_Pin      %#x i2c bypass enable\n", i);

	i = wiringPiI2CWriteReg8(accGyroHandle, 0x38, 0x01);			// INT_Pin, i2c interrupt enable
	printf("INT_Pin      %#x INT_Pin, i2c interrupt enable\n\n", i);
	
	magHandle = wiringPiI2CSetup(0x0C);										// Second,init magnetometer
	printf("magHandle = %d \n",  magHandle);
	
	i =wiringPiI2CReadReg8(magHandle, 0x00);
	printf("Who_am_I should be 0x48: %#x \n",  i);
	
	i = wiringPiI2CWriteReg8(magHandle, 0x0A, 0x00);		// CNTL Power down
	printf("CNTL   %#x Power down\n", i);
	ts.tv_sec = 0;							//  Delay 10 ms
	ts.tv_nsec = 10000000;
	nanosleep(&ts, NULL);
	
	i = wiringPiI2CWriteReg8(magHandle, 0x0A, 0x0F);		// CNTL Enter fuse ROM access mode
	printf("CNTL   %#x Enter fuse ROM access mode\n", i);
	ts.tv_sec = 0;							//  Delay 10 ms
	ts.tv_nsec = 10000000;
	nanosleep(&ts, NULL);
	
	int xCal = wiringPiI2CReadReg8(magHandle, 0x10);			//  ASAX, sensitivity adjustment onstant
	printf("ASAX   %#x Sensitivity adjustment onstant\n", xCal);
	int yCal = wiringPiI2CReadReg8(magHandle, 0x11);			//  ASAY, sensitivity adjustment onstant
	printf("ASAY   %#x Sensitivity adjustment onstant\n", yCal);
	int zCal = wiringPiI2CReadReg8(magHandle, 0x12);			//  ASAZ, sensitivity adjustment onstant
	printf("ASAZ   %#x Sensitivity adjustment onstant\n", zCal);
	
	magFactoryCal[0] = (xCal - 128) * 0.5 / 128 + 1;					//  Calculate adjustment factors
	magFactoryCal[1] = (yCal - 128) * 0.5 / 128 + 1;					//  Calculate adjustment factors
	magFactoryCal[2] = (zCal - 128) * 0.5 / 128 + 1;					//  Calculate adjustment factors
	printf("Correction factor mag x: %f \n", magFactoryCal[0]);
	printf("Correction factor mag y: %f \n", magFactoryCal[1]);
	printf("Correction factor mag z: %f \n", magFactoryCal[2]);
	
	ts.tv_sec = 0;							//  Delay 2 s
	ts.tv_nsec = 2000000000;
	nanosleep(&ts, NULL);
	
	printf("\n*******************    End initMPU9250    *********************\n\n");
	
}

