 
#include "global.h"
#include "madgwick.h"
#include "watson.h"
#include "cavallo.h"

int main() {
	
	initMPU9250();

	struct taitBryanYPR madgwick;
	struct taitBryanYPR watson;
	struct taitBryanYPR cavallo;

	//FILE * fh = fopen("/media/usb/cmplog.txt", "w+");
	//FILE * fh = fopen("/media/pi/ARENASTADEN1/cmplog.txt", "w+");
	//fprintf(fh, "ax ay az ");
	//fprintf(fh, "gx gy gz ");
	//fprintf(fh, "mx my mz ");
	//fprintf(fh, "yawCav yawWat yawMadg ");
	//fprintf(fh, " deltaT \n");
	
	int ii;
	int magCount = 0;
	int accGyroCount = 0;
	bool newData = false;
	wiringPiI2CWriteReg8(magHandle, 0x0A, 0x11);		//  Initiate mag reading 16-bit output
	start = micros();
	lastT = start;
	tAccGyro = micros();
	tMag = micros();
	//while(micros() - start < 17000000) {
	while(true) {		
		ii = wiringPiI2CReadReg8(accGyroHandle, 58);
		if(ii & 0x01) {
		//printf("%lu   %#x  ",  micros() - tAccGyro, ii);
			tAccGyro = micros();
			unsigned long t = micros();
			accGyroRead();
			t = t;		//  Avoid warning when print below is not used
			//printf("%lu\n", micros() - t);
			accGyroCount++;
			newData = true;
		}
		
		ii = wiringPiI2CReadReg8(magHandle, 0x02);
		if(ii & 0x01) {
		//printf("%lu              %lu  %#x   ",  micros() - tAccGyro, micros() - tMag, ii);
			tMag = micros();
			unsigned long t = micros();
			magRead();
			wiringPiI2CWriteReg8(magHandle, 0x0A, 0x11);		//  Initiate new mag reading
			t = t;		//  Avoid warning when print below is not used			
			//printf(" %lu\n", micros() - t);										//  16 bit output
			magCount++;
			newData = true;
		}
		
		if(newData) {
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
			ax = (double)axRaw * aRes;
			ay = (double)ayRaw * aRes;
			az = (double)azRaw * aRes;
			//printf("%f  %f  %f \n", ax, ay, az);
			gx = (double)gxRaw * gRes;		//  All gyro measurements in deg per second
			gy = (double)gyRaw * gRes;
			gz = (double)gzRaw * gRes;
			//printf("%f  %f  %f \n", gx, gy, gz);
			mx = (double)mxRaw * mRes * magFactoryCal[0];	
			my = (double)myRaw * mRes * magFactoryCal[1];
			mz = (double)mzRaw * mRes * magFactoryCal[2];
			//printf("%f  %f  %f \n", mx, my, mz);

			double deltaT = (double)((micros() - lastT) / 1000000.0);
			lastT = micros();
			
			/* This works		 
			cavallo = updateCavallo(ax, -ay, -az, gx*M_PI/180, -gy*M_PI/180,	// to radians
					 -gz*M_PI/180, -mx, my, mz, deltaT);		// align magnetometer

			madgwick = updateMadgwick(ax, ay, az, gx*M_PI/180, gy*M_PI/180,	// to radians
					 gz*M_PI/180, my, mx, -mz, deltaT);		// align magnetometer
					 
			 watson = updateWatson(ax, ay, az, gx*M_PI/180, gy*M_PI/180,	// to radians
					 gz*M_PI/180, my, mx, -mz, deltaT);		// align magnetometer
			*/
			
			cavallo = updateCavallo(ax, -ay, -az, gx*M_PI/180, -gy*M_PI/180,	// to radians
					 -gz*M_PI/180, -mx, my, mz, deltaT);		// align magnetometer

			madgwick = updateMadgwick(ax, ay, az, gx*M_PI/180, gy*M_PI/180,	// to radians
					 gz*M_PI/180, my, mx, -mz, deltaT);		// align magnetometer
					 
			 watson = updateWatson(ax, ay, az, gx*M_PI/180, gy*M_PI/180,	// to radians
					 gz*M_PI/180, my, mx, -mz, deltaT);		// align magnetometer
			
			if (micros() -start > 0) {
			//if (micros() -start > 10000000) {
				printf("%f  %f  %f    ", cavallo.yaw, watson.yaw, madgwick.yaw);
				//printf(" %f \n", deltaT);
				printf(" %f \n", cavallo.roll);

				//fprintf(fh, "%f  %f  %f  ", ax, ay, az);
				//fprintf(fh, "%f  %f  %f  ", gx, gy, gz);
				//fprintf(fh, "%f  %f  %f  ", mx, my, mz);
				//fprintf(fh, "%f  %f  %f  ", cavallo.yaw, watson.yaw, madgwick.yaw);
				//fprintf(fh, "   %f \n", deltaT);
				//fprintf(fh, " %f \n", cavallo.roll);
			}
			
		}
	}
	printf("\n\n# of AccGyro readings: %d     # of Mag readings: %d\n\n", accGyroCount, magCount);
	//fclose(fh);
	
	lastTime = true;
	cavallo = updateCavallo(ax, ay, az, gx*M_PI/180, gy*M_PI/180,	// to radians
			 gz*M_PI/180, my, mx, mz, 0.001);		// align magnetometer
	
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
	
	i = wiringPiI2CWriteReg8(accGyroHandle, 0x6B, 0x01);			// PWR_MGMT_1 PLL as clocksource if possible
	printf("PWR_MGMT_1   %#x PLL as clocksource if possible\n", i);
	ts.tv_sec = 0;							//  Delay 200 ms
	ts.tv_nsec = 200000000;
	nanosleep(&ts, NULL);
	
	//  Configure Gyro and Accelerometer, reg 0x1A, disable FSYNC, DLPF_CFG 2:0 = 010 = 0x2 gives delay
	//  3.9 ms ~250 Hz reading frequency, 1kHz saple rate
	i = wiringPiI2CWriteReg8(accGyroHandle, 0x1A, 0x02);	
	printf("CONFIG       %#x ~250 Hz reading frequency\n", i);
	i = wiringPiI2CWriteReg8(accGyroHandle, 0x19, 0x03);				//  SMPLRT_DIV 1 kHz / (1 + 3) => 0x03
	printf("SMPLRT_DIV   %#x ~250 Hz reading frequency\n", i);
	
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
	
	printf("\n*******************    End initMPU9250    *********************\n\n");
	
}
