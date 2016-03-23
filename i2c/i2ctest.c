 
 #include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "madgwick.h"
#include "cavallo.h"

//  Function prototypes
void initMPU9250(void);
bool pollAccGyro(double *pax, double *pay, double *paz, double *pgx, double *pgy, double *pgz);
bool magRead(double *pmx, double *pmy, double *pmz);
double elapsed(struct timeval t1, struct timeval t0);

// Global variables
int accGyroHandle;			//  i2c variables
int magHandle;
float aRes = 0;				//  resolution accelerometer
float gRes = 0;				//  resolution gyro
float mRes = 1.5;			//  resolution magnetometer 1,5 mG = 0,15 microT
double magFactoryCal[3] = {0, 0, 0};	//Factory calibration of magnetometer read from mpu
//  Struct for YPR return values
struct filter {
	double yaw;
	double w;
	double wdot;
};


int main() {

	struct filter cavallo;
	
	double ax = 0;		// Latest measurements
	double ay = 0;
	double az = 0;
	double gx = 0;
	double gy = 0;
	double gz = 0;
	double mx = 0;
	double my = 0;
	double mz = 0;
	
	double yaw = 0;		// Sensor fusion result
	double w = 0;
	double wdot = 0;
	
	struct timeval tNow, tOld;
	struct timeval tStart, tStop;
	struct timeval t0, t1;
	gettimeofday(&tOld, NULL);
	
	initMPU9250();

	wiringPiI2CWriteReg8(magHandle, 0x0A, 0x11);		//  Initiate mag reading 16-bit output

	int count = 0;
	int countAccGyro = 0;
	int countMag = 0;
	int hit4 = 0;
	int hit5 = 0;
	int hit7 = 0;
	int hit10 = 0;
	gettimeofday(&tStart, NULL);
	while(count < 20000000) {	
		count++;	
		
		gettimeofday(&t0, NULL);	
		bool newAccGyroData = pollAccGyro(&ax, &ay, &az, &gx, &gy, &gz);
		gettimeofday(&t1, NULL);
		double dAG = elapsed(t1, t0);
		//printf("tag = %f  ", dAG);

		if(newAccGyroData) countAccGyro++;

		gettimeofday(&t0, NULL);
		bool newMagData = magRead(&mx, &my, &mz);
		gettimeofday(&t1, NULL);
		dAG = elapsed(t1, t0);
		//printf("tm = %f  ", dAG);

		if(newMagData) countMag++;

		if(newAccGyroData | newMagData) {

			gettimeofday(&tNow, NULL);
			double dt = elapsed(tNow, tOld);
			tOld = tNow;
			if(dt > 4.0) hit4++;
			if(dt > 5.0) hit5++;
			if(dt > 7.0) hit7++;
			if(dt > 10.0) hit10++;
			
			gettimeofday(&t0, NULL);
/*		 
			cavallo = updateCavallo(ax, -ay, -az, gx*M_PI/180, -gy*M_PI/180,	// to radians
					 -gz*M_PI/180, -mx, my, mz, dt / 1000.0);		// align magnetometer
*/
			cavallo = updateCavallo(ax, -ay, -az, gx*M_PI/180, -gy*M_PI/180,	// to radians
					 -gz*M_PI/180, my, -mx, mz, dt / 1000.0);		// align magnetometer
			gettimeofday(&t1, NULL);
			dAG = elapsed(t1, t0);
			//printf("tk = %f", dAG);
					 
			yaw = cavallo.yaw;
			w = cavallo.w;
			wdot = cavallo.wdot;					 
		
			//printf("  %f  %f  %f    %f  %f  %f    %f  %f  %f\n", ax, -ay, -az, gx, -gy, -gz, my, -mx, mz);
			printf("%f   %d  %d     %f\n   ", yaw, newAccGyroData, newMagData, dt);
			
			//if(dt>4.0) printf("%d  %d  %d  %d\n", hit4, hit5, hit7, hit10);	
		}
		//printf("\n");
	}
	gettimeofday(&tStop, NULL);
	printf("%d  %d  %d  %d\n", hit4, hit5, hit7, hit10);
	printf("Total exekveringstid: %f  sekunder,  AccGyro %d gånger och Mag %d gånger\n\n\n", elapsed(tStop, tStart)/1000.0, countAccGyro, countMag);
	return 0;
}

bool pollAccGyro(double  *pax, double *pay, double *paz, double *pgx, double *pgy, double *pgz) {

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
	
	double UAcc11 = 1.0484;
	double UAcc12 = 0.1368;
	double UAcc13 = -0.0310;
	double UAcc22 = 1.1193;
	double UAcc23 = -0.0276;
	double UAcc33 = 1.1504;
	double cAcc1 = -0.0152;
	double cAcc2 = -0.0330;
	double cAcc3 = -0.0433;
	
	bool nd = wiringPiI2CReadReg8(accGyroHandle, 58);
	if(nd & 0x01) {

		unsigned char buf[2];
		buf[0] = 0x3B;
		buf[1] = 0x00;
		unsigned char raw[14];
		write(accGyroHandle, buf, 1);		// Dummy write to set address
		read(accGyroHandle, &raw, 14);		// Read 14 bytes, skip temperature
		//printf("%#x  %#x  %#x  %#x  %#x  %#x \n", raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
		//printf("%#x  %#x  %#x  %#x  %#x  %#x \n", raw[8], raw[9], raw[10], raw[11], raw[12], raw[13]);
		
		axRaw = raw[0] * 256 + raw[1];
		ayRaw = raw[2] * 256 + raw[3];
		azRaw = raw[4] * 256 + raw[5];
		gxRaw = raw[8] * 256 + raw[9];
		gyRaw = raw[10] * 256 + raw[11];
		gzRaw = raw[12] * 256 + raw[13];	
		//printf("%d  %d  %d \n", axRaw, ayRaw, azRaw);
		//printf("%d  %d  %d \n", gxRaw, gyRaw, gzRaw)

/*
		axRead =wiringPiI2CReadReg16(accGyroHandle, 59);
		ayRead =wiringPiI2CReadReg16(accGyroHandle, 61);
		azRead =wiringPiI2CReadReg16(accGyroHandle, 63);
		gxRead =wiringPiI2CReadReg16(accGyroHandle, 67);
		gyRead =wiringPiI2CReadReg16(accGyroHandle, 69);
		gzRead =wiringPiI2CReadReg16(accGyroHandle, 71);
	
		axRaw = (axRead & 0x00FF) * 256 + (axRead >> 8);	// I2CReadReg16 swaps the bytes, swap back
		ayRaw = (ayRead & 0x00FF) * 256 + (ayRead >> 8);	//  and convert to signed int
		azRaw = (azRead & 0x00FF) * 256 + (azRead >> 8);
		//printf("%d  %d  %d \n", axRaw, ayRaw, azRaw);
		gxRaw = (gxRead & 0x00FF) * 256 + (gxRead >> 8);
		gyRaw = (gyRead & 0x00FF) * 256 + (gyRead >> 8);
		gzRaw = (gzRead & 0x00FF) * 256 + (gzRead >> 8);
*/	
		double axp = (double)axRaw * aRes - cAcc1;
		double ayp = (double)ayRaw * aRes - cAcc2;
		double azp = (double)azRaw * aRes - cAcc3;
		*pax = UAcc11 * axp + UAcc12 * ayp +UAcc13 * azp;
		*pay =	                             UAcc22 * ayp +UAcc23 * azp;
		*paz =	                                                        UAcc33 * azp;

		*pgx = (double)gxRaw * gRes;		//  All gyro measurements in deg per second
		*pgy = (double)gyRaw * gRes;
		*pgz = (double)gzRaw * gRes;
	
		return true;
	}
	return false;
}

bool magRead(double *pmx, double *pmy, double *pmz) {

	uint16_t mxRead = 0;
	uint16_t myRead = 0;
	uint16_t mzRead = 0;
	int16_t mxRaw = 0;
	int16_t myRaw = 0;
	int16_t mzRaw = 0;
	
	double UMag11 = 0.0021643;
	double UMag12 = -0.0001458;
	double UMag13 = 0.0002246;
	double UMag22 = 0.0020968;
	double UMag23 = 0.0000382;
	double UMag33 = 0.0019946;
	double cMag1 = 82.1332;
	double cMag2 = 62.7353;
	double cMag3 = -125.644;

	unsigned char buf[2];
	buf[0] = 0x02;
	buf[1] = 0x00;
	unsigned char AkReg0x02;
	write(magHandle, buf, 1);		// Dummy write to set address
	read(magHandle, &AkReg0x02, 1);
	//bool nd = wiringPiI2CReadReg8(magHandle, 0x02);
	if(AkReg0x02 & 0x01) {

		unsigned char buf[2];
		buf[0] = 0x03;
		buf[1] = 0x00;
		unsigned char raw[6];
		write(magHandle, buf, 1);		// Dummy write to set address
		read(magHandle, &raw, 6);		// Read 14 bytes, skip temperature
		//printf("%#x  %#x  %#x  %#x  %#x  %#x \n", raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
		
		mxRaw = raw[1] * 256 +raw[0];
		myRaw = raw[3] * 256 +raw[2];
		mzRaw = raw[5] * 256 +raw[4];

/*
		mxRead = wiringPiI2CReadReg16(magHandle, 0x03);
		myRead = wiringPiI2CReadReg16(magHandle, 0x05);
		mzRead = wiringPiI2CReadReg16(magHandle, 0x07);
	
		mxRaw = mxRead;						//  Magnetometer bytes are already swaped, just copy to signed int
		myRaw = myRead;
		mzRaw = mzRead;
*/
		double mxp = (double)mxRaw * mRes * magFactoryCal[0] - cMag1;	
		double myp = (double)myRaw * mRes * magFactoryCal[1] - cMag2;
		double mzp = (double)mzRaw * mRes * magFactoryCal[2] - cMag3;
		*pmx = UMag11 * mxp + UMag12 * myp + UMag13 * mzp;
		*pmy =                                UMag22 * myp + UMag23 * mzp;
		*pmz =                                                               UMag33 * mzp;

		//wiringPiI2CWriteReg8(magHandle, 0x0A, 0x11);		//  Initiate new mag reading

		//  Initiate new mag reading
		buf[0] = 0x0A;
		buf[1] = 0x11;
		write(magHandle, buf, 2);
		
		return true;
	}
	return false;
}



void initMPU9250(void) {

	printf("\n*******************   Start initMPU9250   *********************\n\n");
	
	accGyroHandle = wiringPiI2CSetup(0x68);								// First, init accGyro
	printf("accGyroHandle = %d \n",  accGyroHandle);
	
	int i =wiringPiI2CReadReg8(accGyroHandle, 0x75);
	printf("Who_am_I should be 0x71: %#x \n",  i);

	i = wiringPiI2CWriteReg8(accGyroHandle, 0x6B, 0x00);		// PWR_MGMT_1 Clear sleep mode and enable all sensors
	printf("PWR_MGMT_1   %#x Clear sleep mode, enable all sensors\n", i);
	struct timespec ts;					//  delaytime for nanosleep function
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
	
	//  GYRO_CONFIG, clear selftest [7:5], Fchoice_b = 00 [1:0], Full Scale 250 deg/s [4:3] = 0b00
	//  All bits are set to zero, no read necessary
	i = wiringPiI2CWriteReg8(accGyroHandle, 0x1B, 0x00);
	printf("GYRO_CONFIG  %#x clear selftest, Full Scale 250 deg/s\n", i);
	gRes = 250 / 32768.0;
	
	//  ACCEL_CONFIG, clear selftest [7:5], Full Scale 2g [4:3] = 0b00 [2:0] reserved but always zero
	//  All bits are set to zero, no read necessary
	i = wiringPiI2CWriteReg8(accGyroHandle, 0x1C, 0x00);
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
	
	int xCal = wiringPiI2CReadReg8(magHandle, 0x10);			//  ASAX, sensitivity adjustment constant
	printf("ASAX   %#x Sensitivity adjustment constant\n", xCal);
	int yCal = wiringPiI2CReadReg8(magHandle, 0x11);			//  ASAY, sensitivity adjustment constant
	printf("ASAY   %#x Sensitivity adjustment constant\n", yCal);
	int zCal = wiringPiI2CReadReg8(magHandle, 0x12);			//  ASAZ, sensitivity adjustment constant
	printf("ASAZ   %#x Sensitivity adjustment constant\n", zCal);
	
	magFactoryCal[0] = (xCal - 128) * 0.5 / 128 + 1;					//  Calculate adjustment factors
	magFactoryCal[1] = (yCal - 128) * 0.5 / 128 + 1;					//  Calculate adjustment factors
	magFactoryCal[2] = (zCal - 128) * 0.5 / 128 + 1;					//  Calculate adjustment factors
	printf("Correction factor mag x: %f \n", magFactoryCal[0]);
	printf("Correction factor mag y: %f \n", magFactoryCal[1]);
	printf("Correction factor mag z: %f \n", magFactoryCal[2]);
	
	printf("\n*******************    End initMPU9250    *********************\n\n");
	
}


double elapsed(struct timeval t1, struct timeval t0){

	double elapsed = (t1.tv_sec - t0.tv_sec) * 1000;		// milliseconds
	elapsed += (t1.tv_usec - t0.tv_usec) / 1000.0;

	return elapsed;

}
