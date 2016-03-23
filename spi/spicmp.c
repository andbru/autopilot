 
 #include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#include "madgwick.h"
#include "cavallo.h"





//  Function prototypes
void initMPU9250(void);
bool pollAccGyro(double *pax, double *pay, double *paz, double *pgx, double *pgy, double *pgz);
bool pollMag(double *pmx, double *pmy, double *pmz);
double elapsed(struct timeval t1, struct timeval t0);
int writeMpu(unsigned char reg, unsigned char byte);
unsigned char readMpu(unsigned char reg);
void mReadMpu(unsigned char reg, unsigned char buf[], int len);
void sleepMs(int t);
unsigned char bReadAk(unsigned char regAk);
int bWriteAk(unsigned char regAk, unsigned char byte);
void magReadAk(void);

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
	int hit2 = 0;
	int hit3 = 0;
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
		//printf("%f  ", dAG);

		if(newAccGyroData) countAccGyro++;

		gettimeofday(&t0, NULL);
		bool newMagData = pollMag(&mx, &my, &mz);
		gettimeofday(&t1, NULL);
		dAG = elapsed(t1, t0);
		//printf("%f\n", dAG);

		if(newMagData) countMag++;

		if(newAccGyroData | newMagData) {

			//printf("%f  %f  %f  %f  %f  %f\n", ax, ay, az, gx, gy, gz);
			
			gettimeofday(&tNow, NULL);
			double dt = elapsed(tNow, tOld);
			tOld = tNow;
			if(dt > 2.0) hit2++;
			if(dt > 3.0) hit3++;
			if(dt > 4.0) hit4++;
			if(dt > 5.0) hit5++;
			if(dt > 7.0) hit7++;
			if(dt > 10.0) hit10++;
			
				 
			cavallo = updateCavallo(ax, -ay, -az, gx*M_PI/180, -gy*M_PI/180,	// to radians
					 -gz*M_PI/180, -mx, my, mz, dt / 1000.0);		// align magnetometer

					 
			yaw = cavallo.yaw;
			w = cavallo.w;
			wdot = cavallo.wdot;					 
		
			//printf("%f   %d  %d     %f\n   ", yaw, newAccGyroData, newMagData, dt);
			
			if(dt>3.0) printf("%d  %d  %d  %d  %d  %d\n", hit2, hit3, hit4, hit5, hit7, hit10);	
		}
	}
	gettimeofday(&tStop, NULL);
	printf("%d  %d  %d  %d\n", hit4, hit5, hit7, hit10);
	printf("Total exekveringstid: %f  sekunder,  AccGyro %d gånger och Mag %d gånger\n\n\n", elapsed(tStop, tStart)/1000.0, countAccGyro, countMag);
	return 0;
}

bool pollAccGyro(double  *pax, double *pay, double *paz, double *pgx, double *pgy, double *pgz) {

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
	
	//bool nd = wiringPiI2CReadReg8(accGyroHandle, 58);
	unsigned char reg3A = readMpu(0x3A);					// Test if RAW_DATA_RDY_INT
	if(reg3A & 0x01) {
	
		unsigned char buf[14];									// Read acc/gyro raw data to buffer
		mReadMpu(0x3B, buf, 14);
		
		axRaw = buf[0] * 256 + buf[1];							// Bytes are swaped, combine to 16 bit raw data
		ayRaw = buf[2] * 256 + buf[3];
		azRaw = buf[4] * 256 + buf[5];
		//printf("%d  %d  %d \n", axRaw, ayRaw, azRaw);
		gxRaw = buf[8] * 256 + buf[9];
		gyRaw = buf[10] * 256 + buf[11];
		gzRaw = buf[12] * 256 + buf[13];
	
		double axp = (double)axRaw * aRes - cAcc1;		// Scale and adjust for calibration
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

bool pollMag(double *pmx, double *pmy, double *pmz) {

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

	if((readMpu(0x34) & 0x80) == 0) {		// Read/Write operation finished
		//printf("0x34 == 0\n");
		unsigned char reg0x35 = readMpu(0x35);
		// Start new read operation
		writeMpu(0x31, 0x8C);		// Read + device address AK8963
		writeMpu(0x32, 0x02);			// AK8963 register to read
		writeMpu(0x34, 0x80);			// Enable i2c read
		if((reg0x35 & 0x01) == 1) {		// Check if data rdy
			//printf("0x35 == 1\n");

			// ***************************************************************
			// Read 6 bytes data from magnetometer to MPU reg 0x49
			// ***************************************************************
			magReadAk();

			// Read from MPU reg0x49 to magData[] array
			unsigned char magData[6];
			mReadMpu(0x49, magData, 6);
		
			mxRaw = magData[1] * 256 + magData[0];							// Bytes are not swaped, combine to 16 bit raw data
			myRaw = magData[3] * 256 + magData[2];
			mzRaw = magData[5] * 256 + magData[4];

			double mxp = (double)mxRaw * mRes * magFactoryCal[0] - cMag1;	
			double myp = (double)myRaw * mRes * magFactoryCal[1] - cMag2;
			double mzp = (double)mzRaw * mRes * magFactoryCal[2] - cMag3;
			*pmx = UMag11 * mxp + UMag12 * myp + UMag13 * mzp;
			*pmy =                                UMag22 * myp + UMag23 * mzp;
			*pmz =                                                               UMag33 * mzp;
		
			//wiringPiI2CWriteReg8(magHandle, 0x0A, 0x11);		//  Initiate new mag reading
			// ****************************************************************************
			// Start mag sample 16 bit single shot
			// ****************************************************************************
			writeMpu(0x27, 0x00);		// Ensure that SLV_0 is disabled
			bWriteAk(0x0A, 0x11);
			
			return true;
		}
	}
	return false;
}


void initMPU9250(void) {

	printf("\n*******************   Start initMPU9250   *********************\n\n");
	
	//accGyroHandle = wiringPiI2CSetup(0x68);								// First, init accGyro
	//printf("accGyroHandle = %d \n",  accGyroHandle);
	wiringPiSPISetup(0, 1000000);		// Channel 0, speed 1 MHz
	
	// PWR_MGMT_1 Clear sleep mode and enable all sensors
	int success = writeMpu(0x6B, 0x00);
	printf("\n\nPWR_MGMT_1   %#x  Clear sleep mode, enable all sensors\n", success);
	sleepMs(100);
		
	// Disable i2c, enable i2c-master
	success = writeMpu(0x6A, 0x30);
	printf("USER_CTRL    %#x  Disable i2c, enable i2c-master\n", success);
	
	// Read Who_Am_I reg of acc/gyro
	unsigned char c = readMpu(0x75);
	printf("WHO_AM_I     %#x Who am I should be 0x71  \n", c);
	
	// PWR_MGMT_1 PLL as clocksource if possible
	success = writeMpu(0x6B, 0x01);		
	printf("PWR_MGMT_1   %#x PLL as clocksource if possible\n", success);
	sleepMs(200);
	
	//  Configure Gyro and Accelerometer, reg 0x1A, disable FSYNC, DLPF_CFG 2:0 = 010 = 0x2 gives delay
	//  3.9 ms ~250 Hz reading frequency, 1kHz saple rate
	success = writeMpu(0x1A, 0x02);
	printf("CONFIG       %#x ~250 Hz reading frequency\n", success);
	success = writeMpu(0x19, 0x03);								//  SMPLRT_DIV 1 kHz / (1 + 3) => 0x03
	printf("SMPLRT_DIV   %#x ~250 Hz reading frequency\n", success);
	
	//  GYRO_CONFIG, clear selftest [7:5], Fchoice_b = 00 [1:0], Full Scale 250 deg/s [4:3] = 0b00
	//  All bits are set to zero, no read necessary
	success = writeMpu(0x1B, 0x00);	
	printf("GYRO_CONFIG  %#x clear selftest, Full Scale 250 deg/s\n", success);
	gRes = 250 / 32768.0;
	
	//  ACCEL_CONFIG, clear selftest [7:5], Full Scale 2g [4:3] = 0b00 [2:0] reserved but always zero
	//  All bits are set to zero, no read necessary
	success = writeMpu(0x1C, 0x00);	
	printf("ACCEL_CONFIG %#x clear selftest, Full Scale 2g\n", success);
	aRes = 2 / 37268.0;
	
	success = writeMpu(0x37, 0x20);					// INT_Pin level hold, no i2c bypass
	printf("INT_Pin      %#x no i2c bypass \n", success);

	success = writeMpu(0x38, 0x01);					// INT_Pin, i2c interrupt enable
	printf("INT_Pin      %#x INT_Pin, i2c interrupt enable\n\n", success);
	
	//***********************************************************************************
	// Read Who_Am_I reg (0x00) of magnetometer (addr = 0x0C) to verify i2c connection
	// ***********************************************************************************
	unsigned char who = bReadAk(0x00);
	printf("Magnetometer: Who_am_I should be 0x48: %#x \n",  who);
	
	// ****************************************************************************
	// Reset magnetometer with power down
	// ****************************************************************************
	bWriteAk(0x0A, 0x00);
	sleepMs(10);

	// CNTL Enter fuse ROM access mode
	bWriteAk(0x0A, 0xFF);
	sleepMs(10);
	
	unsigned char xCal = bReadAk(0x10);										//  ASAX, sensitivity adjustment constant
	printf("ASAX   %#x Sensitivity adjustment onstant\n", xCal);
	unsigned char yCal = bReadAk(0x11);										//  ASAY, sensitivity adjustment constant
	printf("ASAY   %#x Sensitivity adjustment onstant\n", yCal);	
	unsigned char zCal = bReadAk(0x12);										//  ASAZ, sensitivity adjustment constant
	printf("ASAZ   %#x Sensitivity adjustment onstant\n", zCal);
	
	magFactoryCal[0] = (xCal - 128) * 0.5 / 128 + 1;					//  Calculate adjustment factors
	magFactoryCal[1] = (yCal - 128) * 0.5 / 128 + 1;					//  Calculate adjustment factors
	magFactoryCal[2] = (zCal - 128) * 0.5 / 128 + 1;					//  Calculate adjustment factors
	printf("Correction factor mag x: %f \n", magFactoryCal[0]);
	printf("Correction factor mag y: %f \n", magFactoryCal[1]);
	printf("Correction factor mag z: %f \n", magFactoryCal[2]);
	
	// ****************************************************************************
	// Start mag sample 16 bit single shot
	// ****************************************************************************
	writeMpu(0x27, 0x00);		// Ensure that SLV_0 is disabled
	bWriteAk(0x0A, 0x11);
	printf("\nMag reading 16 bit single shot initiated\n");
	
	printf("\n*******************    End initMPU9250    *********************\n\n");
}


double elapsed(struct timeval t1, struct timeval t0){

	double elapsed = (t1.tv_sec - t0.tv_sec) * 1000;		// milliseconds
	elapsed += (t1.tv_usec - t0.tv_usec) / 1000.0;

	return elapsed;

}


int writeMpu(unsigned char reg, unsigned char byte) {

	unsigned char buf[2];
	
	buf[0] = reg;
	buf[1] = byte;
	int success = wiringPiSPIDataRW(0, buf, 2);
	
	return success;
}


unsigned char readMpu(unsigned char reg) {

	unsigned char buf[2];
	
	buf[0] = reg | 0x80;
	buf[1] = 0x00;
	wiringPiSPIDataRW(0, buf, 2);
	
	return buf[1];
}


void mReadMpu(unsigned char reg, unsigned char buf[], int len) {

	unsigned char b[100];
	
	b[0] = reg | 0x80;
	b[1] = 0x00;
	wiringPiSPIDataRW(0, b, len + 1);

	int i;
	for(i = 0; i < len; i++) {
		buf[i] = b[i + 1];
	}	
}


void sleepMs(int t) {

	struct timespec ts;					//  delaytime for nanosleep function
	ts.tv_sec = 0;							//  Delay 100 ms
	ts.tv_nsec = t * 1000000;
	nanosleep(&ts, NULL);
}


unsigned char bReadAk(unsigned char regAk) {

	// Use SLV_4 for byte communication over i2c
	writeMpu(0x31, 0x8C);		// Read + device address AK8963
	writeMpu(0x32, regAk);		// AK8963 register to read
	writeMpu(0x34, 0x80);			// Enable i2c read
	
	unsigned char c = 0x80;
	while(c & 0x80) {				// Wait for enable bit to be cleared
		c = readMpu(0x34);
	}
	
	return readMpu(0x35);
}


int bWriteAk(unsigned char regAk, unsigned char byte) {

	// Use SLV_4 for byte communication over i2c
	writeMpu(0x33, byte);			// Store data to write to AK8963
	writeMpu(0x31, 0x0C);		// Write + device address AK8963
	writeMpu(0x32, regAk);		// AK8963 register to read
	writeMpu(0x34, 0x80);			// Enable i2c write
	/*
	unsigned char c = 0x80;
	while(c & 0x80) {				// Wait for enable bit to be cleared
		c = readMpu(0x34);
	}
	*/
	return 1;
}

void magReadAk(void) {

	// Use SLV_4 for byte communication over i2c
	writeMpu(0x25, 0x8C);	// Read + device address AK8963
	writeMpu(0x26, 0x03);		// AK8963 register to read
	writeMpu(0x27, 0x86);		// Enable i2c read 6 bytes to EXT_SENSE_DATA_0
	/*
	unsigned char buf[4];
	buf[0] = 0x25;
	buf[1] = 0x8C;
	buf[2] = 0x03;
	buf[3] = 0x86;
	wiringPiSPIDataRW(0, buf, 4);
	*/
}
