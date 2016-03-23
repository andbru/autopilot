 
 #include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

int writeMpu(unsigned char reg, unsigned char byte);
unsigned char readMpu(unsigned char reg);
void mReadMpu(unsigned char reg, unsigned char buf[], int len);
void sleepMs(int t);
unsigned char bReadAk(unsigned char regAk);
int bWriteAk(unsigned char regAk, unsigned char byte);
void magReadAk(void);

int main() {

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
	
	//***********************************************************************************
	// Read Who_Am_I reg (0x00) of magnetometer (addr = 0x0C) to verify i2c connection
	// ***********************************************************************************
	unsigned char who = bReadAk(0x00);
	printf("Who_am_I should be 0x48: %#x \n",  who);
	
	// ****************************************************************************
	// Reset magnetometer with power down
	// ****************************************************************************
	bWriteAk(0x0A, 0x00);
	sleepMs(100);

	// ****************************************************************************
	// Start mag sample 16 bit single shot
	// ****************************************************************************
	writeMpu(0x27, 0x00);		// Ensure that SLV_0 is disabled
	bWriteAk(0x0A, 0x11);

	// ****************************************************************
	// Wait for data rdy, check bit0 in AK8963 reg 0x02
	// ****************************************************************	
	unsigned char drdy = 0x00;
	while(drdy == 0x00) {
		drdy = bReadAk(0x02);
	}
	
	// ***************************************************************
	// Read 6 bytes data from magnetometer to MPU reg 0x49
	// ***************************************************************
	magReadAk();
	// Read from MPU reg0x49 to magData[] array
	unsigned char magData[6];
	mReadMpu(0x49, magData, 6);
	printf("Mag reading: %#x  %#x  %#x  %#x  %#x  %#x  \n",  magData[0],  magData[1],  magData[2],  magData[3],  magData[4],  magData[5]);
	
	return 0;
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
	writeMpu(0x34, 0x80);		// Enable i2c read
	
	unsigned char c = 0x80;
	while(c & 0x80) {				// Wait for enable bit to be cleared
		c = readMpu(0x34);
	}
	
	return readMpu(0x35);
}


int bWriteAk(unsigned char regAk, unsigned char byte) {

	// Use SLV_4 for byte communication over i2c
	writeMpu(0x33, byte);		// Store data to write to AK8963
	writeMpu(0x31, 0x0C);		// Write + device address AK8963
	writeMpu(0x32, regAk);		// AK8963 register to read
	writeMpu(0x34, 0x80);		// Enable i2c write
	
	unsigned char c = 0x80;
	while(c & 0x80) {				// Wait for enable bit to be cleared
		c = readMpu(0x34);
	}
	
	return 1;
}

void magReadAk(void) {

	// Use SLV_4 for byte communication over i2c
	writeMpu(0x25, 0x8C);		// Read + device address AK8963
	writeMpu(0x26, 0x03);		// AK8963 register to read
	writeMpu(0x27, 0x86);		// Enable i2c read 6 bytes to EXT_SENSE_DATA_0
}
