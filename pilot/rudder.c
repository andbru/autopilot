
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <linux/spi/spidev.h>
#include<ncurses.h>
#include<pthread.h>

#include "rudder.h"
#include "conversion.h"

void initRudder(void) {
	
	// Initialize ad-converter
	int adcHandle;
	
	pinMode(6, INPUT);
	pullUpDnControl(6, PUD_OFF);
	
	adcHandle = wiringPiSPISetup(0, 1000000);			// Channel 0, speed 1 MHz, get filedescriptor
	printf("Filedescriptor: %d\n", adcHandle);
	
	char mode = SPI_MODE_1;
	int ioc = ioctl(adcHandle, SPI_IOC_WR_MODE, &mode);	// Set mode = 1
	printf("Mode set return should be > -1: %d\n", ioc);
	ioc = ioctl(adcHandle, SPI_IOC_RD_MODE, &mode);			// Check mode = 1
	printf("Mode get return should be = 1: %d\n", mode);
	
	// Write contents of control registers
	unsigned char ctrl[5];
	ctrl[0] = 0x43;		// WREG cmd, write 4 register starting at 0x00
	ctrl[1] = 0x80;
	ctrl[2] = 0xC4;
	ctrl[3] = 0xC0;
	ctrl[4] = 0x00;
	ioc = wiringPiSPIDataRW(0, ctrl, 5);
	printf("Mode set return should be > -1: %d\n", ioc);

	// Send start/sync command 0x08
	unsigned char cmd[2];
	cmd[0] = 0x08;
	cmd[1] = 0x00;
	ioc = wiringPiSPIDataRW(0, cmd, 1);
	printf("Start/sync command, return should be > -1: %d\n", ioc);
	
	// Initialize PWM outputs
	pinMode(25, OUTPUT);				// EA
	pinMode(1, PWM_OUTPUT);		// PA
	pinMode(24, PWM_OUTPUT);		// PB
	pwmSetMode( PWM_MODE_MS);
	pwmSetRange(1024);
	pwmSetClock(32);
}


int pollRudder(double *angle) {

	static int fsRaw = 0x7fffff;		// Full scale digital
	static double fsU = 3.3;			// Full scale volts
	static double fu = 0;				// Filtered voltage
	static double fk = 0.1;				// Filter constant
	
	//  *******************************************************************************************
	//  Conversion to degrees, y angle in deg, x signal in volt. Straight line y=kx+m. degPerVolt = k. 
	//  uZeroDeg = voltage at 0 deg => m = - k * uZeroDeg = - degPerVolt * uZeroDeg
	//  *******************************************************************************************
	static double degPerVolt = 37.42 ;
	//static double uZeroDeg = 2.347;
	static double uZeroDeg = 2.60;
	
	// Check DRDY#
	if (digitalRead(6) == 0) {
		// Read 3 data bytes
		unsigned char rData[4];
		rData[0] = 0x10;		// RDATA cmd
		rData[1] = 0x00;
		rData[2] = 0x00;
		rData[3] = 0x00;
		wiringPiSPIDataRW(0, rData, 4);
		int res = rData[1] * 256 * 256 + rData[2] * 256 + rData[3];
		
		double u = res * fsU / fsRaw;	// Conversion to Volt
		//printf("%f\n", u);
		
		fu = u * fk + (1.0 - fk) * fu; 		// Digital filter
		
		*angle = degPerVolt *fu - degPerVolt * uZeroDeg; 	// Conversion to deg
		//printf("%f  %f  %f  \n", u, fu, *angle);

		return 1;
	}
	
	return 0;
}


void actuateRudder(double rudderSet, double rudderMeas, double *rudderIs) {
	double rudderBound = 10.0;
	// Constants for Florin algorithm
	double db = 0.2;			// Dead band (deg)
	double slow = 1.5;		// Slow speed interval (deg)
	double pFast = 800;		// Max 1024
	double pSlow = 400;
	
	//  Fixed gain observer  ****************************
/*
	static double lastOut = 0;
	{	// Separate block to avoid reuse of variables
		static double dt = 0.01;
		static double k = 0;
		static double T = 0.2;
		static double k11 = 0.1;
		static double x = 0;
		static double y = 0;
		
		if(lastOut == 800) k = 0.00565;
		if(lastOut == 400) k = 0.0045;
		
		x = x + y * dt + k11 * (rudderMeas - x);
		y = y *(1 - dt / T) + dt / T * k * lastOut;
		
		*rudderIs = x;
	}
*/
	//  ***************************************************
	
	int out = 0;
	//struct timeval t;
	
	double dr = rudderSet - *rudderIs;
	dr = deg180to180(dr);	// Ensure right interval
		
	if(dr < -slow) out = - pFast;
	if((-slow < dr) && (dr < -db)) out = - pSlow;
	if((-db < dr) && (dr < db)) out = 0;
	if((db < dr) && (dr < slow)) out = pSlow;
	if( dr > slow) out = pFast;
	
	/*
	// Rudder test
	static int tCount = 300;
	static int pumpMode = 1;
	int tOn = 300;
	int tOff = 300;
	//
	tCount -= 1;
	if(tCount == 0) {
		if(pumpMode == 1) {
			pumpMode = 2;
			tCount = tOff;
			out = 0;
		}  else
		if(pumpMode == 2) {
			pumpMode = 3;
			tCount = tOn;
			out = 800;
		} else
		if(pumpMode == 3) {
			pumpMode = 4;
			tCount = tOff;
			out = 0;
		} else
		if(pumpMode == 4) {
			pumpMode = 1;
			tCount = tOn;
			out = -800;
		} 		
	}
	*/	

	if(out > 0) {
		if(*rudderIs > rudderBound) {
			//printf("Rudder out of bounds (positive)\r\n");
			//return;
		}
		digitalWrite(25, HIGH);
		pwmWrite(24, 0);
		pwmWrite(1, out);
	}
	
	if(out < 0) {
		if(*rudderIs < -rudderBound) {
			//printf("Rudder out of bounds (negative)\r\n");
			//return;
		}	
		digitalWrite(25, HIGH);	
		pwmWrite(1, 0);
		pwmWrite(24, -out);
	}
	
	if(out == 0) {
		digitalWrite(25, HIGH);
		pwmWrite(1, 0);
		pwmWrite(24, 0);	
	}
	
	//pthread_mutex_lock(&mutex1);
	//	fprintf(fp, "%f  %f  %d\n", rudderSet, rudderIs, out);
	//pthread_mutex_unlock(&mutex1);
	
	// Test printout	
	//gettimeofday(&t, NULL);
	//printf("%d   %f  %f  %F  %d\n", t.tv_usec, rudderSet, rudderMeas, *rudderIs, out);

	//printf("%f  %f  %f  %d\n", rudderSet, rudderMeas, *rudderIs, out);
	
	//lastOut = out;
	
	return;
}

