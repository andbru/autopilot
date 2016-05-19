
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
#include<stdlib.h>
#include<string.h>
#include <wiringSerial.h>

#include "global.h"
#include "madgwick.h"
#include "cavallo.h"


#define SIZE 256


// Function prototypes, code at the end
double elapsed(struct timeval t1, struct timeval t0);
void initRudder(void);
int pollRudder(double *angel);
void actuateRudder(double rudderSet, double rudderIs);
void initKnob(void);
int pollKnob(int *mode, double *knobIncDec);
void initCmd(void);
int pollCmd(int *mode, double *cmdIncDec);
double PIDAreg(int mode, double yawCmd, double yawIs, double w, double wDot);
void *th2func();
int initGps(void);
bool pollGps( double *speed, double *course);
bool nmeaOk( double *speed, double *course); 

// Globals
int counter = 0;		// Global counter for test of thread handling
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;	// Global semafor for thread safty
FILE *fp;		//Global filehandle to make it possible to log in all routines
struct taitBryanYPR madgwickG;	// Global for interthread communication
struct taitBryanYPR watsonG;		// Global for interthread communication
struct taitBryanYPR cavalloG;		// Global for interthread communication
double gpsCourseG;					// Global for interthread communication
double gpsSpeedG;					// Global for interthread communication
int hGps;		// Handle to serial gps port
char nmea[100] = "";	// Buffer for nmea-sentences
	
int main() {
	
	int rc1;
	pthread_t th2;
	
	char fname[SIZE];
	time_t curtime;
	struct tm *loctime;
	
	//*****************************************************************************
	//  All angels are expressed in degrees between 0 and 359.999.. in all 
	//  routines except within the sensor fusion algorithms where radians are used. 
	//  All differences between angels are immediatly converted to +/- 180 degrees.
	//*****************************************************************************
	double rudderIs = 0;
	struct timeval t0, t1, tRud0, tRud1;
	struct timeval tReg0, tReg1;
	int mode = 0;
	double yawCmd = 0;
	double yawIs = 0;
	double rudderSet = 0;
	double w = 0;
	double wDot = 0;
	double cmdIncDec = 0;
	double knobIncDec = 0;
	int redLed = 27;
	int greenLed = 5;
	
	wiringPiSetup();

	// Open log file with date and time as filename
	curtime = time(NULL);
	loctime = localtime(&curtime);
	strftime(fname, SIZE, "/home/andbru/autopilot/logs/%F_%T", loctime);
	printf("Logfile = %s\r\n", fname);
	fp  = fopen(fname, "w");
	
	// Start the second thread with gyro compass	
	if((rc1 = pthread_create(&th2, NULL, &th2func, NULL))) printf("No thread created\\n");
	sleep(2);		// Wait for thread to initiate
	
	// Initiate hardware
	initRudder();
	initKnob();
	initGps();
	initCmd();
	
	// mode = 0 startup, = 1 standby, = 2 heading hold, = 7 rudder control
	mode = 1;
	gettimeofday(&tRud0, NULL);
	gettimeofday(&tReg0, NULL);

	
	// Endless main loop
	for (;;) {
		
		//  Poll rudder angel every milli second and filter
		int newAngel = pollRudder(&rudderIs);
		if(newAngel) {			
			gettimeofday(&t1, NULL);		// dt = time between iterations
			double dt = elapsed(t1, t0);
			dt = dt;		// Scilence compiler warnings
			t0 = t1;
		}
		
		//  Poll command knob
		int newMode = pollKnob(&mode, &knobIncDec);
		newMode = newMode;		// Just to scilence compiler warnings
		
		// Poll cmd
		if(pollCmd(&mode, &cmdIncDec) < 0) {
			fclose(fp);
			return -1;
		}
		
		// Poll gps
		double gpsCourse;
		double gpsSpeed;
		bool newGps = pollGps(&gpsCourse, &gpsSpeed);
		if(newGps) {
			//printf("COG = %f   SOG = %f\r\n", gpsCourse, gpsSpeed);
			pthread_mutex_lock(&mutex1);		// Update global variables in a threadsafe way
				gpsCourseG = gpsCourse;
				gpsSpeedG = gpsSpeed;
			pthread_mutex_unlock(&mutex1);
		}
		
		
		// Chose source for PID-regulator input
		switch (mode) {
			case 0:
				break;	
			case 1:
				yawCmd = yawIs;
				break;	
			case 2:
				if(knobIncDec != 0) {
					yawCmd += knobIncDec;
					knobIncDec = 0;
				}
				if(cmdIncDec != 0) {
					yawCmd += cmdIncDec;
					cmdIncDec = 0;
				}
				break;	
			case 7:
				yawCmd = yawIs;
				break;	
		}				
				
		//  Call PID regulator 10 times per second independent of mode.
		//  That gives PID opportunity to follow compass in standby
		double rudderPID = 0;
		gettimeofday(&tReg1, NULL);
		if(elapsed(tReg1, tReg0) > 100.0) {
			tReg0 = tReg1;
			
			//  Get global data for regulator input and rint to logfile
			pthread_mutex_lock(&mutex1);		// Print global variables to logfile
				fprintf(fp, "%d  %f  %f  %f  %f  %f  %f  %f\n", mode , rudderSet, rudderIs, cavalloG.yaw, cavalloG.roll, madgwickG.yaw, gpsCourse, gpsSpeed);
			pthread_mutex_unlock(&mutex1);
			
			rudderPID = PIDAreg(mode, yawCmd, yawIs, w, wDot);
			
			//  Finalize the line in logfile wit rudderPID
			
			
			if(mode == 0) { digitalWrite(greenLed, LOW); digitalWrite(redLed, LOW);}	//Light up the Led's
			if(mode == 1) { digitalWrite(greenLed, HIGH); digitalWrite(redLed, LOW);}
			if(mode == 2) { digitalWrite(greenLed, LOW); digitalWrite(redLed, HIGH);}
			if(mode == 7) { digitalWrite(greenLed, HIGH); digitalWrite(redLed, HIGH);}
			
		}
		
		// Chose source for rudder actuator input
		switch (mode) {
			case 0:
				break;	
			case 1:
				rudderSet = rudderIs;
				break;	
			case 2:
				rudderSet = rudderPID;
				break;					
			case 7:
				if(knobIncDec != 0) {
					rudderSet += knobIncDec;
					knobIncDec = 0;
				}
				if(cmdIncDec != 0) {
					rudderSet += cmdIncDec;
					cmdIncDec = 0;
				}
				break;	
		}
			
		//  Call rudder actuator 100 times per second if mode == 2 or 7
		if((mode == 2) || (mode == 7)) {
			gettimeofday(&tRud1, NULL);
			if(elapsed(tRud1, tRud0) > 10.0) {
				tRud0 = tRud1;
				actuateRudder(rudderSet, rudderIs);		
			}
		} else {
			digitalWrite(25, LOW);		// Stop all rudder activity
			pwmWrite(1, 0);
			pwmWrite(24, 0);	
		}
	}

	return 0;
}

// Functions

double elapsed(struct timeval t1, struct timeval t0){

	double elapsed = (t1.tv_sec - t0.tv_sec) * 1000;		// milliseconds
	elapsed += (t1.tv_usec - t0.tv_usec) / 1000.0;

	return elapsed;
}


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


int pollRudder(double *angel) {

	static int fsRaw = 0x7fffff;		// Full scale digital
	static double fsU = 3.3;			// Full scale volts
	static double fu = 0;				// Filtered voltage
	static double fk = 0.1;			// Filter constant
	
	//  *******************************************************************************************
	//  Conversion to degrees, y angel in deg, x signal in volt. Straight line y=kx+m. degPerVolt = k. 
	//  uZeroDeg = voltage at 0 deg => m = - k * uZeroDeg = - degPerVolt * uZeroDeg
	//  *******************************************************************************************
	static double degPerVolt = 36.49 ;
	static double uZeroDeg = 1.866;
	
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
		
		fu = u * fk + (1.0 - fk) * fu; 		// Digital filter
		
		*angel = degPerVolt *fu - degPerVolt * uZeroDeg; 	// Conversion to deg

		return 1;
	}
	
	return 0;
}


void actuateRudder(double rudderSet, double rudderIs) {
	double rudderBound = 10.0;
	// Constants for Florin algorithm
	double db = 0.3;			// Dead band (deg)
	double slow = 0.7;		// Slow speed interval (deg)
	double pFast = 800;		// Max 1024
	double pSlow = 400;
	
	if(rudderIs < -rudderBound) {
		printf("Rudder out of bounds (negative)\r\n");
		return;
	}		
	if(rudderIs > rudderBound) {
		printf("Rudder out of bounds (positive)\r\n");
		return;
	}
	
	int out = 0;
	double dr = rudderSet - rudderIs;
		
	if(dr < -slow) out = - pFast;
	if((-slow < dr) && (dr < -db)) out = - pSlow;
	if((-db < dr) && (dr < db)) out = 0;
	if((db < dr) && (dr < slow)) out = pSlow;
	if( dr > slow) out = pFast;

/*
	static int rc = 0;

	if(rc <= 200) out = 0;
	if(rc > 200) out = 800;
	if(rc > 250) out = 0;
	if(rc > 400) out = -800;
	if(rc > 450) out = 0;
	if(rc > 700) out = 400;
	if(rc > 750) out = 0;
	if(rc > 1000) out = -400;
	if(rc > 1050) out = 0;
	if(rc > 1500) out = 200;
	if(rc > 1600) out = 0;
	if(rc > 2000) out = -200;
	if(rc > 2100) out = 0;
	if(rc > 2500) out = 100;
	if(rc > 2800) out = 0;
	if(rc > 3000) out = -50;
	if(rc > 3400) out = 0;
	if(rc > 3500) out = -10;
	if(rc > 3900) out = 0;
	if(rc > 4000) out = -5;
	if(rc > 4500) out = 0;

	rc++;
*/	
	
	printf("%f   %f    %d\r\n", rudderSet, rudderIs, out);
	
	if(out > 0) {
		digitalWrite(25, HIGH);
		pwmWrite(24, 0);
		pwmWrite(1, out);
	}
	if(out < 0) {
		digitalWrite(25, HIGH);	
		pwmWrite(1, 0);
		pwmWrite(24, -out);
	}
	if(out == 0) {
		digitalWrite(25, HIGH);
		pwmWrite(1, 0);
		pwmWrite(24, 0);	
	}
	
	pthread_mutex_lock(&mutex1);
	//	fprintf(fp, "%f  %f  %d\n", rudderSet, rudderIs, out);
	pthread_mutex_unlock(&mutex1);
	
	return;
}

void initKnob(void) {

	pinMode(0, INPUT);		// Knob
	pinMode(2, INPUT);
	pinMode(3, INPUT);
	pullUpDnControl(0, PUD_UP);
	pullUpDnControl(2, PUD_UP);
	pullUpDnControl(3, PUD_UP);
	
	pinMode(5, OUTPUT);	// LED
	pinMode(27, OUTPUT);
	digitalWrite(5, LOW);
	digitalWrite(27, LOW);

}


int pollKnob(int *mode, double *knobIncDec) {
	
	static int clk0 = 1;
	static int clk1 = 1;
	static int dt1 = 1;
	static int sw = 1;
	static int swStart = 1000;		// no of consecutive low to avoid switch bumps
	static int swCount = 1000;
	static double degPerClick = 0.5;	// Gearing
	
	if(*mode == 0) return 0;		// Still starting up
	
	dt1 = digitalRead(2);
	clk1 = digitalRead(3);
	sw = digitalRead(0);
	
	int upDown = 0;
	if(clk1 < clk0) {		// falling
		if(dt1 == 1) upDown++; else upDown--;
	}
	if(clk1 > clk0) {		// raising
		if(dt1 == 1) upDown--; else upDown++;
	}
	clk0 = clk1;
	
	//Calculate Increment/Decrement
	if((*mode == 2) || (*mode == 7)) {
		*knobIncDec += upDown * degPerClick;
	} 
	
	if(sw == 0) swCount--; else 	swCount = swStart;	
	if(swCount == 0) {
	
		if(*mode == 1) *mode = 2; 	
		else					
		if((*mode == 2) || (*mode == 7)) *mode = 1;	// Toggle mode
		return 1;
	}
	
	return 0;
}


void initCmd(void) {

	initscr();
	//WINDOW *wp = initscr();
	cbreak();
	//nodelay(wp, TRUE);
	nodelay(stdscr, TRUE);
	keypad(stdscr, TRUE);
}


int pollCmd(int *mode, double *cmdIncDec) {
	double gain = 0.5;	// Gearing deg per click
	
	int cht;
	if((cht = getch()) == ERR) {		
										
		// Do nothing
		return 0;
	}
	else {
		switch (cht) {	
			case KEY_RIGHT:	// Increment					
				*cmdIncDec += gain;	
				return 1;	
			case KEY_LEFT:		// Decrement
				*cmdIncDec += -gain;
				return 1;
			case KEY_UP:		// Increment 5				
				*cmdIncDec += gain * 5;	
				return 1;	
			case KEY_DOWN:	// Decrement 5
				*cmdIncDec += -gain * 5;
				return 1;
			case 'q':		// quit
				endwin();
				digitalWrite(5, LOW);
				digitalWrite(27, LOW);
				return -1;	
			case 'r':		// rudder control
				*mode = 7;
				return  1;	
			case 's':		// standby  
				*mode = 1;
				return  1;	
			case 'h':		// heading hold 
				*mode = 2;
				return  1;					
		}
		return 1;
	}
}


double PIDAreg(int mode, double yawCmd, double yawIs, double w, double wDot) {
	
	// Reference model for setpoint limitation.(Fossen chapter 10.2.1)
	static double dt = 0.1;	// Time interval for PID reg in seconds
	static double rdmax = 3.0 *3.14 /180;		// Desired max rate 3 deg into radians
	static double admax = 1.0 *3.14 /180;	// Desired max accelration 1 deg into radians
	static double xsi = 2.0;		// Damper spring and
	static double ws = 0.63;	//    lp filter constants
	static double psid = 0;		//  Desired yaw in rad
	static double rd = 0;			//  Desired angular rate
	static double ad = 0;		//  Desired angular accelration
	
	psid = dt * rd + psid;
	
	rd = dt * ad + rd;
	if(rd >=   rdmax) rd =   rdmax;		//  Rate saturation
	if(rd <= - rdmax) rd = - rdmax;
	
	ad = -dt*(2*xsi+1)*ws*ad -dt*(2*xsi+1)*ws*ws*rd -dt*ws*ws*ws*(psid-yawCmd*3.14/180) +ad;
	if(ad >=   admax) ad =   admax;	//  Accelration saturation
	if(ad <= - admax) ad = - admax;
		
	
	//  PID-regulator with accelration feedback (Fossen capter 12.2.6
	//  Formulas 12.154 and 12.155 and differentiated filtered accelration 12.153)
	static double Knomoto = 0.34;		// Typical for 6 knots
	static double Tnomoto = 0.;		// Independaent of speed
	static double integralPsiTilde = 0;
	double psiTilde = yawIs - psid;		// diff from desired setpoint
	if(psiTilde < -3.14) psiTilde += 2 * 3.14;	// Ensure -pi < angel <pi
	if(psiTilde > 3.14) psiTilde -= 2 * 3.14;
	
	integralPsiTilde += psiTilde * dt;	// Integral diff
	
	double rTilde = w -rd;		// Diff in angular rate
	
	static double wb = 1;
	static double alfa = 0;
	double m = Tnomoto / Knomoto;
	double wn = 1.56 * wb;
	double Km;
	double Kp;
	double Kd;
	double Ki;
	// If true - automatic parameter calculation from wb and alfa
	if(true) {
		Km = alfa / 100 * m;
		Kp = (m + Km) * wn * wn;
		Kd = 2 * 1 * wn *(m + Km) - m / Tnomoto;
		Ki = wn * 10 * Kp;
	} else {
		Km = 0;
		Kp = 3;
		Kd = 0.5;
		Ki = 0.1;
	}
		
	double tauFF = (m +Km) * (ad + rd / Tnomoto);
	
	double rudderMoment = tauFF - Kp * psiTilde - Kd * rTilde -Ki * integralPsiTilde -Km * wDot;
	
	return rudderMoment / 3.14 * 180;
}


void *th2func() {

	struct taitBryanYPR madgwick;
	struct taitBryanYPR watson;
	struct taitBryanYPR cavallo;
	
	initMPU9250();
	
	int regValue;
	int magCount = 0;
	int accGyroCount = 0;
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
			accGyroCount++;
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
			magCount++;
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

		}
		
		// Transfer values to global variables thred safe and get gps values
		pthread_mutex_lock(&mutex1);
			counter++;
			cavalloG = cavallo;
			madgwickG = madgwick;
			watsonG = watson;
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


int initGps() {
	int handle = serialOpen("/dev/ttyAMA0", 9600);
	if(handle<0) return -1;
	else {
	
		hGps = handle;
		serialFlush(hGps);
		
		serialPrintf(hGps, "\r\n");
		//serialPrintf(hGps, "%s\r\n", "$PMTK104*37");		// Full Cold Start
		//serialPrintf(hGps, "%s\r\n", "$PMTK414*33");		// Querry sentences to send
		//serialPrintf(hGps, "%s\r\n", "$PMTK314,-1*04");	// Back to standard sentences
		//serialPrintf(hGps, "%s\r\n", "$PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
		//serialPrintf(hGps, "%s\r\n\r\n", "$PMTK313,1*2E");		// EGNOS enable
		//serialPrintf(hGps, "%s\r\n", "$PMTK501,2*28");		// EGNOS enable DGPS mode
		serialPrintf(hGps, "%s\r\n", "$PMTK220,100*2F");
		
		printf("Init GPS ready!\n");
		
		return handle;
	}
}


bool pollGps(double *course, double *speed) {
	
	char c;
	int noChar = serialDataAvail(hGps);
	if(noChar>=1) {

	int gpsCount;
		for(gpsCount=1; gpsCount<=noChar; gpsCount++) {
			c = serialGetchar(hGps);	// Read character
			//printf("%c", c);
	
			int len = strlen(nmea);
			nmea[len] = c;					// Append to string
			nmea[len+1] = '\0';				//  Add NULL terminator

			if(c == '\n') {						// If end of line
		
				// Action for new line
				bool dataOk = nmeaOk(course, speed);	
				nmea[0] = '\0';				// Reset buffer to zero
				if(dataOk) return true;
			}
		}
	}	
	return false;
}


bool nmeaOk( double *course, double *speed) {

	int lenNmea = strlen(nmea);
	unsigned int chSum = 0;					// Calculate checksum before corrupting string
	int iSum = 0;
	for(iSum = 1; iSum < lenNmea - 5; iSum++) {
		chSum = chSum ^ nmea[iSum];
	}
	
	//printf("%s\r\n", nmea);
	
	const char s[2] = ",";
	char *param;
	param = strtok(nmea, s);					// Find first parameter
	
	if(strcmp(param, "") == 0) return false;	// Empty string
	if(strcmp(param, "$GPVTG") == 0) {		// Ceck for "Track made good and speed
		
		param = strtok(NULL, s);				// Find second parameter, true course
		float fCourse;
		int i = sscanf(param, "%f", &fCourse);	// Cast to float
		if(i != 1) return false;						// Cast failed
		*course = fCourse;						// Cast to double
		
		param = strtok(NULL, s);				// Find third parameter, "T" as in true
		if(strcmp(param, "T") == 0) {
			
			// Field #4 for magnetic course empty and not recognized

			param = strtok(NULL, s);			// Check for "M" in field #5
			if(strcmp(param, "M") == 0) {
			
				param = strtok(NULL, s);				// Field #6, speed SOG
				float fSpeed;
				int i = sscanf(param, "%f", &fSpeed);	// Cast to float
				if(i != 1) return false;						// Cast failed
				*speed = fSpeed;							// Cast to double
			
				param = strtok(NULL, s);			// Check for "N" in field #7
				if(strcmp(param, "N") == 0) {
				
					param = strtok(NULL, s);		// Skip field #8, speed in km/h
					param = strtok(NULL, s);		// Check for "K" in field #9
					if(strcmp(param, "K") == 0) {
					
						param = strtok(NULL, s);		// Read last four characters
						
						char str1[2];						// Check for A = gpsFix
						str1[0] = param[0];
						str1[1] = '\0';
						char str2[2] = "A";
						if(strcmp(str1, str2) != 0) return false;														
											
						str1[0] = param[1];				// Check for * before checksum
						str1[1] = '\0';
						str2[0] = '*';
						str2[1] = '\0';
						if(strcmp(str1, str2) != 0) return false;
						
						char sumStr[3];					// Check check sum vs
						sumStr[0] = param[2];			// calculated sum first in this function
						sumStr[1] = param[3];
						sumStr[2] = '\0';
						int readSumDec = 0;
						sscanf(sumStr, "%x", &readSumDec);
						if(readSumDec != chSum) return false;

						param = strtok(NULL, s);		// Check that there are no more fields
						if(param == NULL) {
							return true;
						}
					}
				}
			}
		}
	}
	return false;
}
