
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
#include <ncurses.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <wiringSerial.h>

#include "cmd.h"
#include "madgwick.h"
#include "cavallo.h"
#include "conversion.h"
#include "gps.h"
#include "rudder.h"
#include "compass.h"


// Globals accessable from all files if declared as "extern"

//  Struct for return values
struct fusionResult {
	double yaw;
	double w;
	double wdot;
};

int counter = 0;		// Global counter for test of thread handling
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;	// Global semafor for thread safty
FILE *fp;		//Global filehandle to make it possible to log in all routines
struct fusionResult madgwickG;	// Global for interthread communication
struct fusionResult cavalloG;		// Global for interthread communication
double gpsCourseG;					// Global for interthread communication
double gpsSpeedG;					// Global for interthread communication


#define SIZE 256


// Function prototypes, code at the end
double elapsed(struct timeval t1, struct timeval t0);
double PIDAreg(int mode, double yawCmd, double yawIs, double w, double wDot);
struct fusionResult simulate(double rudSet, double dt);

	
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
	if((rc1 = pthread_create(&th2, NULL, &compass, NULL))) printf("No thread created\\n");
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
		yawCmd = deg0to360(yawCmd);		// Esure right interval				
				
		//  Call PID regulator 10 times per second independent of mode.
		//  That gives PID opportunity to follow compass in standby
		static double rudderPID = 0;
		gettimeofday(&tReg1, NULL);
		if(elapsed(tReg1, tReg0) > 100.0) {
			tReg0 = tReg1;
			
			//  Get global data for regulator input and print to logfile
			double cY;
			double cW;
			double cWd;
			double mY;
			double mW;
			double mWd;
			pthread_mutex_lock(&mutex1);		// Fetch global data
				cY = cavalloG.yaw;
				cW = cavalloG.w;
				cWd = cavalloG.wdot;
				mY = madgwickG.yaw;
				mW = madgwickG.w;
				mWd = madgwickG.wdot;
			pthread_mutex_unlock(&mutex1);
			// Print global variables to logfile
			fprintf(fp, "%d  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f\n", mode , rudderSet, rudderIs, cY, cW, cWd, mY, mW, mWd, gpsCourse, gpsSpeed);
			//printf("%d  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f\r\n", mode , rudderSet, rudderIs, cY, cW, cWd, mY, mW, mWd, gpsCourse, gpsSpeed);
			
			//  Simulate behaviour according to rudderSet
			struct fusionResult sim;
			sim = simulate(rudderSet, 0.1);
			
			switch(1) {				// Chose sensor algorithm or simulation
				case 1:				// Madgwick
					yawIs = mY;		
					w = mW;
					wDot = mWd;
					break;
				case 2:				// Cavallo
					yawIs = cY;		
					w = cW;
					wDot = cWd;
					break;
				case 3:				// Simulation
					yawIs = sim.yaw;		
					w = sim.w;
					wDot = sim.wdot;
					break;
			}
			
			// Call regulator
			rudderPID = PIDAreg(mode, yawCmd, yawIs, w, wDot);
			printf("%f  %f  %f  %f  %f\r\n", yawCmd, rudderSet, yawIs, w, wDot);
			
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


double PIDAreg(int mode, double yawCmdDeg, double yawIsDeg, double wDeg, double wDotDeg) {

	// All calculations in radians in this function
	double yawCmd = degtorad(yawCmdDeg);
	double yawIs = degtorad(yawIsDeg);
	double w = degtorad(wDeg);
	double wDot = degtorad(wDotDeg);
	
	static double dt = 0.1;	// Time interval for PID reg in seconds
	static double rdmax;
	static double admax;
	rdmax = degtorad(5.0);		// Desired max rate 3 deg into radians
	admax = degtorad(1.0);		// Desired max accelration 1 deg into radians
	static double xsi = 2.0;		// Damper spring and
	static double ws = 0.63;	//  lp filter constants
	static double psid = 0;		//  Desired yaw in rad
	static double rd = 0;			//  Desired angular rate
	static double ad = 0;		//  Desired angular accelration
	
	static double psiTilde = 0;	// Yaw error
	static double rTilde = 0;		// w error
	static double integralPsiTilde = 0;	// Integral of yaw error
	static double rudderMoment = 0;	// Regulator output
	
	// Only run setpoint limitation and PID reg when heading hold (mode == 2)
	if(mode == 2) {
	
		// Reference model for setpoint limitation.(Fossen chapter 10.2.1)
		psid = dt * rd + psid;
	
		rd = dt * ad + rd;
		if(rd >=   rdmax) rd =   rdmax;		//  Rate saturation
		if(rd <= - rdmax) rd = - rdmax;
	
		ad = -dt*(2*xsi+1)*ws*ad -dt*(2*xsi+1)*ws*ws*rd -dt*ws*ws*ws*radpitopi((psid-yawCmd)) +ad;
		if(ad >=   admax) ad =   admax;	//  Accelration saturation
		if(ad <= - admax) ad = - admax;
		
	
		//  PID-regulator with accelration feedback (Fossen capter 12.2.6
		//  Formulas 12.154 and 12.155 and differentiated filtered accelration 12.153)
		static double Knomoto = 0.75;		// Typical for 6 knots
		static double Tnomoto = 3.0;		// Typical for 6 knots
	
		psiTilde = radpitopi(yawIs - psid);		// diff from desired setpoint
	
		integralPsiTilde += psiTilde * dt;	// Integral diff
	
		rTilde = w -rd;		// Diff in angular rate
	
		static double wb = 1.0;
		static double alfa = 25;
		double m = Tnomoto / Knomoto;
		double wn = 1.56 * wb;
		double Km;
		double Kp;
		double Kd;
		double Ki;
		// If true - automatic parameter calculation from wb and alfa
		if(false) {
			Km = alfa / 100 * m;
			Kp = (m + Km) * wn * wn;
			Kd = 2 * 1 * wn *(m + Km) - 1/ Knomoto;
			Ki = wn / 10 * Kp;
		} else {
			Km = 0;
			Kp = 3;
			Kd = 0.5;
			Ki = 0.1;
		}
		
		double tauFF = (m +Km) * (ad + rd / Tnomoto);
	
		rudderMoment = tauFF - Kp * psiTilde - Kd * rTilde -Ki * integralPsiTilde -Km * wDot;
		
		//printf("%f  %f  %f  %f\r\n", yawCmd, psid, yawIsDeg, psiTilde);
		return deg180to180(radtodeg(rudderMoment));
	
	} else {		// If not mode == 2, let parameters follow actual values
		psid = yawIs;
		rd = w;
		ad = 0;
		psiTilde = 0;
		rTilde = 0;
		integralPsiTilde = 0;
		rudderMoment = 0;
		
		return 0;
	}
	
	
	
}


struct fusionResult simulate(double rudSet, double dt) {
	
	static double w = 0;
	static double psi = 0;
	static double wDot = 0;
	static double x = 0;
	static double Kn = 0.75;		// Nomoto constants for simulation model
	static double Tn = 3.0;
	static double Tf = 0.5;		// Numerical differiation filter constant for wDot
	
	w = dt/Tn*(Kn*rudSet - w) + w;		// Nomoto
	psi = w * dt + psi;
	x = dt / Tf *(w - x) + x;				// Numerical differentiation
	wDot = 1 / Tf *(w - x);
	
	struct fusionResult sim;
	sim.yaw = psi;
	sim.w = w;
	sim.wdot = wDot;

	return sim;
}




