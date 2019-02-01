
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
#include "server.h"
#include "deviation.h"
#include "EM7180.h"


// Globals accessable from all files if declared as "extern"

//  Struct for return values
struct fusionResult {
	double yaw;
	double w;
	double wdot;
};

int counter = 0;		// Global counter for test of thread handling
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;	// Global semafor for thread safty
pthread_mutex_t mutexTcp = PTHREAD_MUTEX_INITIALIZER;	// Global semafor for thread safty
FILE *fp;		//Global filehandle to make it possible to log in all routines
struct fusionResult madgwickG;	// Global for interthread communication
struct fusionResult cavalloG;		// Global for interthread communication
double gpsCourseG = 0;					// Global for interthread communication
double gpsSpeedG = 0;					// Global for interthread communication

char data[100] = "";					// Global for datatransfer over tcp
char *dataP = data;
char cmd[50] = "";					
char *cmdP = cmd;

double Kp = 0.9;						// Global regulator parameters
double Kd = 1.9;
double Ki = 0.09;
double Km = 0.0;

int accGyroCount = 0;				// Global for sensor reading freq.
int magCount = 0;

int loopCount = 0;

#define SIZE 256


// Function prototypes, code at the end
double elapsed(struct timeval t1, struct timeval t0);
double PIDAreg(int mode, double yawCmd, double yawIs, double w, double wDot, double gpsSpeedLp);
struct fusionResult simulate(double rudSet, double dt);
void powerOff();

	
int main() {
	
	int countMain = 0;	// Counter for mainloop
	int rc2;			// Handle and struct for new threads
	pthread_t th2;
	int rc3;
	pthread_t th3;
	
	char fname[SIZE];
	time_t curtime;
	struct tm *loctime;
	
	// *****************************************************************************
	//  All angles are expressed in degrees between 0 and 359.999.. in all 
	//  routines except within the sensor fusion algorithms where radians are used. 
	//  All differences between angles are immediatly converted to +/- 180 degrees.
	// *****************************************************************************
	double rudderIs = 0;
	double rudderMeasured = 0;
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
	double tcpIncDec = 0;
	int redLed = 27;
	int greenLed = 5;
	
	double gpsCourse = 0;
	double gpsSpeed = 0;
	static double gpsSpeedLp = 0;
	

	clock_t last, now;
	last = clock();
	
	wiringPiSetup();

	// Open log file with date and time as filename
	curtime = time(NULL);
	loctime = localtime(&curtime); 
	strftime(fname, SIZE, "/media/usb/logs/%F_%T.txt", loctime);
	fname[29] ='-';		// raspbian stretch doesnt accept : in filenames
	fname[32] ='-';
	printf("Logfile = %s\r\n", fname);
	fp  = fopen(fname, "w");
	// Print headline to logfile
	static double timestamp = 0;
	fprintf(fp, "time mode rudderSet gpsSpeedLp rudderIs gpsCourse gpsSpeed ");
	fprintf(fp, "cY cW cWd mY mW mWd accGyroCount magCount ");
	fprintf(fp, "yawCmd psid psiTilde rTilde integralPsiTilde rudderMoment tauFF Kp Kd Ki\n");

	
	// Start the second thread with gyro compass	
	if((rc2 = pthread_create(&th2, NULL, &compass, NULL))) printf("No thread #2 created\\n");
	
	// Start the third thread with tcp server for user communication	
	if((rc3 = pthread_create(&th3, NULL, &server, NULL))) printf("No thread #3 created\\n");
	sleep(2);		// Wait for thread to initiate
	
	// Initiate hardware
	initRudder();
	initKnob();
	int gpsPresent = initGps();
	//printf("%d\n", gpsPresent);
	initDev();
	//initCmd();		// Uncomment if controlled from keyboard or over SSH

	//unsigned long ts;
	initEM7180();
	
	// mode = 0 startup, = 1 standby, = 2 heading hold, = 7 rudder control
	mode = 1;
	gettimeofday(&tRud0, NULL);
	gettimeofday(&tReg0, NULL);

	
	// Endless main loop
	for (;;) {
		
		//  Poll rudder angle every milli second
		int newAngle = pollRudder(&rudderMeasured);
		if(newAngle) {
			rudderIs = rudderMeasured;		// No fixed gain observer
			
			gettimeofday(&t1, NULL);		// dt = time between iterations
			double dt = elapsed(t1, t0);
			dt = dt;		// Scilence compiler warnings
			t0 = t1;
		}
		
		//  Poll command knob
		int newMode = pollKnob(&mode, &knobIncDec);
		newMode = newMode;		// Just to scilence compiler warnings
		
		// Poll cmd over ssh
		/* if(pollCmd(&mode, &cmdIncDec) < 0) {	// Uncomment if controlled from keyboard or over SSH
			fclose(fp);
			return -1;
		}	*/

		// Poll gps and filter
		if(gpsPresent) {
			bool newGps = pollGps(&gpsCourse, &gpsSpeed);
			loopCount++;
			if(newGps) {
				//printf("COG = %f   SOG = %f\r\n", gpsCourse, gpsSpeed);
				//printf("%d \n", loopCount);
				loopCount = 0;
				
				double kGps = 0.02;		// Lp filter the speed signal
				gpsSpeedLp = gpsSpeed * kGps + (1 - kGps) * gpsSpeedLp;

				//printf("%f  %f \n", gpsSpeed, gpsSpeedLp);

				pthread_mutex_lock(&mutex1);		// Update global variables in a threadsafe way
					gpsCourseG = gpsCourse;
					gpsSpeedG = gpsSpeed;
				pthread_mutex_unlock(&mutex1);

				now = clock();			//  Time between gps updates
				double tGps = (double)(now - last) / CLOCKS_PER_SEC * 1000;
				tGps = tGps;
				//printf("%f \n", tGps);
				last = now;
			}
		}

		// Update PID-regulator input from tcp server
		switch (mode) {
			case 0:
				break;	
			case 1:
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
				if(tcpIncDec != 0) {
					yawCmd += tcpIncDec;
					tcpIncDec = 0;
				}
				break;	
			case 7:
				// Temporary use rudder control mode (=7) for restart of logging
				
				mode = 1;
				powerOff();

		        	// Open log file with date and time as filename
		        	curtime = time(NULL);
		        	loctime = localtime(&curtime);
		        	strftime(fname, SIZE, "/media/usb/logs/%F_%T.txt", loctime);
				fname[29] ='-';		// raspbian stretch doesnt accept : in filenames
				fname[32] ='-';
				fp  = fopen(fname, "w");
		        	// Print headline to logfile
				timestamp = 0;
		        	fprintf(fp, "time mode rudderSet gpsSpeedLp rudderIs gpsCourse gpsSpeed ");
		        	fprintf(fp, "cY cW cWd mY mW mWd accGyroCount magCount ");
		        	fprintf(fp, "yawCmd psid psiTilde rTilde integralPsiTilde rudderMoment tauFF Kp Kd Ki\n");

				break;	
		}
		yawCmd = deg0to360(yawCmd);		// Ensure right interval
		
		countMain++;				
				
		//  Call PID regulator 20 times per second independent of mode.
		//  That gives PID opportunity to follow compass in standby
		static double rudderPID = 0;
		gettimeofday(&tReg1, NULL);
		if(elapsed(tReg1, tReg0) > 50.0) {
			//printf("%f\n",elapsed(tReg1, tReg0));
			tReg0 = tReg1;
			
			//printf("%d \n", countMain);
			countMain = 0;

			//  Get global data for regulator input and print to logfile
			double cY;
			double cW;
			double cWd;
			double mY;
			double mW;
			double mWd;
			pthread_mutex_lock(&mutex1);		// Get global data
				cY = cavalloG.yaw;
				cW = cavalloG.w;
				cWd = cavalloG.wdot;
				//mY = madgwickG.yaw;
				//mW = madgwickG.w;
				//mWd = madgwickG.wdot;
			pthread_mutex_unlock(&mutex1);
			//printf("%.1f   %.1f       %.1f   %.1f\n", mY, cY, mW, cW);
			
			// Measure time interval
			//struct timeval tStamp;
			//gettimeofday(&tStamp, NULL);
			//printf("%ld \n", tStamp.tv_usec/1000);
			
	
			//unsigned long tEM7180;
			float cEM7180;
			float rEM7180;
			int pollRet = pollEM7180(&cEM7180, &rEM7180);
			if(pollRet != 0) {	
				mY = cEM7180;
				mW = rEM7180;
			} 
			//ts = ts;

			//printf("%.1f   ", gpsCourse);
			//printf("%.1f   %.1f       %.1f   %.1f\n", mY, cY, mW, cW);

			// Remove when EM7180 test is over
			// Numerical differentiation to get wdot
			static double Tw = 0.25;
			static double xw = 0;
			xw = 0.05 / Tw * (-xw + mW) + xw;
			mWd = 1 / Tw * (-xw + mW);
			
			
			printf("%.1f   %.1f       %.1f   %.1f\n", mY, cY, mW, cW);
						
			//  Simulate behaviour according to rudderSet
			struct fusionResult sim;
			sim = simulate(rudderSet, 0.1);
			
			switch(1) {				// Chose sensor algorithm or simulation
				case 1:				// Madgwick
					yawIs = deviation(mY);		
					w = mW;
					wDot = mWd;
					break;
				case 2:				// Cavallo
					yawIs = deviation(cY);		
					w = cW;
					wDot = cWd;
					break;
				case 3:				// Simulation
					yawIs = sim.yaw;		
					w = sim.w;
					wDot = sim.wdot;
					break;
			}
			
			// Let yawCmd follow bearing if not heading hold
			if(mode != 2) yawCmd = yawIs;
			
			// Print global variables to log file
			fprintf(fp, "%f %d %f %f %f %f %f ", timestamp, mode , rudderSet, gpsSpeedLp, rudderIs, gpsCourse, gpsSpeed);
			fprintf(fp, "%f  %f  %f  %f  %f  %f %d %d ", cY, cW, cWd, mY, mW, mWd, accGyroCount, magCount);
			timestamp += .05;
			
			// Call regulator
			rudderPID = PIDAreg(mode, yawCmd, yawIs, w, wDot, gpsSpeedLp);
			//printf("%f  %f  \n", gpsSpeed, gpsSpeedLp);
			
			pthread_mutex_lock(&mutexTcp);		// read/write to globals thread safe
				// Check for command over tcp			
				if(strcmpNS(cmdP, "") != 0) {
					char *tokP = strtok(cmdP, ",");
					double dMode = atoi(tokP);
					if(dMode != 0) mode = dMode;			// No change in mode is indicated by the value zero
					tokP = strtok(NULL, ",");
					tcpIncDec = atof(tokP);
					tokP = strtok(NULL, ",");
					double dKp = atof(tokP);
					Kp += dKp;
					tokP = strtok(NULL, ",");
					double dKd = atof(tokP);
					Kd += dKd;
					tokP = strtok(NULL, ",");
					double dKi = atof(tokP);
					Ki += dKi;
					tokP = strtok(NULL, ",");
					double dKm = atof(tokP);
					Km += dKm;					
					//printf ("%f  %f  %f  %f  %f\r\n",  tcpIncDec, dKp, dKd, dKi, dKm);
					strcpy(cmdP,  "");
									}
				// Update data for tcp transfer
				sprintf(dataP, "%1.0d %06.2f %06.2f %06.2f %3.1f %3.1f %4.2f %3.1f %06.2f %06.2f %3.1f %1.0d \r\n", mode, yawCmd, yawIs, rudderIs, Kp, Kd, Ki, Km, gpsSpeed, gpsCourse, w, magCount);
				//sprintf(dataP, "%1.0d %06.2f %06.2f %06.2f %3.1f %3.1f %4.2f %3.1f %06.2f %06.2f %1.0d %1.0d ", mode, deviation(mY), yawIs,  rudderPID, Kp, Kd, Ki, Km, mY, gpsCourse, accGyroCount, magCount);
				accGyroCount = 0;
				magCount = 0;

			pthread_mutex_unlock(&mutexTcp);
	
			if(mode == 0) { digitalWrite(greenLed, LOW); digitalWrite(redLed, LOW);}	//Light up the Led's
			if(mode == 1) { digitalWrite(greenLed, HIGH); digitalWrite(redLed, LOW);}
			if(mode == 2) { digitalWrite(greenLed, HIGH); digitalWrite(redLed, HIGH);}
			if(mode == 7) { digitalWrite(greenLed, LOW); digitalWrite(redLed, HIGH);}
			
		}
		
		// Chose source for rudder actuator input
		switch (mode) {
			case 0:
				break;	
			case 1:
				//rudderSet = rudderIs;
				rudderSet = 0;			//  Safer ??
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
				if(tcpIncDec != 0) {
					rudderSet += tcpIncDec;
					tcpIncDec = 0;
				}
				break;	
		}
			
		//  Call rudder actuator 100 times per second if mode == 2 or 7
		if((mode == 2) || (mode == 7)) {
			gettimeofday(&tRud1, NULL);
			if(elapsed(tRud1, tRud0) > 10.0) {
				tRud0 = tRud1;
				actuateRudder(rudderSet, rudderMeasured, &rudderIs);		
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


double PIDAreg(int mode, double yawCmd, double yawIs, double w, double wDot, double gpsSpeedLp) {

	// All calculations in degrees in this function
	static double dt = 0.05;	// Time interval for PID reg in seconds
	static double rdmax =3.0;
	static double admax = 1.5;
	static double xsi = 1.0;		// Damper spring and
	static double ws = 1.0;		//  lp filter constants
	static double psid = 0;		//  Desired yaw in rad
	static double rd = 0;			//  Desired angular rate
	static double ad = 0;		//  Desired angular accelration
	
	static double psiTilde = 0;	// Yaw error
	static double rTilde = 0;		// w error
	static double integralPsiTilde = 0;	// Integral of yaw error
	static double rudderMoment = 0;	// Regulator output
	static double tauFF = 0;		// Feed forward of commands

	static double Kpp = 0;
	static double Kdp = 0;
	static double Kip = 0;
	
	static int lastMode = 0;

	
	// Only run setpoint limitation and PID reg when heading hold (mode == 2)
	if(mode == 2) {
	
		// Reference model for setpoint limitation.(Fossen chapter 10.2.1)
		if(lastMode != 2) {		//  Update psid in first iteration
			psid = yawCmd;
			lastMode = mode;
		}
		psid = dt * rd + psid;
		psid = deg0to360(psid);
		//psid = yawCmd;		//  No setpoint limitation
	
		rd = dt * ad + rd;
		if(rd >=   rdmax) rd =   rdmax;		//  Rate saturation
		if(rd <= - rdmax) rd = - rdmax;

		ad = -dt*(2*xsi+1)*ws*ad -dt*(2*xsi+1)*ws*ws*rd -dt*ws*ws*ws*deg180to180((psid-yawCmd)) +ad;
		if(ad >=   admax) ad =   admax;	//  Accelration saturation
		if(ad <= - admax) ad = - admax;
		//printf("%f  %f  %f\n", psid, rd, ad);
	
		//  PID-regulator with accelration feedback (Fossen capter 12.2.6
		//  Formulas 12.154 and 12.155 and differentiated filtered accelration 12.153)
		double Knomoto = 0.874;		// Average
		double Tnomoto = 107 * pow(gpsSpeedLp, -1.8);		// Function of speed
		if (gpsSpeedLp < 3.0) Tnomoto = 107*pow(3.0, -1.8);	// Limit when low speed
		
		//		Adjust regulator parameters for speed
		double p7 = Kp / pow(7, -1);
		double d7 = Kd / pow(7, -0.801);
		double i7  = (Ki - 0.18) / 7;
		Kpp = p7 * pow(gpsSpeedLp, -1);
		Kdp = d7 * pow(gpsSpeedLp, -0.801);
		Kip  = i7 * gpsSpeedLp + 0.18;
		if(gpsSpeedLp < 3.0) {		//Avoid strange control signals at low speed
			Kpp = p7 * pow(3.0, -1);
			Kdp = d7 * pow(3.0, -0.801);
			Kip = i7 * 3.0 + 0.18;
		}
		Kpp=Kp;		//  Temporarily use inputparameters
		Kdp=Kd;
		Kip=Ki;
		
		//		 Regulator errors
		psiTilde = deg180to180(yawIs - psid);		// diff from desired setpoint
		integralPsiTilde += psiTilde * dt;	// Integral diff
		rTilde = w -rd;		// Diff in angular rate
	
		//		feed forward term for maneuvers
		double m = Tnomoto / Knomoto;
		tauFF = (m +Km) * (ad + rd / Tnomoto);
		//printf("%F  %f  %f  %f  %f  %f  %f\n", tauFF, m, Tnomoto, Km, gpsSpeedLp, ad, rd);
		
		//		Regulator calculation
		rudderMoment = tauFF*0.5 - Kpp * psiTilde - Kdp * rTilde -Kip * integralPsiTilde -Km * wDot;	//  With feed forward
		//rudderMoment =  - Kpp * psiTilde - Kdp * rTilde - Kip * integralPsiTilde -Km * wDot;			//  Without feed forward
		
	} else {		// If not mode == 2, let parameters follow actual values
		psid = yawIs;
		rd = 0;
		ad = 0;
		psiTilde = 0;
		rTilde = 0;
		integralPsiTilde = 0;
		rudderMoment = 0;
		lastMode = mode;
	}
	
	// Apend values to log file
	fprintf(fp, "%f %f %f %f %f %f %f %f %f %f\n",  yawCmd, psid, psiTilde, rTilde, integralPsiTilde, rudderMoment, tauFF, Kpp, Kdp, Kip);
	
	//fflush(fp);		// Empty buffer during debugging

	//printf("%f  %f  %f  %f  %f  %f %f  %f  %f  \n",  psid, psiTilde, rTilde, integralPsiTilde, rudderMoment, tauFF, Kpp, Kdp, Kip);	
	
	return deg180to180(rudderMoment);
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


void powerOff() {
	fclose(fp);
	system("poweroff");
}






