
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

// Globals
int counter = 0;		// Global counter for test of thread handling
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;	// Global semafor for thread safty
FILE *fp;		//Global filehandle to make it possible to log in all routines
	
int main() {
	
	int rc1;
	pthread_t th2;
	
	char fname[SIZE];
	time_t curtime;
	struct tm *loctime;
	
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
	
	initRudder();
	initKnob();
	initCmd();
	
	if((rc1 = pthread_create(&th2, NULL, &th2func, NULL))) printf("No thread created\\n");
	
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
			rudderPID = PIDAreg(mode, yawCmd, yawIs, w, wDot);
			
			if(mode == 0) { digitalWrite(greenLed, LOW); digitalWrite(redLed, LOW);}	//Light up the Led's
			if(mode == 1) { digitalWrite(greenLed, HIGH); digitalWrite(redLed, LOW);}
			if(mode == 2) { digitalWrite(greenLed, LOW); digitalWrite(redLed, HIGH);}
			if(mode == 7) { digitalWrite(greenLed, HIGH); digitalWrite(redLed, HIGH);}
			
			pthread_mutex_lock(&mutex1);
				//fprintf(fp, "%d   %f   %f %f %f       %d\n", mode ,yawCmd, yawIs, rudderSet, rudderIs, counter);
			pthread_mutex_unlock(&mutex1);
			
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
	static double uZeroDeg = 1.596;
	
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
	double rudderMax = 10.0;
	double rudderMin = -10.0;
	double db = 0.5;		//  Dead band (deg)
	double slow = 1.0;		// Slow speed interval (deg)
	double pFast = 400;		// Max 1024
	double pSlow = 200;
	
	if(rudderIs < rudderMin) return;
	if(rudderIs > rudderMax) return;
	
	int out = 0;
	double dr = rudderSet - rudderIs;
	if(dr < -slow) out = - pFast;
	if((-slow < dr) && (dr < -db)) out = - pSlow;
	if((-db < dr) && (dr < db)) out = 0;
	if((db < dr) && (dr < slow)) out = pSlow;
	if( dr > slow) out = pFast;

	static int rc = 0;
/*
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
*/	
	rc++;
	
	printf("%d  %f\r\n", out, rudderIs);
	
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
		fprintf(fp, "%f  %f  %d\n", rudderSet, rudderIs, out);
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
	for(;;) {
		//printf("HEJSAN\r\n");
		pthread_mutex_lock(&mutex1);
			counter++;
		pthread_mutex_unlock(&mutex1);
		sleep(1);
	}
	return 0;
}
