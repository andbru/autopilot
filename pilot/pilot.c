 
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


// Function prototypes, code at the end
double elapsed(struct timeval t1, struct timeval t0);
void initRudder(void);
int pollRudder(double *angel);
void actuateRudder(double rudderSet, double rudderIs);
void initKnob(void);
int pollKnob(int *mode, double *yawCmd);
void PIDAreg(int mode, double yawCmd, double yawIs, double w, double wDot);

// Globals

	
int main() {

	double rudderIs = 0;
	struct timeval t0, t1, tRud0, tRud1;
	struct timeval tReg0, tReg1;
	int mode = 0;
	double yawCmd = 0;
	double yawIs = 0;
	double w = 0;
	double wDot = 0;
	
	wiringPiSetup();	
	initRudder();
	initKnob();
	
	mode = 1;
	gettimeofday(&tRud0, NULL);
	gettimeofday(&tReg0, NULL);
	
	// Endless main loop
	for (;;) {
		
		//  Poll rudder angel every milli second and filter angel
		int newAngel = pollRudder(&rudderIs);
		if(newAngel) {
			printf("%f  %d  %f  ", rudderIs, mode, yawCmd);
			
			gettimeofday(&t1, NULL);		// dt = time between iterations
			double dt = elapsed(t1, t0);
			t0 = t1;
			printf("    %f \n ", dt);
		}
		
		//  Poll command knob
		int newMode = pollKnob(&mode, &yawCmd);
		if(newMode) printf("%d  %f\n", mode, yawCmd);
				
		//  Call PID regulator 10 times per second independent of mode.
		//  That gives PID opportunity to follow compass in standby
		gettimeofday(&tReg1, NULL);
		if(elapsed(tReg1, tReg0) > 10.0) {
			tReg0 = tReg1;
			PIDAreg(mode, yawCmd, yawIs, w, wDot);
		}
		
		//  Call rudder actuator 100 times per second if mode == 2
		if(mode == 2) {
			gettimeofday(&tRud1, NULL);
			if(elapsed(tRud1, tRud0) > 10.0) {
				tRud0 = tRud1;
				actuateRudder(yawCmd, rudderIs);
			}
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
	static double degPerVolt = 81.8 ;
	static double uZeroDeg = 1.30;
	
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
	double rudderMax = 20.0;
	double rudderMin = -20.0;
	double db = 0.035;		//  Dead band
	double slow = 0.14;		// Slow speed interval
	double pFast = 800;		// Max 1024
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
	
	if(out > 0) {
		digitalWrite(25, HIGH);
		pwmWrite(1, 0);
		pwmWrite(24, out);
	}
	if(out < 0) {
		digitalWrite(25, HIGH);	
		pwmWrite(24, 0);
		pwmWrite(1, -out);
	}
	if(out == 0) {
		digitalWrite(25, LOW);
		pwmWrite(1, 0);
		pwmWrite(24, 0);	
	}
	
	return;
}

void initKnob(void) {

	pinMode(0, INPUT);
	pinMode(2, INPUT);
	pinMode(3, INPUT);
	pullUpDnControl(0, PUD_UP);
	pullUpDnControl(2, PUD_UP);
	pullUpDnControl(3, PUD_UP);

}


int pollKnob(int *mode, double *yawCmd) {
	
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
	*yawCmd += upDown * degPerClick;
	
	if(sw == 0) swCount--; else 	swCount = swStart;	
	if(swCount == 0) {
		if(*mode == 1) *mode = 2; else *mode = 1;		// Toggle mode
		return 1;
	}
	
	return 0;
}


void PIDAreg(int mode, double yawCmd, double yawIs, double w, double wDot) {

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
	
	printf("\n%f %f %f\n", psid/3.14*180, rd, ad);

}
