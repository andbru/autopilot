// Program for test of rudder servo

#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <linux/spi/spidev.h>
#include <time.h>
#include <sys/time.h>

#include "conversion.h"
#include "rudder.h"

double elapsed(struct timeval t1, struct timeval t0) {
	double elapsed = (t1.tv_sec - t0.tv_sec) * 1000.0;
	elapsed += (t1.tv_usec -t0.tv_usec) / 1000.0;

	return elapsed;
}

int main() {
	double rIs = 0;
	double rMeas = 0;
	struct timeval rt0, rt1;
	int tms = 0;
	int firsttime = 1;

	wiringPiSetup();

	initRudder();

	for(int i=0;i<20000000;i++) {
		int newAngle = pollRudder(&rMeas);
		if (newAngle == 1) {
			printf("%f %f\n", rMeas, rIs);	
			tms++;
		}

		if(tms > 1000) {
			if(firsttime) rIs = rMeas;
			firsttime = 0;

			gettimeofday(&rt1, NULL);
			if(elapsed(rt1, rt0) > 10) {
				rt0 = rt1;
//				rIs = rMeas;
				actuateRudder( 2.0, rMeas, &rIs);
//				printf("%f %f\n", rMeas, rIs);
//				printf("Hej\n");
			}
		} else rIs = rMeas;

	} 
	
	actuateRudder(rIs, rMeas, &rIs);
	digitalWrite(25,LOW);
	pwmWrite(1, 0);
	pwmWrite(24, 0);
}
