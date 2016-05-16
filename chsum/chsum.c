
#include <stdio.h>
#include<stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include<string.h>
#include<unistd.h>
#include <time.h>
#include <sys/time.h>
#include<fcntl.h>
#include<wiringPi.h>
#include <wiringSerial.h>

int main(void) {
	
	char nmea[100]  = "$PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*\0";
	
	
	int lenNmea = strlen(nmea);
	unsigned int chSum = 0;					// Calculate checksum before corrupting string
	int iSum = 0;
	for(iSum = 1; iSum <= lenNmea - 2; iSum++) {
		chSum = chSum ^ nmea[iSum];
	}
	
	printf("%d   Checksum = %2X\n", strlen(nmea), chSum);
	
	return(1);

}
