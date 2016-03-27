
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

// Prototypes
int initGps(void);
bool pollGps( double *speed, double *course);
bool nmeaOk( double *speed, double *course); 

//Globals
int hGps;
char nmea[100] = "";

int main(void) {

	initGps();

	double gpsCourse;
	double gpsSpeed;
	serialFlush(hGps);
	for(;;) {
		bool newGps = pollGps(&gpsCourse, &gpsSpeed);
		if(newGps) printf("COG = %f   SOG = %f\n", gpsCourse, gpsSpeed);
	}
	
	serialClose(hGps);
	return 0;
}

int initGps() {
	int handle = serialOpen("/dev/ttyAMA0", 9600);
	if(handle<0) return -1;
	else {
	
		hGps = handle;
		serialFlush(hGps);
		return handle;
	}
}


bool pollGps(double *course, double *speed) {
	
	char c = serialGetchar(hGps);	// Read character
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
	return false;
}


bool nmeaOk( double *course, double *speed) {

	int lenNmea = strlen(nmea);
	unsigned int chSum = 0;					// Calculate checksum before corrupting string
	int iSum = 0;
	for(iSum = 1; iSum < lenNmea - 5; iSum++) {
		chSum = chSum ^ nmea[iSum];
	}
	
	printf("%s", nmea);
	
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