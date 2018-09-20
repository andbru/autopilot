#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>

#include "EM7180.h"

int initEM7180(int *htty) {

	*htty = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);

	if ( htty < 0)	
		return 0;
	else {
			struct termios ttyData;
			if ( tcgetattr(*htty, &ttyData) < 0) {
				printf("No ttyData\n");
				return -1;
			}

			cfsetospeed(&ttyData, (speed_t)B115200);
			cfsetispeed(&ttyData, (speed_t)B115200);
			
			fcntl(*htty,F_SETFL, FNDELAY);		//  No blocking in read, return zero if no data
	}

	int rLen = 0;
	char buf[100];
	rLen = read(*htty, buf,sizeof(buf) - 1); 		//  Empty buffer
	while(rLen > 0) {
		rLen = read(*htty, buf,sizeof(buf) - 1); 	//  Read data until nothing left
	}
	printf("Init EM7180 ok!\n");

	return 0;
}

int pollEM7180(int *htty, unsigned long *ts, double *course, double *rate) {
	//printf("Hejsan poll\n");
	int rLen;
	char buf[100];
	char line[100];
	line[0] = '\0';
	int count = 0;

	rLen = read(*htty, buf,sizeof(buf) - 1); 		//  Empty buffer

	write(*htty, "b", sizeof("b"));

	while (1)  {
		count++;

		rLen = read(*htty, buf,sizeof(buf) - 1); 	//  Read data
	
		int lCount;
		for(lCount = 0; lCount <= rLen - 1; lCount++) {

			char c = buf[lCount]; //  printf("BP \n");
			//printf("%x  ", c);
			int lLen = strlen(line);
			line[lLen] = c;						//  Append to line
			lLen++;
			line[lLen] = '\0';					//  Add null-terminator

			if(c == '\n' ) {						//  New line completed
				//printf("%s", line);

				if(lLen < 10) break;			//  Too short string, not valit. Try again.

				const char s[2] = ",";
				char *param;
				param = strtok(line, s);			//  Find first parameter
				char *ptr;
				*ts = strtoul(param, &ptr, 10);

				param = strtok(NULL, s);		//  Next find course
				*course = strtod(param, &ptr);

				param = strtok(NULL, s);		//  Next find angle rate
				*rate = strtod(param, &ptr);

				line[0] = '\0';					//  Reset line to empty
				return 1;
			}
		}	
	}
	return 1;
}
