#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

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

			cfsetospeed(&ttyData, (speed_t)B38400);
			cfsetispeed(&ttyData, (speed_t)B38400);
			
			//fcntl(*htty,F_SETFL, FNDELAY);		//  No blocking in read, return zero if no data
	}

/*
	int rLen = 0;
	char buf[100];
	rLen = read(*htty, buf,sizeof(buf) - 1); 		//  Empty buffer
	while(rLen > 0) {
		rLen = read(*htty, buf,sizeof(buf) - 1); 	//  Read data until nothing left
	}
*/
	
	printf("Init EM7180 ok!\n");

	return 0;
}

int pollEM7180(int *htty, unsigned long *ts, double *course, double *rate) {

	int rLen;
	char buf[100];
	

	tcflush(*htty, TCIFLUSH);
/*
	rLen = read(*htty, buf,sizeof(buf) - 1); 		//  Empty buffer
	if(rLen != 0) printf("%d \n", rLen);
	while(rLen > 0) {
		rLen = read(*htty, buf,sizeof(buf) - 1); 	//  Read data until nothing left
		if(rLen != 0) printf("I loopen : %d \n", rLen);
	}
*/

	// Request new AHRS data
	write(*htty, "b", 2);

	rLen = read(*htty, buf,sizeof(buf) - 1); 	//  Read one line of data

	unsigned long t;
	float c;
	float r;
	int k  = sscanf(buf, "%lu,%f,%f", &t, &c, &r);
	//printf("%d %lu %f %f \n", k, t, c, r);
	
	*ts = t;
	*course = c;
	*rate = r;
	
	buf[0] = '\n';		// Clean buffer
	
	return k;

}

// Useful code snippets

	/*	Print buffer in hexadecimal
	if(count > 2500) {
		printf("%d \n", count);
		for(int ibuf = 0; ibuf <= strlen(buf) -1; ibuf++) {printf("%x  ", buf[ibuf]);}
		printf("\n");
	}
	*/

	//struct timeval t3000;		Print time in ms
	//gettimeofday(&t3000, NULL);
	//printf("%ld \n", t3000.tv_usec);
