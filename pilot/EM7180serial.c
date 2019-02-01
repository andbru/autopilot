#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <errno.h>

#include "EM7180.h"

int initEM7180(int *htty) {

	*htty = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);

	if ( htty < 0)	
		return 0;
	else {
		struct termios ttyData;
		if ( tcgetattr(*htty, &ttyData) < 0) {
			printf("No ttyData\n");
			return -1;
		}

		//cfsetospeed(&ttyData, (speed_t)B38400);
		//cfsetispeed(&ttyData, (speed_t)B38400);
		
		cfsetospeed(&ttyData, (speed_t)B115200);
		cfsetispeed(&ttyData, (speed_t)B115200);
		
		fcntl(*htty,F_SETFL, FNDELAY);		//  No blocking in read, return zero if no data

	}

	// Clean input buffer and request new data to be read next cycle
	tcflush(*htty, TCIFLUSH);

	// Request new AHRS data
	write(*htty, "b", 2);
	
	printf("Init EM7180 ok!\n");

	return 0;
}

int pollEM7180(int *htty, unsigned long *ts, double *course, double *rate) {

	int rLen;
	char buf[100];

	rLen = read(*htty, buf,sizeof(buf) - 1); 	//  Read one line of data
	//printf("%d %s\n", rLen, buf);
	while (rLen > 0) {
		rLen = read(*htty, buf,sizeof(buf) - 1);	// Find last line in buffer
		//for(int ibuf = 0; ibuf <= strlen(buf) -1; ibuf++) {printf("%d  ", buf[ibuf]);}
		//printf("\n");
		//printf("%d %s \n", rLen, buf);
	}
	//printf("%s \n", strerror(errno));
	//printf("\n\n");
		
	unsigned long t;
	float c;
	float r;
	int k  = sscanf(buf, "$%lu,%f,%f*", &t, &c, &r);
	//printf("%d %lu %f %f \n", k, t, c, r);
		
	*ts = t;
	*course = c;
	*rate = r;
	
	// Clean input buffer and request new data to be read next cycle
	tcflush(*htty, TCIFLUSH);

	// Request new AHRS data
	write(*htty, "b", 2);
	
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

	//struct timeval t3000;		// Print time in ms
	//gettimeofday(&t3000, NULL);
	//printf("%ld \n", t3000.tv_usec / 1000);
