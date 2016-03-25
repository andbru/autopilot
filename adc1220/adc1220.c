 
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


double elapsed(struct timeval t1, struct timeval t0){

	double elapsed = (t1.tv_sec - t0.tv_sec) * 1000;		// milliseconds
	elapsed += (t1.tv_usec - t0.tv_usec) / 1000.0;

	return elapsed;

}


int main() {
	int adcHandle;
	struct timeval t0, t1;
	
	wiringPiSetup();
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
	
	int fsRaw = 0x7fffff;		// Full scale digital
	double fsU = 3.3;			// Full scale volts
	double fu = 0;				// Filtered voltage
	double fk = 0.1;			// Filter constant
		
	// Endless read loop
	for (;;) {
	
		// Check DRDY#
		if (digitalRead(6) == 0) {
			// Read 3 data bytes
			unsigned char rData[4];
			rData[0] = 0x10;		// RDATA cmd
			rData[1] = 0x00;
			rData[2] = 0x00;
			rData[3] = 0x00;
			ioc = wiringPiSPIDataRW(0, rData, 4);
			//printf("Mode set return should be > -1: %d\n", ioc);
			//printf("MRead data = : %#x  %#x  %#x\n", rData[1], rData[2], rData[3]);
			int res = rData[1] * 256 * 256 + rData[2] * 256 + rData[3];
			printf("%#x  %d  ", res, res);
			
			double u = res * fsU / fsRaw;	// Conversion to Volt
			printf("%f  ", u);  
			
			fu = u * fk + (1.0 - fk) * fu; 		// Digital filter
			printf("%f", fu);  
			
			gettimeofday(&t1, NULL);		// dt = time between iterations
			double dt = elapsed(t1, t0);
			t0 = t1;
			printf("    %f \n ", dt);
		}
	}

	return 0;
}

