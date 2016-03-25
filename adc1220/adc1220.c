 
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
	//struct timeval t0, t1;
	
	adcHandle = wiringPiSPISetup(0, 1000000);			// Channel 0, speed 1 MHz, get filedescriptor
	printf("Filedescriptor: %d\n", adcHandle);
	
	char mode = SPI_MODE_1;
	int ioc = ioctl(adcHandle, SPI_IOC_WR_MODE, &mode);	// Set mode = 1
	printf("Mode set return should be > -1: %d\n", ioc);
	ioc = ioctl(adcHandle, SPI_IOC_RD_MODE, &mode);			// Check mode = 1
	printf("Mode get return should be = 1: %d\n", mode);
	
	// Write contents of control registers
	unsigned char ctrl[5];
	ctrl[0] = 0x00;
	ctrl[1] = 0x80;
	ctrl[2] = 0x04;
	ctrl[3] = 0x00;
	ctrl[4] = 0x00;
	ioc = wiringPiSPIDataRW(0, ctrl, 5);
	printf("Mode set return should be > -1: %d\n", ioc);

/*
	// Write to config reg, MSB = 48, LSB = 73 -- write swaps the bytes
	//int success = wiringPiI2CWriteReg16(adcHandle, 0x01, 0x7348);
	char buf[3];
	buf[0] = 0x01;
	buf[1] = 0x48;
	buf[2] = 0x73;
	ioc = write(adcHandle, buf, 3);

	double fres0, fres1;			// Filter variables
	for(;;) {	
	
		// Read conversion register, swap the bytes and shift right 4 bits
		//unsigned int iRes = wiringPiI2CReadReg16(adcHandle, 0x00);
		buf[0] = 0x00;		// Dummy write to set address
		buf[1] = 0x00;
		write(adcHandle, buf, 1);
		unsigned char val[2];
		read(adcHandle, &val, 2);

		int raw = val[0] * 256 + val[1];	// Combine the two bytes
		signed int res = raw >> 4;		// Shift the 12 bit value to lower bits
		if(res & 0x0800) res = res | 0xfffff000; // and fill out with "1"'s when negative 
		
		fres1 = res *0.05 + fres0*0.95;		// Digital first order LP-filter
		fres0 = fres1;
		
		printf("%#x   %d  %f    ", res, res, fres1);
		
		gettimeofday(&t1, NULL);
		double dAG = elapsed(t1, t0);
		t0 = t1;
		printf(" %f \n ", dAG);
	}
*/	
	return 0;
}

