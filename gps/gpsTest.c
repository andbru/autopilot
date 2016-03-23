
#include <stdio.h>
#include <math.h>
#include<wiringPi.h>
#include <wiringSerial.h>

int main(void) {

	int handle = serialOpen("/dev/ttyAMA0", 9600);
	
	for (;;) {
		
		char c = serialGetchar(handle);
		printf("%c", c);
	}
	
	serialClose(handle);
	
	
	return 0;
	
}
