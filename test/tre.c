
#include<stdio.h>
#include<wiringPi.h>

int main(void) {

	wiringPiSetup();
	pinMode(0, INPUT);
	pinMode(2, INPUT);
	pinMode(3, INPUT);
	pullUpDnControl(0, PUD_UP);
	pullUpDnControl(2, PUD_UP);
	pullUpDnControl(3, PUD_UP);
	
	printf("\n\n\n");
	
	int clk0 = 1;
	int clk1 = 1;
	int dt0 = 1;
	int dt1 = 1;
	int count = 0;
	int count0 = 0;
	int sw = 1;
	while(sw) {
		dt1 = digitalRead(2);
		clk1 = digitalRead(3);
		sw = digitalRead(0);
		
		if(clk1 < clk0) {		// falling
			if(dt1 == 1) count++; else count--;
		}
		
		if(clk1 > clk0) {		// raising
			if(dt1 == 1) count--; else count++;
		}
		
		dt0 = dt1;
		clk0 = clk1;
		
		if(count != count0) printf("%d\n", count);
		count0 = count;
	}
	
	return 1;
}
