
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
#include <ncurses.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <wiringSerial.h>

#include "cmd.h"

void initKnob(void) {

	pinMode(0, INPUT);		// Knob
	pinMode(2, INPUT);
	pinMode(3, INPUT);
	pullUpDnControl(0, PUD_UP);
	pullUpDnControl(2, PUD_UP);
	pullUpDnControl(3, PUD_UP);
	
	pinMode(5, OUTPUT);	// LED
	pinMode(27, OUTPUT);
	digitalWrite(5, LOW);
	digitalWrite(27, LOW);

}


int pollKnob(int *mode, double *knobIncDec) {
	
	static int clk0 = 1;
	static int clk1 = 1;
	static int dt1 = 1;
	static int sw = 1;
	static int swShort = 1000;		// no of consecutive low to avoid switch bumps
	static int swLong = 30000;		// No of consecutive low for power off
	static int swCount = 0;
	static double degPerClick = 0.5;	// Gearing
	
	if(*mode == 0) return 0;		// Still starting up
	
	dt1 = digitalRead(2);
	clk1 = digitalRead(3);
	sw = digitalRead(0);
	
	int upDown = 0;
	if(clk1 < clk0) {		// falling
		if(dt1 == 1) upDown++; else upDown--;
	}
	if(clk1 > clk0) {		// raising
		if(dt1 == 1) upDown--; else upDown++;
	}
	clk0 = clk1;
	
	//Calculate Increment/Decrement
	if((*mode == 2) || (*mode == 7)) {
		*knobIncDec += upDown * degPerClick;
	} 
	
	// Switch short pulse = change mode, long pulse power off
	if(sw == 0) {
		swCount++ ;
		printf("%d\n", swCount);
		if(swCount >= swLong) system("poweroff");
	} else {
		if(swCount >= swShort) {
			if(*mode == 1) *mode = 2; 	
			else					
			if((*mode == 2) || (*mode == 7)) *mode = 1;	// Toggle mode
		}
		swCount = 0;		// Reset counting for spikes
		return 1;
	}
	
	return 0;
}


void initCmd(void) {

	initscr();
	//WINDOW *wp = initscr();
	cbreak();
	//nodelay(wp, TRUE);
	nodelay(stdscr, TRUE);
	keypad(stdscr, TRUE);
}


int pollCmd(int *mode, double *cmdIncDec) {
	double gain = 0.5;	// Gearing deg per click
	
	int cht;
	if((cht = getch()) == ERR) {		
										
		// Do nothing
		return 0;
	}
	else {
		switch (cht) {	
			case KEY_RIGHT:	// Increment					
				*cmdIncDec += gain;	
				return 1;	
			case KEY_LEFT:		// Decrement
				*cmdIncDec += -gain;
				return 1;
			case KEY_UP:		// Increment 5				
				*cmdIncDec += gain * 5;	
				return 1;	
			case KEY_DOWN:	// Decrement 5
				*cmdIncDec += -gain * 5;
				return 1;
			case 'q':		// quit
				endwin();
				digitalWrite(5, LOW);
				digitalWrite(27, LOW);
				return -1;	
			case 'r':		// rudder control
				*mode = 7;
				return  1;	
			case 's':		// standby  
				*mode = 1;
				return  1;	
			case 'h':		// heading hold 
				*mode = 2;
				return  1;					
		}
		return 1;
	}
}


