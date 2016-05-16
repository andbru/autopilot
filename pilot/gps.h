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

