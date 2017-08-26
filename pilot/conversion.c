
#include<math.h>
#include<string.h>

int strcmpNS(char *p1, char *p2) {	//strcmp NULL Safe
	if(p1 == 0 || p2 == 0) return -1;
	return strcmp(p1, p2);
}

double radtodeg(double angle) {
	return (angle * 180 / M_PI);
}

double degtorad(double angle) {
	return (angle / 180 * M_PI);
}

double deg0to360(double angle) {
	double res = angle;
	
	if(angle>1000 || angle<-1000) return 0;

	while(res < 0 || res >= 360) {
		if(res < 0) res += 360;
		if(res >= 360) res -= 360;
	}
	return res;
}

double deg180to180(double angle) {
	double res = angle;
	
	while(res <= -180 || res > 180) {
		if(res <= -180) res += 360;
		if(res > 180) res -= 360;
	}
	return res;
}

double rad0to2pi(double angle) {
	double res = angle;
	
	while(res < 0 || res >= 2 * M_PI) {
		if(res < 0) res += 2 * M_PI;
		if(res >= 2 * M_PI) res -= 2 * M_PI;
	}
	return res;
}

double radpitopi(double angle) {
	double res = angle;
	
	while(res <= -M_PI || res > M_PI) {
		if(res <= -M_PI) res += 2 * M_PI;
		if(res > M_PI) res -= 2 * M_PI;
	}
	return res;
}
