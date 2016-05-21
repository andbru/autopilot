
#include<math.h>

double radtodeg(double angel) {
	return (angel * 180 / M_PI);
}

double degtorad(double angel) {
	return (angel / 180 * M_PI);
}

double deg0to360(double angel) {
	double res = angel;
	
	while(res < 0 || res >= 360) {
		if(res < 0) res += 360;
		if(res >= 360) res -= 360;
	}
	
	return res;
}

double deg180to180(double angel) {
	double res = angel;
	
	while(res <= -180 || res > 180) {
		if(res <= -180) res += 360;
		if(res > 180) res -= 360;
	}
	
	return res;
}
