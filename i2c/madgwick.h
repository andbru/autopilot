
// Sensor fusion with Kalman filter, gyro and compass.		Anders Bruse 2015

struct filter updateMadgwick(double ax, double ay, double az,
		  double wx, double wy, double wz,
		  double mx, double my, double mz, double dt);
