
// Sensor fusion with Cavallo Kalman filter, gyro and compass.		Anders Bruse 2016

struct taitBryanYPR updateCavallo(double ax, double ay, double az,
		  double wx, double wy, double wz,
		  double mx, double my, double mz, double dt);
