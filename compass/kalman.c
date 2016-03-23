
//  Globals from main
extern double roll;
extern double pitch;
extern double yaw;

// Sensor fusion with Kalman filter, gyro and compass.		Anders Bruse 2015

#include <stdio.h>
#include <wiringPi.h>
#include <math.h>

#include <gsl/gsl_linalg.h>
#include <gsl/gsl_cblas.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_matrix.h>

// Function declarations //
void matrixPrint(gsl_matrix * m, int imax, int jmax);
void vectorPrint(gsl_vector * v, int imax);
void matrixInverse(gsl_matrix * m, gsl_matrix * mi, int n);
void matrixDiag(gsl_matrix * m, int n, double a[]);

//                                                            Global variables                                                        //
// Matrices are declared globally to avoid them to be initialized in every function call //

double pitch = 0;
double roll = 0;
double yaw = 0;

int qnr = 7;
int qnc = 7;
gsl_matrix * Q;

int rnr = 9;
int rnc = 9;
gsl_matrix * R;

int pnr = 7;
int pnc = 7;
gsl_matrix * P;

int xnr = 7;
gsl_vector * x;

int znr = 9;
gsl_vector * z;

int fnr = 7;
int fnc =7;
gsl_matrix * F;

int rqnr = 3;
int rqnc = 3;
gsl_matrix * Rq;

int mnr = 3;
gsl_vector * m;

int rmnr = 3;
gsl_vector * Rm;

int hvnr = 9;
gsl_vector * h;

int ynr = 9;
gsl_vector * y;

int hnr = 9;
int hnc = 7;
gsl_matrix * H;

int snr = 9;
int snc = 9;
gsl_matrix * S;

int sinr = 9;
int sinc = 9;
gsl_matrix * Si;

int knr = 7;
int knc = 9;
gsl_matrix * K;

int inr = 7;
int inc = 7;
gsl_matrix * I;

int cnr = 7;	//  To store temporary results
int cnc =7;
gsl_matrix * C;

int dnr = 7;	//  To store temporary results
int dnc =9;
gsl_matrix * D;

unsigned long start;


void initKalman(void) {
	qnr = 7;

	// Initialize matrices and vectors
	Q = gsl_matrix_calloc (qnr, qnc);
	double kq = 1e-3;
	double kw = 1e-3;
	double Qd[7] = {kq, kq, kq, kq, kw, kw, kw};
	matrixDiag(Q, 7, Qd);
	matrixPrint(Q, 7, 7);
	printf("\n");
	
	R = gsl_matrix_calloc (rnr, rnc);
	double Rd[9] = {0.084, 0.091, 0.125, 1e-4, 1e-4, 1e-7, 3e-4, 7e-5, 7e-5};
	matrixDiag(R, 9, Rd);
	matrixPrint(R, 9, 9);
	printf("\n");
	
	P = gsl_matrix_calloc (pnr, pnc);
	double Pd[7] = {1, 1, 1, 1, 1, 1, 1};
	matrixDiag(P, 7, Pd);
	matrixPrint(P, 7, 7);
	printf("\n");
	
	x = gsl_vector_calloc (xnr);
	gsl_vector_set (x, 0, 1.0);
	
	z = gsl_vector_calloc (znr);
	
	F = gsl_matrix_calloc (fnr, fnc);
	Rq = gsl_matrix_calloc (rqnr, rqnc);
	m = gsl_vector_calloc (mnr);
	Rm = gsl_vector_calloc (rmnr);
	h = gsl_vector_calloc (hvnr);
	y = gsl_vector_calloc (ynr);
	H = gsl_matrix_calloc (hnr, hnc);
	S = gsl_matrix_calloc (snr, snc);
	Si = gsl_matrix_calloc (sinr, sinc);
	K = gsl_matrix_calloc (knr, knc);
	C = gsl_matrix_calloc (cnr, cnc);
	D = gsl_matrix_calloc (dnr, dnc);
	
	I = gsl_matrix_calloc (inr, inc);
	double Id[7] = {1, 1, 1, 1, 1, 1, 1};
	matrixDiag(I, 7, Id);

}


void updateKalman(double ax, double ay, double az,
		  double wx, double wy, double wz,
		  double mx, double my, double mz, double dt) {
	
	// Import values from last iteration
	double q0 = gsl_vector_get ( x, 0);
	double q1 = gsl_vector_get ( x, 1);
	double q2 = gsl_vector_get ( x, 2);
	double q3 = gsl_vector_get ( x, 3);
	double wxh = gsl_vector_get ( x, 4);
	double wyh = gsl_vector_get ( x, 5);
	double wzh = gsl_vector_get ( x, 6);
	//vectorPrint(x, 7);

	// Measurements
	gsl_vector_set (z, 0, ax);
	gsl_vector_set (z, 1, ay);
	gsl_vector_set (z, 2, az);
	gsl_vector_set (z, 3, wx);
	gsl_vector_set (z, 4, wy);
	gsl_vector_set (z, 5, wz);
	gsl_vector_set (z, 7, mx);
	gsl_vector_set (z, 6, my);
	gsl_vector_set (z, 8, mz);
	
	////////////  PREDICT  ///////////////
	//  Predicted state estimate
	//  x = f(x, u)
	gsl_vector_set ( x, 0, q0+dt/2*(-q1*wxh-q2*wyh-q3*wzh));
	gsl_vector_set ( x, 1, q1+dt/2*(q0*wxh-q3*wyh+q2*wzh));
	gsl_vector_set ( x, 2, q2+dt/2*(q3*wxh+q0*wyh-q1*wzh));
	gsl_vector_set ( x, 3, q3+dt/2*(-q2*wxh+q1*wyh+q0*wzh));
	gsl_vector_set ( x, 4, wxh);
	gsl_vector_set ( x, 5, wyh);
	gsl_vector_set ( x, 6, wzh);
	
	//  Normalize quaternion
	double qnorm = sqrt (pow(gsl_vector_get (x, 0), 2) + pow(gsl_vector_get (x, 1), 2) +
	      pow(gsl_vector_get (x, 2), 2) + pow(gsl_vector_get (x, 3), 2));
	gsl_vector_set (x, 0, gsl_vector_get (x, 0) / qnorm);
	gsl_vector_set (x, 1, gsl_vector_get (x, 1) / qnorm);
	gsl_vector_set (x, 2, gsl_vector_get (x, 2) / qnorm);
	gsl_vector_set (x, 3, gsl_vector_get (x, 3) / qnorm);
	q0 = gsl_vector_get ( x, 0);
	q1 = gsl_vector_get ( x, 1);
	q2 = gsl_vector_get ( x, 2);
	q3 = gsl_vector_get ( x, 3);
	
	//  Populate F Jacobian
	gsl_matrix_set(F, 0, 0, 1.0);
	gsl_matrix_set(F, 0, 1, -dt/2*wxh);
	gsl_matrix_set(F, 0, 2, -dt/2*wyh);
	gsl_matrix_set(F, 0, 3, -dt/2*wzh);
	gsl_matrix_set(F, 0, 4, -dt/2*q1);
	gsl_matrix_set(F, 0, 5, -dt/2*q2);
	gsl_matrix_set(F, 0, 6, -dt/2*q3);
	
	gsl_matrix_set(F, 1, 0, dt/2*wxh);
	gsl_matrix_set(F, 1, 1, 1.0);
	gsl_matrix_set(F, 1, 2, dt/2*wzh);
	gsl_matrix_set(F, 1, 3, -dt/2*wyh);
	gsl_matrix_set(F, 1, 4, dt/2*q0);
	gsl_matrix_set(F, 1, 5, -dt/2*q3);
	gsl_matrix_set(F, 1, 6, dt/2*q2);
	
	gsl_matrix_set(F, 2, 0, dt/2*wyh);
	gsl_matrix_set(F, 2, 1, -dt/2*wzh);
	gsl_matrix_set(F, 2, 2, 1.0);
	gsl_matrix_set(F, 2, 3, dt/2*wxh);
	gsl_matrix_set(F, 2, 4, dt/2*q3);
	gsl_matrix_set(F, 2, 5, dt/2*q0);
	gsl_matrix_set(F, 2, 6, -dt/2*q1);
		
	gsl_matrix_set(F, 3, 0, dt/2*wzh);
	gsl_matrix_set(F, 3, 1, dt/2*wyh);
	gsl_matrix_set(F, 3, 2, -dt/2*wxh);
	gsl_matrix_set(F, 3, 3, 1.0);
	gsl_matrix_set(F, 3, 4, -dt/2*q2);
	gsl_matrix_set(F, 3, 5, dt/2*q1);
	gsl_matrix_set(F, 3, 6, dt/2*q0);
		
	gsl_matrix_set(F, 4, 0, 0);
	gsl_matrix_set(F, 4, 1, 0);
	gsl_matrix_set(F, 4, 2, 0);
	gsl_matrix_set(F, 4, 3, 0);
	gsl_matrix_set(F, 4, 4, 1);
	gsl_matrix_set(F, 4, 5, 0);
	gsl_matrix_set(F, 4, 6, 0);
		
	gsl_matrix_set(F, 5, 0, 0);
	gsl_matrix_set(F, 5, 1, 0);
	gsl_matrix_set(F, 5, 2, 0);
	gsl_matrix_set(F, 5, 3, 0);
	gsl_matrix_set(F, 5, 4, 0);
	gsl_matrix_set(F, 5, 5, 1);
	gsl_matrix_set(F, 5, 6, 0);
		
	gsl_matrix_set(F, 6, 0, 0);
	gsl_matrix_set(F, 6, 1, 0);
	gsl_matrix_set(F, 6, 2, 0);
	gsl_matrix_set(F, 6, 3, 0);
	gsl_matrix_set(F, 6, 4, 0);
	gsl_matrix_set(F, 6, 5, 0);
	gsl_matrix_set(F, 6, 6, 1);
	
	//matrixPrint(F, 7, 7);
	
	//  Predicted covariance estmate
	//  P = F P F' + Q	
	gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1, P, F, 0, C);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1, F, C, 0, C);
	gsl_matrix_add (C, Q);
	gsl_matrix_memcpy (P, C);
	
	////////////////////////////////                UPDATE            ///////////////////////////////////
	//   Normalize accelerometer and magnetometer measurements
	double acc_norm =sqrt (pow(gsl_vector_get(z, 0), 2) + pow(gsl_vector_get(z, 1), 2) + pow(gsl_vector_get(z, 2), 2));
	gsl_vector_set (z, 0, gsl_vector_get (z, 0) / acc_norm);
	gsl_vector_set (z, 1, gsl_vector_get (z, 1) / acc_norm);
	gsl_vector_set (z, 2, gsl_vector_get (z, 2) / acc_norm);
	
	double mag_norm =sqrt (pow(gsl_vector_get(z, 6), 2) + pow(gsl_vector_get(z, 7), 2) + pow(gsl_vector_get(z, 8), 2));
	gsl_vector_set (z, 6, gsl_vector_get (z, 6) / mag_norm);
	gsl_vector_set (z, 7, gsl_vector_get (z, 7) / mag_norm);
	gsl_vector_set (z, 8, gsl_vector_get (z, 8) / mag_norm);
	
	//  Reference field calculation
	//  Build quaternion rotation matrix
	gsl_matrix_set (Rq, 0, 0, q0*q0+q1*q1-q2*q2-q3*q3);
	gsl_matrix_set (Rq, 0, 1, 2*(q1*q2-q0*q3));
	gsl_matrix_set (Rq, 0, 2, 2*(q1*q3+q0*q2));
	gsl_matrix_set (Rq, 1, 0, 2*(q1*q2+q0*q3));
	gsl_matrix_set (Rq, 1, 1, q0*q0-q1*q1+q2*q2-q3*q3);
	gsl_matrix_set (Rq, 1, 2, 2*(q2*q3-q0*q1));
	gsl_matrix_set (Rq, 2, 0, 2*(q1*q3-q0*q2));
	gsl_matrix_set (Rq, 2, 1, 2*(q2*q3+q0*q1));
	gsl_matrix_set (Rq, 2, 2, q0*q0-q1*q1-q2*q2+q3*q3);
	
	//  Rotate magnetic vector into reference frame
	//  Rm = Rq * m (Rm is a vector(3) and Rq is a matrix(3,3))
	gsl_vector_set (m, 0, gsl_vector_get (z, 6));
	gsl_vector_set (m, 1, gsl_vector_get (z, 7));
	gsl_vector_set (m, 2, gsl_vector_get (z, 8));
	gsl_blas_dgemv (CblasNoTrans, 1, Rq, m, 0, Rm);
	double bx = sqrt( pow (gsl_vector_get (Rm, 0), 2) + pow (gsl_vector_get (Rm, 1), 2));
	double bz = gsl_vector_get (Rm, 2);
	
	//  Populate h
	gsl_vector_set (h, 0, -2*(q1*q3-q0*q2));
	gsl_vector_set (h, 1, -2*(q2*q3+q0*q1));
	gsl_vector_set (h, 2, -q0*q0+q1*q1+q2*q2-q3*q3);
	gsl_vector_set (h, 3, wxh);
	gsl_vector_set (h, 4, wyh);
	gsl_vector_set (h, 5, wzh);
	gsl_vector_set (h, 6, bx*(q0*q0+q1*q1-q2*q2-q3*q3)+2*bz*(q1*q3-q0*q2));
	gsl_vector_set (h, 7, 2*bx*(q1*q2-q0*q3)+2*bz*(q2*q3+q0*q1));
	gsl_vector_set (h, 8, 2*bx*(q1*q3+q0*q2)+bz*(q0*q0-q1*q1-q2*q2+q3*q3));
	
	//  Measurement residual
	//  y = z - h(x)
	gsl_vector_sub (z, h);	// ATTENTION! the result is stored in z and overwrites measurements, 
							// but z is not used any more in this iteration
	gsl_vector_memcpy (y, z);
												
	//  Populate H Jacobian	
	gsl_matrix_set(H, 0, 0, 2*q2);
	gsl_matrix_set(H, 0, 1, -2*q3);
	gsl_matrix_set(H, 0, 2, 2*q0);
	gsl_matrix_set(H, 0, 3, -2*q1);
	gsl_matrix_set(H, 0, 4, 0);
	gsl_matrix_set(H, 0, 5, 0);
	gsl_matrix_set(H, 0, 6, 0);
	
	gsl_matrix_set(H, 1, 0, -2*q1);
	gsl_matrix_set(H, 1, 1, -2*q0);
	gsl_matrix_set(H, 1, 2, -2*q3);
	gsl_matrix_set(H, 1, 3, -2*q2);
	gsl_matrix_set(H, 1, 4, 0);
	gsl_matrix_set(H, 1, 5, 0);
	gsl_matrix_set(H, 1, 6, 0);
	
	gsl_matrix_set(H, 2, 0, -2*q0);
	gsl_matrix_set(H, 2, 1, 2*q1);
	gsl_matrix_set(H, 2, 2, 2*q2);
	gsl_matrix_set(H, 2, 3, -2*q3);
	gsl_matrix_set(H, 2, 4, 0);
	gsl_matrix_set(H, 2, 5, 0);
	gsl_matrix_set(H, 2, 6, 0);
		
	gsl_matrix_set(H, 3, 0, 0);
	gsl_matrix_set(H, 3, 1, 0);
	gsl_matrix_set(H, 3, 2, 0);
	gsl_matrix_set(H, 3, 3, 0);
	gsl_matrix_set(H, 3, 4, 1);
	gsl_matrix_set(H, 3, 5, 0);
	gsl_matrix_set(H, 3, 6, 0);
		
	gsl_matrix_set(H, 4, 0, 0);
	gsl_matrix_set(H, 4, 1, 0);
	gsl_matrix_set(H, 4, 2, 0);
	gsl_matrix_set(H, 4, 3, 0);
	gsl_matrix_set(H, 4, 4, 0);
	gsl_matrix_set(H, 4, 5, 1);
	gsl_matrix_set(H, 4, 6, 0);
		
	gsl_matrix_set(H, 5, 0, 0);
	gsl_matrix_set(H, 5, 1, 0);
	gsl_matrix_set(H, 5, 2, 0);
	gsl_matrix_set(H, 5, 3, 0);
	gsl_matrix_set(H, 5, 4, 0);
	gsl_matrix_set(H, 5, 5, 0);
	gsl_matrix_set(H, 5, 6, 1);
		
	gsl_matrix_set(H, 6, 0, 2*(q0*bx-q2*bz));
	gsl_matrix_set(H, 6, 1, 2*(q1*bx+q3*bz));
	gsl_matrix_set(H, 6, 2, 2*(-q2*bx-q0*bz));
	gsl_matrix_set(H, 6, 3, 2*(-q3*bx+q1*bz));
	gsl_matrix_set(H, 6, 4, 0);
	gsl_matrix_set(H, 6, 5, 0);
	gsl_matrix_set(H, 6, 6, 0);
	
	gsl_matrix_set(H, 7, 0, 2*(-q3*bx+q1*bz));
	gsl_matrix_set(H, 7, 1, 2*(q2*bx+q0*bz));
	gsl_matrix_set(H, 7, 2, 2*(q1*bx+q3*bz));
	gsl_matrix_set(H, 7, 3, 2*(-q0*bx+q2*bz));
	gsl_matrix_set(H, 7, 4, 0);
	gsl_matrix_set(H, 7, 5, 0);
	gsl_matrix_set(H, 7, 6, 0);
	
	gsl_matrix_set(H, 8, 0, 2*(q2*bx+q0*bz));
	gsl_matrix_set(H, 8, 1, 2*(q3*bx-q1*bz));
	gsl_matrix_set(H, 8, 2, 2*(q0*bx-q2*bz));
	gsl_matrix_set(H, 8, 3, 2*(q1*bx+q3*bz));
	gsl_matrix_set(H, 8, 4, 0);
	gsl_matrix_set(H, 8, 5, 0);
	gsl_matrix_set(H, 8, 6, 0);
	
	//  Residual covariance
	//  S = H * P * H' + R
	gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1, P, H, 0, D);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1, H, D, 0, S);
	gsl_matrix_add (S, R);
	//matrixInverse(S, Si, snr);
	//matrixInverse of S in Si
	int signum; 
	gsl_permutation * perm = gsl_permutation_alloc(9);		
	gsl_linalg_LU_decomp (S, perm, &signum);		// Make the LU decomposition of matrix m
	gsl_linalg_LU_invert (S, perm, Si);				// Invert the matrix
	
	//  Calculate Kalman gain	
	//  K = P * H' * Si
	//gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1, P, H, 0, D);
	//gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1, D, Si, 0, K);
	gsl_blas_dgemm (CblasTrans, CblasNoTrans, 1, H, Si, 0, D);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1, P, D, 0, K);
	
	//matrixPrint(K, 7, 9);

	//  Update state estimate
	//  x = x + K * y
	gsl_blas_dgemv (CblasNoTrans, 1, K, y, 1, x);
	
	//  Normalize quaternion
	qnorm = sqrt (pow(gsl_vector_get (x, 0), 2) + pow(gsl_vector_get (x, 1), 2) +
	      pow(gsl_vector_get (x, 2), 2) + pow(gsl_vector_get (x, 3), 2));
	gsl_vector_set (x, 0, gsl_vector_get (x, 0) / qnorm);
	gsl_vector_set (x, 1, gsl_vector_get (x, 1) / qnorm);
	gsl_vector_set (x, 2, gsl_vector_get (x, 2) / qnorm);
	gsl_vector_set (x, 3, gsl_vector_get (x, 3) / qnorm);
	q0 = gsl_vector_get ( x, 0);
	q1 = gsl_vector_get ( x, 1);
	q2 = gsl_vector_get ( x, 2);
	q3 = gsl_vector_get ( x, 3);
	
	//  Update estimate covariance
	//  P = (I - K * H) * P
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1, K, H, 0, C);
	gsl_matrix_sub (I, C);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1, I, P, 0, P);
	gsl_matrix_set_identity(I);
	//I = gsl_matrix_calloc (inr, inc);			// Reset I to 7x7 Identity matrix
	//double Id[7] = {1, 1, 1, 1, 1, 1, 1};
	//matrixDiag(I, 7, Id);
	
	//  Generate YPR - values
	pitch = (180/M_PI)*atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
	roll = (180/M_PI)*asin(2*(q0*q2-q3*q1));
	yaw = (180/M_PI)*atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
	
	yaw = 90 - yaw;		//  Switch to TaitBryan-angels Z-X'-Z'', yaw, pitch, roll
	if (yaw < -180) yaw = yaw + 360;
	if (yaw > 180) yaw = yaw - 360;
	pitch =180 - pitch;
	if (pitch < -180) pitch = pitch + 360;
	if (pitch > 180) pitch = pitch - 360;
	
	
	//  fi
	//printf("121  %f  ", 180/M_PI*atan2(2*q1*q2-2*q0*q3, 2*q1*q3+2*q0*q2));
	//printf("123  %f  ", 180/M_PI*atan2(2*q2*q3+2*q0*q1, q3*q3-q2*q2-q1*q1+q0*q0));
	//printf("131  %f  ", 180/M_PI*atan2(2*q1*q3+2*q0*q2, -2*q1*q2+2*q0*q3));
	//printf("132  %f  ", 180/M_PI*atan2(-2*q2*q3+2*q0*q1, q2*q2-q3*q3+q0*q0-q1*q1));
	//printf("212  %f  ", 180/M_PI*atan2(2*q1*q2+2*q0*q3, -2*q2*q3+2*q0*q1));
	//printf("213  %f  ", 180/M_PI*atan2(-2*q1*q3+2*q0*q2, q3*q3-q2*q2-q1*q1+q0*q0));
	//printf("231  %f  ", 180/M_PI*atan2(2*q1*q3+2*q0*q2, q1*q1+q0*q0-q3*q3-q2*q2));
	//printf("232  %f  ", 180/M_PI*atan2(2*q2*q3-2*q0*q1, 2*q1*q2+2*q0*q3));
	//printf("312  %f  ", 180/M_PI*atan2(2*q1*q2+2*q0*q3, q2*q2-q3*q3+q0*q0-q1*q1));
	//printf("313  %f  ", 180/M_PI*atan2(2*q1*q3-2*q0*q2, 2*q2*q3+2*q0*q1));
	//printf("321  %f  ", 180/M_PI*atan2(-2*q1*q2+2*q0*q3, q1*q1+q0*q0-q3*q3-q2*q2));
	//printf("323  %f  ", 180/M_PI*atan2(2*q2*q3+2*q0*q1, -2*q1*q3+2*q0*q2));
	
	// theta
	//printf("121  %f  ", 180/M_PI*acos(q0*q0+q1*q1-q2*q2-q3*q3));
	//printf("123  %f  ", 180/M_PI*-asin(2*q1*q3-2*q0*q2));
	//printf("131  %f  ", 180/M_PI*acos(q0*q0+q1*q1-q2*q2-q3*q3));
	//printf("132  %f  ", 180/M_PI*asin(2*q1*q2+2*q0*q3));
	//printf("212  %f  ", 180/M_PI*acos(q0*q0-q1*q1+q2*q2-q3*q3));
	//printf("213  %f  ", 180/M_PI*asin(2*q2*q3+2*q0*q1));
	//printf("231  %f  ", 180/M_PI*-asin(2*q1*q2-2*q0*q3));
	//printf("232  %f  ", 180/M_PI*acos(q0*q0-q1*q1+q2*q2-q3*q3));
	//printf("312  %f  ", 180/M_PI*-asin(2*q2*q3-2*q0*q1));
	//printf("313  %f  ", 180/M_PI*acos(q0*q0-q1*q1-q2*q2+q3*q3));
	//printf("321  %f  ", 180/M_PI*asin(2*q1*q3+2*q0*q2));
	//printf("323  %f  ", 180/M_PI*acos(q0*q0-q1*q1-q2*q2+q3*q3));
	
	//  psi
	//printf("121  %f  ", 180/M_PI*atan2(2*q1*q2+2*q0*q3, -2*q1*q3+2*q0*q2));
	//printf("123  %f  ", 180/M_PI*atan2(2*q1*q2+2*q0*q3, -q3*q3-q2*q2+q1*q1+q0*q0));
	//printf("131  %f  ", 180/M_PI*atan2(2*q1*q3-2*q0*q2, 2*q1*q2+2*q0*q3));
	//printf("132  %f  ", 180/M_PI*atan2(-2*q1*q3+2*q0*q2, -q2*q2-q3*q3+q0*q0+q1*q1));
	//printf("212  %f  ", 180/M_PI*atan2(2*q1*q2-2*q0*q3, 2*q2*q3+2*q0*q1));
	//printf("213  %f  ", 180/M_PI*atan2(-2*q1*q2+2*q0*q3, -q3*q3+q2*q2-q1*q1+q0*q0));
	//printf("231  %f  ", 180/M_PI*atan2(2*q2*q3+2*q0*q1, -q1*q1+q0*q0-q3*q3+q2*q2));
	//printf("232  %f  ", 180/M_PI*atan2(2*q2*q3+2*q0*q1, -2*q1*q2+2*q0*q3));
	//printf("312  %f  ", 180/M_PI*atan2(2*q1*q3+2*q0*q2, -q2*q2+q3*q3+q0*q0-q1*q1));
	//printf("313  %f  ", 180/M_PI*atan2(2*q1*q3+2*q0*q2, -2*q2*q3+2*q0*q1));
	//printf("321  %f  ", 180/M_PI*atan2(-2*q2*q3+2*q0*q1, -q1*q1+q0*q0+q3*q3-q2*q2));
	//printf("323  %f  ", 180/M_PI*atan2(2*q2*q3-2*q0*q1, 2*q1*q3+2*q0*q2));
	
	
}

void vectorPrint(gsl_vector * v, int imax) {
	int i;

	for( i = 0; i <= imax - 1; i++) {
			printf("%f  ", gsl_vector_get(v, i));
	}
	printf("\n");
	printf("\n");
}

void matrixPrint(gsl_matrix * m, int imax, int jmax) {
	int i;
	int j;
	for( i = 0; i <= imax - 1; i++) {
		for( j = 0; j <= jmax -1; j++) {
			printf("%f  ", gsl_matrix_get(m, i, j));
		}
		printf("\n");
	}
	printf("\n");
}

void matrixInverse(gsl_matrix * m, gsl_matrix * mi, int n) {
	int signum;
	gsl_permutation * perm = gsl_permutation_alloc(n);
	// Make the LU decomposition of matrix m
	gsl_linalg_LU_decomp (m, perm, &signum);
	// Invert the matrix
	gsl_linalg_LU_invert (m, perm, mi);
}

void matrixDiag(gsl_matrix * m, int n, double a[]) {
	int i;
	for( i = 0; i <= n - 1; i++) {
		gsl_matrix_set(m, i, i, a[i]);
	}
}

