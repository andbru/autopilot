
// Sensor fusion with Cavallo et al Kalman filter.		Anders Bruse 2016

#include <stdio.h>
#include <wiringPi.h>
#include <math.h>
#include <stdbool.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_cblas.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_matrix.h>

#include"conversion.h"

// Function declarations //
void matrixPrint(gsl_matrix * m, int imax, int jmax);
void vectorPrint(gsl_vector * v, int imax);
void matrixInverse(gsl_matrix * m, gsl_matrix * mi, int n);

//  Struct for YPR return values	
struct fusionResult {
	double yaw;
	double w;
	double wdot;
};

extern bool lastTime;

struct fusionResult updateCavallo(double ax, double ay, double az,
		  double wxDeg, double wyDeg, double wzDeg,
		  double mx, double my, double mz, double dt) {
	
	double wx = degtorad(wxDeg);
	double wy = degtorad(wyDeg);
	double wz = degtorad(wzDeg);
		  
	static bool firstTime = true;		//  Initializaton of matrices are to complex to do
										//  in the declaration statment. Solved by creating a code
										//  segment thats executed only once

	struct fusionResult retC;

	//  Matrices and vectors are declared static to avoid initialization in each iteration
	//  x and P also for keeping their value between iterations
	static gsl_matrix * Q;
	static gsl_matrix * R;
	static gsl_matrix * P;
	static gsl_vector * x;
	static gsl_vector * z;
	static gsl_matrix * F;
	static gsl_matrix * C;	//  For temporary results
	static gsl_matrix * Rq;
	static gsl_vector * m;
	static gsl_vector * rm;
	static gsl_vector * h;
	static gsl_matrix * H;
	static gsl_vector * y;
	static gsl_matrix * D;	//  For temporary results
	static gsl_matrix * S;
	static gsl_matrix * Si;
	static gsl_matrix * K;
	static gsl_matrix * I;
		
	if(firstTime) {	//  Initialize matrices and vectors first time updateCavallo() is called
		 firstTime = false;
		 /*
		 Q = gsl_matrix_calloc(7, 7);
		 double kq = 1e-6;
		 double kw = 1e-3;
		 gsl_matrix_set(Q, 0, 0, kq);
		 gsl_matrix_set(Q, 1, 1, kq);
		 gsl_matrix_set(Q, 2, 2, kq);
		 gsl_matrix_set(Q, 3, 3, kq);
		 gsl_matrix_set(Q, 4, 4, kw);
		 gsl_matrix_set(Q, 5, 5, kw);
		 gsl_matrix_set(Q, 6, 6, kw);
		 
		 R = gsl_matrix_calloc(9, 9);
		 gsl_matrix_set(R, 0, 0, 1e-4);
		 gsl_matrix_set(R, 1, 1, 1e-4);
		 gsl_matrix_set(R, 2, 2, 1e-4);
		 gsl_matrix_set(R, 3, 3, 2e-5);
		 gsl_matrix_set(R, 4, 4, 2e-5);
		 gsl_matrix_set(R, 5, 5, 2e-5);
		 gsl_matrix_set(R, 6, 6, 0.001);
		 gsl_matrix_set(R, 7, 7, 0.001);
		 gsl_matrix_set(R, 8, 8, 0.001);
		 */
		 
		 Q = gsl_matrix_calloc(7, 7);
		 double kq = 7.5e-6;
		 double kw = 1e-3;
		 gsl_matrix_set(Q, 0, 0, kq);
		 gsl_matrix_set(Q, 1, 1, kq);
		 gsl_matrix_set(Q, 2, 2, kq);
		 gsl_matrix_set(Q, 3, 3, kq);
		 gsl_matrix_set(Q, 4, 4, kw);
		 gsl_matrix_set(Q, 5, 5, kw);
		 gsl_matrix_set(Q, 6, 6, kw);
		 
		 R = gsl_matrix_calloc(9, 9);
		 gsl_matrix_set(R, 0, 0, 0.001);
		 gsl_matrix_set(R, 1, 1, 0.001);
		 gsl_matrix_set(R, 2, 2, 0.001);
		 gsl_matrix_set(R, 3, 3, 0.1);
		 gsl_matrix_set(R, 4, 4, 0.1);
		 gsl_matrix_set(R, 5, 5, 0.1);
		 gsl_matrix_set(R, 6, 6, 0.001);
		 gsl_matrix_set(R, 7, 7, 0.001);
		 gsl_matrix_set(R, 8, 8, 0.001);
		 
		 P = gsl_matrix_calloc(7, 7);
		 gsl_matrix_set_identity(P);
		 
		 x = gsl_vector_calloc(7);
		 //gsl_vector_set(x, 0, 1.0);
		double initPsi = atan2(my, -mx) - 3.14*75/180;
		gsl_vector_set(x, 0, cos(initPsi / 2));
		gsl_vector_set(x, 3, -sin(initPsi / 2));
		//gsl_vector_set(x, 0, -0.573);
		//gsl_vector_set(x, 1, -0.0217);
		//gsl_vector_set(x, 2, -0.0044);
		//gsl_vector_set(x, 3, 0.8198);
	
		 z = gsl_vector_calloc(9);		 
		 F = gsl_matrix_calloc(7, 7);		 
		 C = gsl_matrix_calloc(7, 7);		 
		 Rq = gsl_matrix_calloc(3, 3);
		 m = gsl_vector_calloc(3);
		 rm = gsl_vector_calloc(3);
		 h = gsl_vector_calloc(9);
		 H = gsl_matrix_calloc(9, 7);
		 y = gsl_vector_calloc(9);
		 D = gsl_matrix_calloc(7, 9);
		 S = gsl_matrix_calloc(9, 9);
		 Si = gsl_matrix_calloc(9, 9);
		 K = gsl_matrix_calloc(7, 9);
		 
		 I = gsl_matrix_calloc(7, 7);
		 gsl_matrix_set_identity(I);		 
		 
	}	//  End of initialization in first iteration

	
	//  Start of code segment that are executed every iteration
	//
		
	// Import values from last iteration
	double q0 = gsl_vector_get ( x, 0);
	double q1 = gsl_vector_get ( x, 1);
	double q2 = gsl_vector_get ( x, 2);
	double q3 = gsl_vector_get ( x, 3);
	double wxh = gsl_vector_get ( x, 4);
	double wyh = gsl_vector_get ( x, 5);
	double wzh = gsl_vector_get ( x, 6);
	//vectorPrint(x, 7);
	//printf("%f %f %f %f %f %f\n", mx,my, q0, q1, q2 , q3);

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
	gsl_vector_set ( x, 1, q1+dt/2*(q0*wxh-q3*wyh+q2*wzh));//
	gsl_vector_set ( x, 2, q2+dt/2*(q3*wxh+q0*wyh-q1*wzh));//
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
	gsl_matrix_set(F, 1, 2, dt/2*wzh);	//
	gsl_matrix_set(F, 1, 3, -dt/2*wyh);	//
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
	//  rm = Rq * m (rm is a vector(3) and Rq is a matrix(3,3))
	gsl_vector_set (m, 0, gsl_vector_get (z, 6));
	gsl_vector_set (m, 1, gsl_vector_get (z, 7));
	gsl_vector_set (m, 2, gsl_vector_get (z, 8));
	gsl_blas_dgemv (CblasNoTrans, 1, Rq, m, 0, rm);
	double bx = sqrt( pow (gsl_vector_get (rm, 0), 2) + pow (gsl_vector_get (rm, 1), 2));
	double bz = gsl_vector_get (rm, 2);
	
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
	gsl_linalg_LU_invert (S, perm, Si);					// Invert the matrix
	gsl_permutation_free(perm);						// Free up memory to avoid leakage
	
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

	// Numerical differentiation to get wdot
	static double Tw = 0.25;
	static double xw = 0;
	xw = dt / Tw * (-xw + wzh) + xw;
	double wdot = 1 / Tw * (-xw + wzh);
	
	//  Assign return values
	retC.wdot = radtodeg(wdot);	
	retC.w = radtodeg(wzh) - 1.35;		// Adjust for bias
	//ret.yaw = deg0to360(radtodeg(atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))));	//  Return value in deg (0 to 360 deg)
	
	retC.yaw = deg0to360(90 -radtodeg(atan2(2*(-q0*q3+q1*q2), -1+2*(q0*q0+q1*q1))));	//  Return value in deg (0 to 360 deg)

	//printf("%f %f %f %f %f\n", atan2(-my, mx), q0, q1, q2, q3);

	return retC;
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

