#ifndef KALMAN_SIMPLE_H
#define KALMAN_SIMPLE_H

#include <kalman/kfilter.hpp>
#include <kalman/kvector.hpp>
#include <kalman/kmatrix.hpp>

class KalmanSimple: public Kalman::KFilter<double,1, false, true, true> {
public:
	KalmanSimple(const double aObservationNoise, const double aVelocityNoise);
	void setDT(const double aDT);
protected:

	void makeA();
	void makeH();
	void makeV();
	void makeR();
	void makeW();
	void makeQ();
	void makeB();
	
	double dt;
        double observationNoise;
        double velocityNoise;
};

typedef Kalman::KVector<double, 1, 1> Vector;
typedef Kalman::KMatrix<double, 1, 1> Matrix;
#endif
