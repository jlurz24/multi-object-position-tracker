#include "kalman_simple.h"
#include <math.h>
#include <iostream>

using namespace std;

inline double square(double x){
	return x * x;
}

inline double cube(double x){
	return x * x * x;
}

inline double quad(double x){
	return x * x * x * x;
}

const double V_NOISE = 0.1;
const double OBS_NOISE = 0.001;
const double V_GAMMA = 1.;
const double SIGMA = 0.0001; // Regularization constant

// Filter's Initial state uncertainty: System state is unknown
const double i_P_NOISE = 1000.;
const double i_V_NOISE = 10.;

KalmanSimple::KalmanSimple() 
{
	setDim(6, 0, 6, 3, 3);
	dt = 1;
}

void KalmanSimple::setDT(const double aDT){
	dt = aDT;
}

void KalmanSimple::makeA()
{
	// P = P + dt * t
	A(1,1) = 1.;
	A(1,4) = dt;
	
	A(2,2) = 1.;
	A(2,5) = dt;
	
	A(3,3) = 1.;
	A(3,6) = dt;

	// Velocity I matrix
	A(4,4) = 1;
	A(5,5) = 1.;
	A(6,6) = 1.;
}

void KalmanSimple::makeW()
{
	W(1,1) = 1.0;
	W(2,2) = 1.0;
	W(3,3) = 1.0;
	W(4,4) = 1.0;
	W(5,5) = 1.0;
	W(6,6) = 1.0;
}

void KalmanSimple::makeB(){
}
void KalmanSimple::makeQ()
{
	Q(1,1) = quad(dt) / 4.0 * square(V_NOISE) + SIGMA;
	Q(1,2) = 0.;
	Q(1,3) = 0.;
	Q(1,4) = cube(dt) / 2 * square(V_NOISE);
	Q(1,5) = 0.;
	Q(1,6) = 0.;

	Q(2,1) = 0.;
	Q(2,2) = quad(dt) / 4.0 * square(V_NOISE) + SIGMA;
        Q(2,3) = 0.;
        Q(2,4) = 0.;
        Q(2,5) = cube(dt) / 2 * square(V_NOISE);
        Q(2,6) = 0.;

	Q(3,1) = 0.;
	Q(3,2) = 0.;
        Q(3,3) = quad(dt) / 4.0 * square(V_NOISE) + SIGMA;
        Q(3,4) = 0.;
        Q(3,5) = 0.;
        Q(3,6) = cube(dt) / 2 * square(V_NOISE);
	
	Q(4,1) = cube(dt) / 2 * square(V_NOISE);
	Q(4,2) = 0.;
	Q(4,3) = 0.;
	Q(4,4) = square(dt) * square(V_NOISE) + SIGMA;
	Q(4,5) = 0.;
	Q(4,6) = 0.;

	Q(5,1) = 0.;
	Q(5,2) = cube(dt) / 2 * square(V_NOISE);
        Q(5,3) = 0.;
        Q(5,4) = 0.;
        Q(5,5) = square(dt) * square(V_NOISE) + SIGMA;
        Q(5,6) = 0.;

	Q(6,1) = 0.;
	Q(6,2) = 0.;
        Q(6,3) = cube(dt) / 2 * square(V_NOISE);
        Q(6,4) = 0.;
        Q(6,5) = 0.;
        Q(6,6) = square(dt) * square(V_NOISE) + SIGMA;
}

void KalmanSimple::makeH()
{
	H(1,1) = 1.;
	H(2,2) = 1.;
	H(3,3) = 1.;
}

void KalmanSimple::makeV()
{
	V(1,1) = V_GAMMA;
	V(2,2) = V_GAMMA;
	V(3,3) = V_GAMMA;
}

void KalmanSimple::makeR()
{
	R(1,1) = square(OBS_NOISE);
	R(2,2) = square(OBS_NOISE);
	R(3,3) = square(OBS_NOISE);
}

int main(){
	const int n = 6;
	const int m = 3;

	KalmanSimple filter;

	Vector x(n);
	
	static const double _P0[] = {square(i_P_NOISE), 0., 0., 0., 0., 0.,
			             0., square(i_P_NOISE), 0., 0., 0., 0.,
				     0., 0., square(i_P_NOISE), 0., 0., 0.,
                                     0., 0., 0., square(i_V_NOISE), 0., 0.,
				     0., 0., 0., 0., square(i_V_NOISE), 0.,
				     0., 0., 0., 0., 0., square(i_V_NOISE)};
	Matrix P0(n, n, _P0);

	// Initial Estimate
	x(1) = 100;
	x(2) = 200;
	x(3) = 300;
        x(4) = 1;
	x(5) = 1;
	x(6) = 1;
	
	Vector x_true(n);
	x_true(1) = x(1);
	x_true(2) = x(2);
	x_true(3) = x(3);
	x_true(4) = x(4);
	x_true(5) = x(5);
	x_true(6) = x(6);

	Vector u(0);

	Vector z(m);
	z(1) = x_true(1);
	z(2) = x_true(2);
	z(3) = x_true(3);

	filter.init(x, P0);
	for(int i = 0; i < 10; ++i){
		x_true(1) += 1 * i;
		x_true(2) += 10 * i;
		x_true(3) += 100 * i;
		x_true(4) = i;
		x_true(5) = 10 * i;
		x_true(6) = 100 * i;

		z(1) = x_true(1);
		z(2) = x_true(2);
		z(3) = x_true(3);
		filter.step(u, z);
		cout << "Step " << filter.getX() << endl;
	}
	cout << "True: " << x_true << endl;
	cout << "Estimated: " << filter.getX() << endl;
	
	return 0;
}
