#include "kalman_simple.h"
#include <math.h>
#include <iostream>

using namespace std;

inline double square(double x){
	return x * x;
}

const double V_NOISE = 0.1;
const double OBS_NOISE = 0.001;
const double V_GAMMA = 1.;

// Filter's Initial state uncertainty: System state is unknown
const double i_P_NOISE = 1000.;
const double i_V_NOISE = 10.;

KalmanSimple::KalmanSimple() 
{
	setDim(2, 1, 2, 1, 1);
	dt = 1.;
}

void KalmanSimple::makeA()
{
	A(1,1) = 1.0;
	A(1,2) = dt;
	A(2,1) = 0.0;
	A(2,2) = 1;
}

void KalmanSimple::makeW()
{
	W(1,1) = 0.0;
	W(1,1) = 1.0;
	W(2,1) = 0.0;
	W(2,2) = 1.0;
}

void KalmanSimple::makeB(){
	B(1,1) = 0.0;
}
void KalmanSimple::makeQ()
{
	Q(1,1) = dt * dt * dt * dt / 4.0 * square(V_NOISE);
	Q(1,2) = dt * dt * dt / 2 * square(V_NOISE);
	Q(2,1) = dt * dt * dt / 2 * square(V_NOISE);
	Q(2,2) = dt * dt * square(V_NOISE);
}

void KalmanSimple::makeH()
{
	H(1,1) = 1.;
	H(1,2) = 0.;
}

void KalmanSimple::makeV()
{
	V(1,1) = V_GAMMA;
}

void KalmanSimple::makeR()
{
	R(1,1) = square(OBS_NOISE);
}

int main(){
	const int n = 2;
	const int m = 1;

	KalmanSimple filter;

	Vector x(n);
	
	static const double _P0[] = {square(i_P_NOISE), 0.0,
                                     0.0, square(i_V_NOISE)};
	Matrix P0(n, n, _P0);

	// Initial Estimate
	x(1) = 100;
        x(2) = 10;
	
	Vector x_true(n);
	x_true(1) = x(1);
	x_true(2) = x(2);

	Vector u(m);
	u(1) = 0;

	Vector z(1);
	z(1) = x_true(1);

	filter.init(x, P0);
	for(int i = 0; i < 10; ++i){
		x_true(1) += 10 * i;
		x_true(2) = i;
		z(1) = x_true(1);
		filter.step(u, z);
		cout << "Step " << filter.getX() << endl;
	}
	cout << "True: " << x_true << endl;
	cout << "Estimated: " << filter.getX() << endl;
	
	return 0;
}
