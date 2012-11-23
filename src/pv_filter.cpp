#include "pv_filter.h"

using namespace std;

static inline double square(const double x){
  return x * x;
}

PVFilter::PVFilter(){
}

void PVFilter::setDT(const double dt){
  filter.setDT(dt);
}

void PVFilter::init(const vector<double>& positions, const vector<double>& velocities){
        // Number of variables
        const int n = 6;

	// Filter's Initial state uncertainty: System state is unknown
	const double i_P_NOISE = 1000.;
	const double i_V_NOISE = 10.;

        // Initial covariance matrix
        static const double _P0[] = {square(i_P_NOISE), 0., 0., 0., 0., 0.,
                                     0., square(i_P_NOISE), 0., 0., 0., 0.,
                                     0., 0., square(i_P_NOISE), 0., 0., 0.,
                                     0., 0., 0., square(i_V_NOISE), 0., 0.,
                                     0., 0., 0., 0., square(i_V_NOISE), 0.,
                                     0., 0., 0., 0., 0., square(i_V_NOISE)};
        Matrix P0(n, n, _P0);

        // Initial Estimate
        Vector x(n);
        x(1) = positions[0];
        x(2) = positions[1];
        x(3) = positions[2];
        x(4) = velocities[0];
        x(5) = velocities[1];
        x(6) = velocities[2];

        filter.init(x, P0);
}

void PVFilter::measure(const vector<double>& positions){
  Vector z(3);
  z(1) = positions[0];
  z(2) = positions[1];
  z(3) = positions[2];
  filter.measureUpdateStep(z);
}

/* static */ const PVPair PVFilter::toPVPair(const Vector& x){
  PVPair results;
  results.first.resize(3);
  results.first[0] = x(1);
  results.first[1] = x(2);
  results.first[2] = x(3);

  results.second.resize(3);
  results.second[0] = x(4);
  results.second[1] = x(5);
  results.second[2] = x(6);
  return results;
}

const PVPair PVFilter::predict(){
  Vector u(0);
  filter.timeUpdateStep(u);
  return toPVPair(filter.getX());
}

const PVPair PVFilter::getX() const {
  return toPVPair(filter.getX());  
}
