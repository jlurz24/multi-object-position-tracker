#include "pv_filter.h"
#include <iostream>

namespace {
using namespace std;

static unsigned int uniqueId = 0;

static inline double square(const double x) {
    return x * x;
}

}

PVFilter::PVFilter(const double aObservationNoise, const double aVelocityNoise) :
        filter(new KalmanSimple(aObservationNoise, aVelocityNoise)), id(uniqueId++) {
}

void PVFilter::init(const vector<double>& positions, const vector<double>& velocities,
        ros::Time time) {

    updateTime = time;

    // Number of variables
    const int n = 6;

    // Filter's Initial state uncertainty: Position is approximately
    // known but velocity is unknown.
    const double i_P_NOISE = 1.;
    const double i_V_NOISE = 10.;

    // Initial covariance matrix
    static const double _P0[] = { square(i_P_NOISE), 0., 0., 0., 0., 0., 0., square(i_P_NOISE), 0.,
            0., 0., 0., 0., 0., square(i_P_NOISE), 0., 0., 0., 0., 0., 0., square(i_V_NOISE), 0.,
            0., 0., 0., 0., 0., square(i_V_NOISE), 0., 0., 0., 0., 0., 0., square(i_V_NOISE) };
    Matrix P0(n, n, _P0);

    // Initial Estimate
    Vector x(n);
    x(1) = positions[0];
    x(2) = positions[1];
    x(3) = positions[2];
    x(4) = velocities[0];
    x(5) = velocities[1];
    x(6) = velocities[2];

    filter->init(x, P0);
}

void PVFilter::measure(const vector<double>& positions, const ros::Time& time) {
    Vector z(3);
    z(1) = positions[0];
    z(2) = positions[1];
    z(3) = positions[2];
    filter->setDT(time.toSec() - updateTime.toSec());
    filter->measureUpdateStep(z);
    updateTime = time;
}

void PVFilter::predict(vector<double>& positions, const ros::Time& time) {
    Vector u(0);
    filter->setDT(time.toSec() - updateTime.toSec());
    filter->timeUpdateStep(u);

    const Vector x = filter->getX();
    positions.resize(3);
    positions[0] = x(1);
    positions[1] = x(2);
    positions[2] = x(3);
}

void PVFilter::getX(vector<double>& positions, vector<double>& velocities) const {
    const Vector x = filter->getX();
    positions.resize(3);
    positions[0] = x(1);
    positions[1] = x(2);
    positions[2] = x(3);

    velocities.resize(3);
    velocities[0] = x(4);
    velocities[1] = x(5);
    velocities[2] = x(6);
}

