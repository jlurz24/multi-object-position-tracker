#ifndef PV_FILTER_H
#define PV_FILTER_H

#include "kalman_simple.h"
#include <vector>
#include <utility>

typedef std::pair<std::vector<double>, std::vector<double> > PVPair;

class PVFilter {

public:
  PVFilter(const double aObservationNoise, const double aVelocityNoise);
  void init(const std::vector<double>& positions, const std::vector<double>& velocities);  
  void measure(const std::vector<double>& positions);
  void predict(std::vector<double>& positions);
  void setDT(const double DT);
  void getX(std::vector<double>& positions, std::vector<double>& velocities) const;
private:
  KalmanSimple filter;
};
#endif
