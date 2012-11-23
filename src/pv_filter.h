#ifndef PV_FILTER_H
#define PV_FILTER_H

#include "kalman_simple.h"
#include <vector>
#include <utility>

typedef std::pair<std::vector<double>, std::vector<double> > PVPair;

class PVFilter {

public:
  PVFilter();
  void init(const std::vector<double>& positions, const std::vector<double>& velocities);  
  void measure(const std::vector<double>& positions);
  const PVPair predict();
  void setDT(const double DT);
  const PVPair getX() const;
  static const PVPair toPVPair(const Vector& x);
private:
  KalmanSimple filter;
};
#endif
