#ifndef PV_FILTER_H
#define PV_FILTER_H

#include "kalman_simple.h"
#include <vector>
#include <utility>
#include <ros/ros.h>

typedef std::pair<std::vector<double>, std::vector<double> > PVPair;

class PVFilter {

public:
  PVFilter(const double aObservationNoise, const double aVelocityNoise);
  void init(const std::vector<double>& positions, const std::vector<double>& velocities, const ros::Time time);  
  void measure(const std::vector<double>& positions, ros::Time time);
  void predict(std::vector<double>& positions, ros::Time time);
  void getX(std::vector<double>& positions, std::vector<double>& velocities) const;
  ros::Time getLastUpdate() const { return updateTime; }
private:
  KalmanSimple filter;
  ros::Time updateTime;
};
#endif
