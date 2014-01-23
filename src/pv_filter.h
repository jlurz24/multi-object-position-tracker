#ifndef PV_FILTER_H
#define PV_FILTER_H

#include <vector>
#include <utility>
#include <ros/ros.h>

namespace cv {
    class KalmanFilter;
}

class PVFilter {

public:
    PVFilter(const double aObservationNoise, const double aVelocityNoise);
    void init(const std::vector<double>& positions, const std::vector<double>& velocities,
            const ros::Time time);
    void measure(const std::vector<double>& positions, const ros::Time& time);
    void predict(std::vector<double>& positions, const ros::Time& time);
    void getX(std::vector<double>& positions, std::vector<double>& velocities) const;
    const ros::Time& getLastUpdate() const {
        return updateTime;
    }
    unsigned int getId() const {
        return id;
    }
private:
    std::auto_ptr<cv::KalmanFilter> filter;
    ros::Time updateTime;
    double mVelocityNoise;
    double mObservationNoise;
    unsigned int id;

    void updateTransitionMatrix(double dt);
};
#endif
