#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

namespace position_tracker {

  // returns the track detected in the image.
  const std::vector<cv::Point> findTrack(const cv::Mat& img);

  // the function draws the track in the image
  void drawTrack(const cv::Mat& img, const std::vector<cv::Point>& track, const std::string& windowName);

  void showTrackInImage(const cv::Mat& img);
  void showTrackInFile(const std::string& fileName);
};
