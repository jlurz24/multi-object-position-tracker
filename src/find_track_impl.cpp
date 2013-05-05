#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include "find_track_impl.h"

namespace position_tracker {

typedef std::vector<cv::Point> Contour;

// returns the track detected on the image.
const Contour findTrack(const cv::Mat& src){
 
    // Convert to HSV which is more stable.
    cv::Mat hsvImage;
    cv::cvtColor(src, hsvImage, CV_BGR2HSV);
    
    cv::Mat imgThreshed;
    cv::inRange(hsvImage, cv::Scalar(90, 40, 0), cv::Scalar(140, 228, 255), imgThreshed);
    
    cv::erode(imgThreshed, imgThreshed, cv::Mat());
    cv::dilate(imgThreshed, imgThreshed, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)));

    std::vector<Contour> contours;
    cv::findContours(imgThreshed, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    // Find the largest contour and assume it's the outer contour.
    Contour outerContour;
    for(unsigned int k = 0; k < contours.size(); ++k){
      if(contours[k].size() > outerContour.size()){
        outerContour = contours[k];
      }
    }


    Contour result;
    cv::approxPolyDP(cv::Mat(outerContour), result, 1.0, true);
    return result;
}


// the function draws the track in the image
void drawTrack(const cv::Mat& img, const Contour& track, const std::string& windowName){

    cv::Mat cpy = img.clone();

    std::vector<Contour> contours;
    contours.push_back(track);

    // draw the contour as a closed polyline.
    cv::drawContours(cpy, contours, -1, CV_RGB(0, 255, 0), 3, CV_AA);

    // show the resultant image
    cv::imshow(windowName, cpy);
}

void showTrackInFile(const std::string& fileName){

  cv::Mat img = cv::imread(fileName);
  showTrackInImage(img);
}

void showTrackInImage(const cv::Mat& img){
  static const std::string windowName = "Track";
  cv::namedWindow(windowName, 1);

  // find and draw the track
  const Contour track = findTrack(img);
  drawTrack(img, track, windowName);

  // cv::waitKey takes care of event processing
  cv::waitKey(0);
}
}

