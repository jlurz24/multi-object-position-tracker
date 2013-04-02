#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include "find_track.h"

namespace position_tracker {

typedef std::vector<cv::Point> Contour;

// returns the track detected on the image.
const Contour findTrack(const cv::Mat& src){

    cv::Mat tgrayMat;
    IplImage timgIpl = src;
 
    // Extract the blue channel
    cv::extractImageCOI(&timgIpl, tgrayMat, 2 /* B color channel */);
      
    // Reduce noise with a kernel 3x3
    cv::Mat cleanedImage;
    cv::blur(tgrayMat, cleanedImage, cv::Size(3,3));  
 
    cv::Mat grayMat;
    cv::Canny(cleanedImage, cleanedImage, 100 /* Lower threshold */, 300 /* Upper threshold */, 3 /* Kernel size */);
    
    // dilate canny output to remove potential holes between edge segments
    cv::dilate(cleanedImage, cleanedImage, cv::Mat());

    // find contours and store them all as a list
    std::vector<Contour> contours;
    cv::findContours(cleanedImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    // TEMP
    cv::Mat cpy = src.clone();

    // draw the contour as a closed polyline.
    cv::drawContours(cpy, contours, -1, CV_RGB(0, 255, 0), 3, CV_AA);

    // show the resultant image
    cv::imshow("Contours", cpy);
    cv::waitKey(0);

    // Find the largest contour and assume it's the outer contour.
    Contour outerContour;
    for(unsigned int k = 0; k < contours.size(); ++k){
      if(contours[k].size() > outerContour.size()){
        outerContour = contours[k];
      }
      std::cout << contours[k].size() << std::endl;
    }
      
    Contour result;
    cv::approxPolyDP(cv::Mat(outerContour), result, 0.05, true);
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

