#include "stdafx.h"
#include "VisionCore.h"
#include <iostream>

using namespace std;

int
main() {
  cv::VideoCapture cap(0);
  cap.set(cv::CAP_PROP_FPS, 60);

  int lowerH = 17, lowerS = 157, lowerV = 132, upperH = 35, upperS = 255, upperV = 255;
  float focalLength = 505, FOV = 76.6, targetWidth = 3;
  //                       In Degrees

  cv::Mat frame;
  VisionCore vcore(focalLength);

  while(true) {
    cv::createTrackbar("upperH", "dst", &upperH, 255);
    cv::createTrackbar("upperS", "dst", &upperS, 255);
    cv::createTrackbar("upperV", "dst", &upperV, 255);

    cv::createTrackbar("lowerH", "dst", &lowerH, 255);
    cv::createTrackbar("lowerS", "dst", &lowerS, 255);
    cv::createTrackbar("lowerV", "dst", &lowerV, 255);

    vcore.setBounds(cv::Scalar(lowerH, lowerS, lowerV), cv::Scalar(upperH, upperS, upperV));
    cap >> frame;
    vcore.VisionCore::DetectObjects(frame);

    cv::waitKey(1);
  }

  cap.release();
  return 0;
}