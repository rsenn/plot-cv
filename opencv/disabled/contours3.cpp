#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include "highgui.h"
#include <iostream>
#include <vector>

// Get thresholded image in HSV format
IplImage*
GetThresholdedImageHSV(IplImage* img) {
  // Create an HSV cv::format image from image passed
  IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);

  cvCvtColor(img, imgHSV, cv::COLOR_BGR2HSV);

  // Create binary thresholded image acc. to max/min HSV ranges
  // For detecting blue gloves in "MOV.MPG - HSV mode
  IplImage* imgThresh = cvCreateImage(cvGetSize(img), 8, 1);

  cvInRangeS(imgHSV, cvScalar(104, 178, 70), cvScalar(130, 240, 124), imgThresh);

  cvReleaseImage(&imgHSV);
  return imgThresh;
}

int
main() {
  CvCapture* capture = cvCaptureFromFile("MOV.MPG");
  IplImage* frame = cvQueryFrame(capture);

  // Can't get device? Complain and quit
  if(!capture) {
    std::cout << "Could not initialize capturing...\n";
    return -1;
  }

  while(true) {
    frame = cvQueryFrame(capture);

    if(!frame)
      break;

    IplImage* imgThresh = GetThresholdedImageHSV(frame);
    cv::Mat imageThreshcv::Mat = cv::Mat(imgThresh, false);
    cv::Mat framecv::Mat = cv::Mat(frame, false);

    // Find the contours.
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat contourOutput = imageThreshcv::Mat.clone();
    cv::findContours(contourOutput, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    // Draw the contours
    cv::Mat contourImage = framecv::Mat.clone();
    for(size_t idx = 0; idx < contours.size(); idx++) { cv::drawContours(contourImage, contours, idx, cv::Scalar(255, 0, 0)); }

    cv::imshow("video", contourImage);
    cvWaitKey(40);
  }

  // We're through with using camera.
  cvReleaseCapture(&capture);

  return 0;
}
