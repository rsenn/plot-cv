/**
 * @function goodFeaturesToTrack_Demo.cpp
 * @brief Demo code for detecting corners using Shi-Tomasi method
 * @author OpenCV team
 */

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

// using namespace cv;
using namespace std;

/// Global variables
cv::Mat src, src_gray;

int maxCorners = 23;
int maxTrackbar = 100;

cv::RNG rng(12345);
const char* source_window = "Image";

/// Function header
void goodFeaturesToTrack_Demo(int, void*);

/**
 * @function main
 */
int
main(int argc, char** argv) {
  /// Load source image and convert it to gray
  cv::CommandLineParser parser(argc, argv, "{@input | pic3.png | input image}");
  src = cv::imread(cv::samples::findFile(parser.get<cv::String>("@input")));
  if(src.empty()) {
    cout << "Could not open or find the image!\n" << endl;
    cout << "Usage: " << argv[0] << " <Input image>" << endl;
    return -1;
  }
  cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);

  /// Create Window
  cv::namedWindow(source_window);

  /// Create Trackbar to set the number of corners
  cv::createTrackbar("Max corners:", source_window, &maxCorners, maxTrackbar, goodFeaturesToTrack_Demo);

  cv::imshow(source_window, src);

  goodFeaturesToTrack_Demo(0, 0);

  cv::waitKey();
  return 0;
}

/**
 * @function goodFeaturesToTrack_Demo.cpp
 * @brief Apply Shi-Tomasi corner detector
 */
void
goodFeaturesToTrack_Demo(int, void*) {
  /// Parameters for Shi-Tomasi algorithm
  maxCorners = MAX(maxCorners, 1);
  vector<cv::Point2f> corners;
  double qualityLevel = 0.01;
  double minDistance = 10;
  int blockSize = 3, gradientSize = 3;
  bool useHarrisDetector = false;
  double k = 0.04;

  /// Copy the source image
  cv::Mat copy = src.clone();

  /// Apply corner detection
  cv::goodFeaturesToTrack(
      src_gray, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, gradientSize, useHarrisDetector, k);

  /// Draw corners detected
  cout << "** Number of corners detected: " << corners.size() << endl;
  int radius = 4;
  for(size_t i = 0; i < corners.size(); i++) {
    cv::circle(copy, corners[i], radius, cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 256), rng.uniform(0, 256)), cv::FILLED);
  }

  /// Show what you got
  cv::namedWindow(source_window);
  cv::imshow(source_window, copy);
}
