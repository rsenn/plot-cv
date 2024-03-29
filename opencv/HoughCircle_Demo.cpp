/**
 * @file HoughCircle_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
// using namespace cv;

namespace {
// windows and trackbars name
const std::string windowName = "Hough Circle Detection Demo";
const std::string cannyThresholdTrackbarName = "cv::Canny cv::threshold";
const std::string accumulatorThresholdTrackbarName = "Accumulator Threshold";

// initial and max values of the parameters of interests.
const int cannyThresholdInitialValue = 100;
const int accumulatorThresholdInitialValue = 50;
const int maxAccumulatorThreshold = 200;
const int maxCannyThreshold = 255;

void
HoughDetection(const cv::Mat& src_gray, const cv::Mat& src_display, int cannyThreshold, int accumulatorThreshold) {
  // will hold the results of the detection
  std::vector<cv::Vec3f> circles;
  // runs the actual detection
  cv::HoughCircles(src_gray, circles, HOUGH_GRADIENT, 1, src_gray.rows / 8, cannyThreshold, accumulatorThreshold, 0, 0);

  // clone the colour, input image for displaying purposes
  cv::Mat display = src_display.clone();
  for(size_t i = 0; i < circles.size(); i++) {
    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);
    // cv::circle center
    cv::circle(display, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
    // cv::circle outline
    cv::circle(display, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
  }

  // shows the results
  cv::imshow(windowName, display);
}
} // namespace

int
main(int argc, char** argv) {
  cv::Mat src, src_gray;

  // Read the image
  cv::String imageName("stuff.jpg"); // by default
  if(argc > 1) {
    imageName = argv[1];
  }
  src = cv::imread(cv::samples::findFile(imageName), cv::IMREAD_COLOR);

  if(src.empty()) {
    std::cerr << "Invalid input image\n";
    std::cout << "Usage : " << argv[0] << " <path_to_input_image>\n";
    ;
    return -1;
  }

  // Convert cv::it to gray
  cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);

  // Reduce the noise so we avoid false cv::circle detection
  cv::GaussianBlur(src_gray, src_gray, cv::Size(9, 9), 2, 2);

  // declare and initialize both parameters that are subjects to change
  int cannyThreshold = cannyThresholdInitialValue;
  int accumulatorThreshold = accumulatorThresholdInitialValue;

  // create the main window, and attach the trackbars
  cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
  cv::createTrackbar(cannyThresholdTrackbarName, windowName, &cannyThreshold, maxCannyThreshold);
  cv::createTrackbar(accumulatorThresholdTrackbarName, windowName, &accumulatorThreshold, maxAccumulatorThreshold);

  // infinite loop to display
  // and refresh the content of the output image
  // until the user presses q or Q
  char key = 0;
  while(key != 'q' && key != 'Q') {
    // those parameters cannot be =0
    // so we must check here
    cannyThreshold = std::max(cannyThreshold, 1);
    accumulatorThreshold = std::max(accumulatorThreshold, 1);

    // runs the detection, and update the display
    HoughDetection(src_gray, src, cannyThreshold, accumulatorThreshold);

    // get user key
    key = (char)cv::waitKey(10);
  }

  return 0;
}
