#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

cv::Mat src;
cv::Mat src_gray;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

/// Function header
void thresh_callback(int, void*);

/** @function main */
int
main(int argc, char** argv) {
  /// Load source image and convert it to gray
  src = imread(argv[1], 1);

  /// Convert image to gray and blur it
  cvtColor(src, src_gray, COLOR_BGR2GRAY);
  blur(src_gray, src_gray, Size(3, 3));

  /// Create Window
  char* source_window = "Source";
  namedWindow(source_window, WINDOW_AUTOSIZE);
  imshow(source_window, src);

  createTrackbar(" Canny thresh:", "Source", &thresh, max_thresh, thresh_callback);
  thresh_callback(0, 0);

  waitKey(0);
  return (0);
}

/** @function thresh_callback */
void
thresh_callback(int, void*) {
  cv::Mat canny_output;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<Vec4i> hierarchy;

  /// Detect edges using canny
  Canny(src_gray, canny_output, thresh, thresh * 2, 3);
  /// Find contours
  findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  /// Draw contours
  cv::Mat drawing = cv::Mat::zeros(canny_output.size(), CV_8UC3);
  for(int i = 0; i < contours.size(); i++) {
    Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
  }

  /// Show in a window
  namedWindow("Contours", WINDOW_AUTOSIZE);
  imshow("Contours", drawing);
}
