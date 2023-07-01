/**
 * @function generalContours_demo2.cpp
 * @brief Demo code to obtain ellipses and rotated rectangles that contain detected contours
 * @author OpenCV team
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

// using namespace cv;
using namespace std;

cv::Mat src;
cv::Mat src_gray;
int thresh = 100;
int max_thresh = 255;
cv::RNG rng(12345);

/// Function header
void thresh_callback(int, void*);

/**
 * @function main
 */
int
main(int, char** argv) {
  /// Load source image and convert it to gray
  src = cv::imread(argv[1], 1);

  /// Convert image to gray and cv::blur it
  cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
  cv::blur(src_gray, src_gray, cv::Size(3, 3));

  /// Create Window
  const char* source_window = "Source";
  cv::namedWindow(source_window, cv::WINDOW_AUTOSIZE);
  cv::imshow(source_window, src);

  cv::createTrackbar(" Threshold:", "Source", &thresh, max_thresh, thresh_callback);
  thresh_callback(0, 0);

  cv::waitKey(0);
  return (0);
}

/**
 * @function thresh_callback
 */
void
thresh_callback(int, void*) {
  cv::Mat threshold_output;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  /// Detect edges using Threshold
  cv::threshold(src_gray, threshold_output, thresh, 255, cv::THRESH_BINARY);
  /// Find contours
  cv::findContours(threshold_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  /// Find the rotated rectangles and ellipses for each contour
  std::vector<cv::RotatedRect> minRect(contours.size());
  std::vector<cv::RotatedRect> minEllipse(contours.size());

  for(size_t i = 0; i < contours.size(); i++) {
    minRect[i] = cv::minAreaRect(cv::Mat(contours[i]));
    if(contours[i].size() > 5) {
      minEllipse[i] = cv::fitEllipse(cv::Mat(contours[i]));
    }
  }

  /// Draw contours + rotated rects + ellipses
  cv::Mat drawing = cv::Mat::zeros(threshold_output.size(), CV_8UC3);
  for(size_t i = 0; i < contours.size(); i++) {
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    // contour
    cv::drawContours(drawing, contours, (int)i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    // ellipse
    cv::ellipse(drawing, minEllipse[i], color, 2, 8);
    // rotated rectangle
    cv::Point2f rect_points[4];
    minRect[i].points(rect_points);
    for(int j = 0; j < 4; j++) cv::line(drawing, rect_points[j], rect_points[(j + 1) % 4], color, 1, 8);
  }

  /// Show in a window
  cv::namedWindow("Contours", cv::WINDOW_AUTOSIZE);
  cv::imshow("Contours", drawing);
}
