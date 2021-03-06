#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
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
  cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
  blur(src_gray, src_gray, Size(3, 3));

  /// Create Window
  char* source_window = "Source";
  namedWindow(source_window, cv::WINDOW_AUTOSIZE);
  imshow(source_window, src);

  createTrackbar(" Threshold:", "Source", &thresh, max_thresh, thresh_callback);
  thresh_callback(0, 0);

  waitKey(0);
  return (0);
}

/** @function thresh_callback */
void
thresh_callback(int, void*) {
  cv::Mat threshold_output;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<Vec4i> hierarchy;

  /// Detect edges using Threshold
  threshold(src_gray, threshold_output, thresh, 255, THRESH_BINARY);
  /// Find contours
  findContours(threshold_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  /// Approximate contours to polygons + get bounding rects and circles
  std::vector<std::vector<cv::Point>> contours_poly(contours.size());
  std::vector<Rect> boundRect(contours.size());
  std::vector<cv::Point2f> center(contours.size());
  std::vector<float> radius(contours.size());

  for(int i = 0; i < contours.size(); i++) {
    approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
    boundRect[i] = boundingRect(cv::Mat(contours_poly[i]));
    minEnclosingCircle((cv::Mat)contours_poly[i], center[i], radius[i]);
  }

  /// Draw polygonal contour + bonding rects + circles
  cv::Mat drawing = cv::Mat::zeros(threshold_output.size(), CV_8UC3);
  for(int i = 0; i < contours.size(); i++) {
    Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    drawContours(drawing, contours_poly, i, color, 1, 8, std::vector<Vec4i>(), 0, cv::Point());
    rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
    circle(drawing, center[i], (int)radius[i], color, 2, 8, 0);
  }

  /// Show in a window
  namedWindow("Contours", cv::WINDOW_AUTOSIZE);
  imshow("Contours", drawing);
}
