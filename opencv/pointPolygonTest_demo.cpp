/**
 * @function pointPolygonTest_demo.cpp
 * @brief Demo code to use the cv::pointPolygonTest function...fairly easy
 * @author OpenCV team
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

// using namespace cv;
using namespace std;

/**
 * @function main
 */
int
main(void) {
  /// Create an image
  const int r = 100;
  cv::Mat src = cv::Mat::zeros(cv::Size(4 * r, 4 * r), CV_8UC1);

  /// Create a sequence of points to make a contour:
  std::vector<cv::Point2f> vert(6);

  vert[0] = cv::Point(3 * r / 2, static_cast<int>(1.34 * r));
  vert[1] = cv::Point(1 * r, 2 * r);
  vert[2] = cv::Point(3 * r / 2, static_cast<int>(2.866 * r));
  vert[3] = cv::Point(5 * r / 2, static_cast<int>(2.866 * r));
  vert[4] = cv::Point(3 * r, 2 * r);
  vert[5] = cv::Point(5 * r / 2, static_cast<int>(1.34 * r));

  /// Draw cv::it in src
  for(int j = 0; j < 6; j++) { cv::line(src, vert[j], vert[(j + 1) % 6], cv::Scalar(255), 3, 8); }

  /// Get the contours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat src_copy = src.clone();

  cv::findContours(src_copy, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  /// Calculate the distances to the contour
  cv::Mat raw_dist(src.size(), CV_32FC1);

  for(int j = 0; j < src.rows; j++) {
    for(int i = 0; i < src.cols; i++) {
      raw_dist.at<float>(j, i) = (float)cv::pointPolygonTest(contours[0], cv::Point2f((float)i, (float)j), true);
    }
  }

  double minVal;
  double maxVal;
  cv::minMaxLoc(raw_dist, &minVal, &maxVal, 0, 0, cv::Mat());
  minVal = abs(minVal);
  maxVal = abs(maxVal);

  /// Depicting the  distances graphically
  cv::Mat drawing = cv::Mat::zeros(src.size(), CV_8UC3);

  for(int j = 0; j < src.rows; j++) {
    for(int i = 0; i < src.cols; i++) {
      if(raw_dist.at<float>(j, i) < 0) {
        drawing.at<cv::Vec3b>(j, i)[0] = (uchar)(255 - abs(raw_dist.at<float>(j, i)) * 255 / minVal);
      } else if(raw_dist.at<float>(j, i) > 0) {
        drawing.at<cv::Vec3b>(j, i)[2] = (uchar)(255 - raw_dist.at<float>(j, i) * 255 / maxVal);
      } else {
        drawing.at<cv::Vec3b>(j, i)[0] = 255;
        drawing.at<cv::Vec3b>(j, i)[1] = 255;
        drawing.at<cv::Vec3b>(j, i)[2] = 255;
      }
    }
  }

  /// Create Window and show your results
  const char* source_window = "Source";
  cv::namedWindow(source_window, cv::WINDOW_AUTOSIZE);
  cv::imshow(source_window, src);
  cv::namedWindow("Distance", cv::WINDOW_AUTOSIZE);
  cv::imshow("Distance", drawing);

  cv::waitKey(0);
  return (0);
}
