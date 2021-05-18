/**
 * @file HoughLines_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

//using namespace cv;
using namespace std;

/// Global variables

/** General variables */
cv::Mat src, edges;
cv::Mat src_gray;
cv::Mat standard_hough, probabilistic_hough;
int min_threshold = 50;
int max_trackbar = 150;

const char* standard_name = "Standard Hough Lines Demo";
const char* probabilistic_name = "Probabilistic Hough Lines Demo";

int s_trackbar = max_trackbar;
int p_trackbar = max_trackbar;

/// Function Headers
void help();
void Standard_Hough(int, void*);
void Probabilistic_Hough(int, void*);

/**
 * @function main
 */
int
main(int argc, char** argv) {
  // Read the image
  cv::String imageName("building.jpg"); // by default
  if(argc > 1) {
    imageName = argv[1];
  }
  src = cv::imread(cv::samples::findFile(imageName), cv::IMREAD_COLOR);

  if(src.empty()) {
    help();
    return -1;
  }

  /// Pass the image to gray
  cv::cvtColor(src, src_gray, cv::COLOR_RGB2GRAY);

  /// Apply cv::Canny edge detector
  cv::Canny(src_gray, edges, 50, 200, 3);

  /// Create Trackbars for Thresholds
  char thresh_label[50];
  sprintf(thresh_label, "Thres: %d + input", min_threshold);

  cv::namedWindow(standard_name, cv::WINDOW_AUTOSIZE);
  cv::createTrackbar(thresh_label, standard_name, &s_trackbar, max_trackbar, Standard_Hough);

  cv::namedWindow(probabilistic_name, cv::WINDOW_AUTOSIZE);
  cv::createTrackbar(thresh_label, probabilistic_name, &p_trackbar, max_trackbar, Probabilistic_Hough);

  /// Initialize
  Standard_Hough(0, 0);
  Probabilistic_Hough(0, 0);
  cv::waitKey(0);
  return 0;
}

/**
 * @function help
 * @brief Indications of how to run this program and why is cv::it for
 */
void
help() {
  printf("\t Hough Transform to detect lines \n ");
  printf("\t---------------------------------\n ");
  printf(" Usage: ./HoughLines_Demo <image_name> \n");
}

/**
 * @function Standard_Hough
 */
void
Standard_Hough(int, void*) {
  vector<cv::Vec2f> s_lines;
  cv::cvtColor(edges, standard_hough, cv::COLOR_GRAY2BGR);

  /// 1. Use Standard Hough Transform
  cv::HoughLines(edges, s_lines, 1, CV_PI / 180, min_threshold + s_trackbar, 0, 0);

  /// Show the result
  for(size_t i = 0; i < s_lines.size(); i++) {
    float r = s_lines[i][0], t = s_lines[i][1];
    double cos_t = cos(t), sin_t = sin(t);
    double x0 = r * cos_t, y0 = r * sin_t;
    double alpha = 1000;

    cv::Point pt1(cvRound(x0 + alpha * (-sin_t)), cvRound(y0 + alpha * cos_t));
    cv::Point pt2(cvRound(x0 - alpha * (-sin_t)), cvRound(y0 - alpha * cos_t));
    cv::line(standard_hough, pt1, pt2, cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
  }

  cv::imshow(standard_name, standard_hough);
}

/**
 * @function Probabilistic_Hough
 */
void
Probabilistic_Hough(int, void*) {
  vector<cv::Vec4i> p_lines;
  cv::cvtColor(edges, probabilistic_hough, cv::COLOR_GRAY2BGR);

  /// 2. Use Probabilistic Hough Transform
  cv::HoughLinesP(edges, p_lines, 1, CV_PI / 180, min_threshold + p_trackbar, 30, 10);

  /// Show the result
  for(size_t i = 0; i < p_lines.size(); i++) {
    cv::Vec4i l = p_lines[i];
    cv::line(probabilistic_hough, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
  }

  cv::imshow(probabilistic_name, probabilistic_hough);
}
