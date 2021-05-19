// SHREYASH SHARMA
// Branch:CSE
// 6th Semester B.E.
// shreyneil@yahoo.co.in
//(Reg no.140905304)
// MIT MANIPAL
// sources
// http://docs.opencv.org/2.4/doc/tutorials/tutorials.html
// http://docs.opencv.org/2.4/modules/refman.html

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
// using namespace cv;
using namespace std;
/// Global variables

cv::Mat src, src_gray;
cv::Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold;
int ratio = 3;
int kernel_size = 3;
const char* window_name = "Jelly Fish";

/**
 * @function CannyThreshold
 * Detects the edges of the jellyfish image and then utilizing their contours, detects the centroid
 * of the jellyfish. Marks them with a cross built from two lines
 */
void
CannyDetect() {
  std::vector<std::vector<cv::Point>> contours;
  // Reduce noise with a kernel for better detection
  cv::blur(src_gray, detected_edges, cv::Size(3, 3));
  std::vector<cv::Vec4i> hierarchy;

  // Running the cv::Canny detector
  cv::Canny(detected_edges, detected_edges, lowThreshold, lowThreshold * ratio, kernel_size);
  // finding contours
  cv::findContours(detected_edges, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
  // converting image from grayscale to coloured
  cv::cvtColor(detected_edges, detected_edges, cv::COLOR_GRAY2BGR);

  // cv::add the detected edges on top of the image with partial visibility
  addWeighted(src, 1.0, detected_edges, 0.5, 0.0, dst);

  // Getting centres fo each contour using cv::moments and centres

  // Get the moments
  std::vector<cv::Moments> mu(contours.size());
  for(int i = 0; i < contours.size(); i++) { mu[i] = cv::moments(contours[i], false); }

  ///  Get the centroids using moments
  std::vector<cv::Point> mc(contours.size());
  for(int i = 0; i < contours.size(); i++) { mc[i] = cv::Point(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00); }

  // making the red cross using lines
  for(int i = 0; i < mc.size(); i++) {

    cv::line(dst, cv::Point(mc[i].x - 6, mc[i].y), cv::Point(mc[i].x + 6, mc[i].y), cv::Scalar(0, 0, 255), 1, 8, 0);
    cv::line(dst, cv::Point(mc[i].x, mc[i].y - 6), cv::Point(mc[i].x, mc[i].y + 6), cv::Scalar(0, 0, 255), 1, 8, 0);
  }

  cv::imshow(window_name, dst);
}

/** @function main */
int
main(int argc, char** argv) {
  // Load the image
  src = cv::imread("../jellyfish.jpg");

  if(!src.data) {
    return -1;
  }

  /// Create a matrix of the same type and size as src (for dst)
  dst.create(src.size(), src.type());

  /// Convert the image to grayscale
  cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);

  /// Create a window
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

  // The cv::threshold of detecting edges of the jellyfish
  lowThreshold = 90;
  // do a canny detection of the edges
  CannyDetect();
  cv::waitKey(0);

  return 0;
}
