#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

// using namespace cv;
using namespace std;

cv::Mat src, src_gray;

int thresh = 100;
int max_thresh = 255;
cv::RNG rng(12345);

/// Function header
void threshold_callback(int, void*);
// void canny_callback( int, void* );

/** @function main */
int
main(int argc, char* argv[]) {
  /// Load source image
  src = cv::imread(argv[1], 1);

  if(!src.data) {
    cerr << "Problem loading image!!!" << endl;
    return EXIT_FAILURE;
  }

  /// Convert image to gray and cv::blur it
  cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
  cv::blur(src_gray, src_gray, cv::Size(3, 3));
  cv::adaptiveThreshold(src_gray, src_gray, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 75, 10);
  cv::Mat kernel = cv::Mat::ones(2, 2, CV_8UC1);
  cv::dilate(src_gray, src_gray, kernel);
  cv::bitwise_not(src_gray, src_gray);

  /// Create Window
  char* source_window = "Source";
  cv::namedWindow(source_window, cv::WINDOW_AUTOSIZE);
  cv::imshow(source_window, src_gray);

  cv::createTrackbar(" Threshold:", "Source", &thresh, max_thresh, threshold_callback);

  threshold_callback(0, 0);

  cv::waitKey(0);
  return (0);
}

/** @function threshold_callback */
void
threshold_callback(int, void*) {
  cv::Mat threshold_output;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;

  /// Detect edges using Threshold
  cv::threshold(src_gray, threshold_output, thresh, 255, cv::THRESH_BINARY);
  /// Find contours
  cv::findContours(threshold_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  /// Approximate contours to polygons + get bounding rects and circles
  // vector<cv::Point2f> center( contours.size() );
  vector<cv::Rect> boundRect(contours.size());
  // vector<float> radius( contours.size() );
  vector<vector<cv::Point>> contours_poly(contours.size());

  for(int i = 0; i < contours.size(); i++) {
    cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
    boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
    // cv::minEnclosingCircle( (cv::Mat)contours_poly[i], center[i], radius[i] );
  }

  /// Draw polygonal contour + bonding rects + circles
  cv::Mat drawing = cv::Mat::zeros(threshold_output.size(), CV_8UC3);
  for(int i = 0; i < contours.size(); i++) {

    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    // cv::drawContours( drawing, contours_poly, i, color, 1, 8, vector<cv::Vec4i>(), 0, cv::Point() );
    cv::rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
    // cv::circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
    // printf("x : %d / y : %d\n", boundRect[i].tl().x, boundRect[i].br().y);
  }

  /// Show in a window
  cv::namedWindow("Contours", cv::WINDOW_AUTOSIZE);
  cv::imshow("Contours", drawing);
}
