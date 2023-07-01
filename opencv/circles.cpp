// I dont know what to call this file

#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

// using namespace cv;
using namespace std;

int
main() {
  cv::Mat src, gray;

  cv::VideoCapture cam(0);

  if(!cam.isOpened()) {
    cout << "unable to open capture";
    return -1;
  }

  cv::namedWindow("Hough Circle Transform Demo");

  while(true) {
    cam >> src;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

    // Reduce the noise so we avoid false cv::circle detection
    cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);
    cv::Canny(gray, gray, 0, 50, 3);
    cv::imshow("edges", gray);

    vector<cv::Vec3f> circles;

    // Apply the Hough Transform to find the circles
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0);

    // Draw the circles detected
    for(size_t i = 0; i < circles.size(); i++) {
      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      cv::circle(src, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);     // cv::circle center
      cv::circle(src, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0); // cv::circle outline
      cout << "center : " << center << "\nradius : " << radius << endl;
    }

    // Show your results
    cv::imshow("Hough Circle Transform Demo", src);

    if(cv::waitKey(30) == 27)
      break; // escape key
  }
  return 0;
}
