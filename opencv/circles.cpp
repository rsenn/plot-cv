// I dont know what to call this file

#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

int
main() {
  Mat src, gray;

  VideoCapture cam(0);

  if(!cam.isOpened()) {
    cout << "unable to open capture";
    return -1;
  }

  namedWindow("Hough Circle Transform Demo");

  while(true) {
    cam >> src;
    cvtColor(src, gray, cv::COLOR_BGR2GRAY);

    // Reduce the noise so we avoid false circle detection
    GaussianBlur(gray, gray, Size(9, 9), 2, 2);
    Canny(gray, gray, 0, 50, 3);
    imshow("edges", gray);

    vector<Vec3f> circles;

    // Apply the Hough Transform to find the circles
    HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, 30, 200, 50, 0, 0);

    // Draw the circles detected
    for(size_t i = 0; i < circles.size(); i++) {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      circle(src, center, 3, Scalar(0, 255, 0), -1, 8, 0);     // circle center
      circle(src, center, radius, Scalar(0, 0, 255), 3, 8, 0); // circle outline
      cout << "center : " << center << "\nradius : " << radius << endl;
    }

    // Show your results
    imshow("Hough Circle Transform Demo", src);

    if(waitKey(30) == 27)
      break; // escape key
  }
  return 0;
}
