#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <stdio.h>

using namespace cv;

/**
 * Helper function to display text in the center of a contour
 */
void
setLabel(Mat& im, const std::string label, std::vector<Point>& contour) {
  double scale = 0.4;
  int baseline = 0;
  int fontface = FONT_HERSHEY_SIMPLEX;
  int thickness = 1;

  Size text = getTextSize(label, fontface, scale, thickness, &baseline);
  Rect r = boundingRect(contour);

  Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
  rectangle(im, pt + Point(0, baseline), pt + Point(text.width, -text.height), CV_RGB(255, 255, 255), cv::FILLED);
  putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

int
main(int argc, char* argv[]) {
  Mat img_src, img_gray, img_bin, img_dest;
  Rect roi(0, 0, 150, 150);

  // Read source image
  img_src = imread(argv[1], 1);
  if(img_src.empty())
    return -1;

  // Convert to grayscale
  cvtColor(img_src(roi), img_gray, cv::COLOR_BGR2GRAY);

  // Convert to binary image using Canny
  Canny(img_gray, img_bin, 0, 50, 5);

  // Find contours
  std::vector<std::vector<Point>> contours;
  cv::findContours(img_bin.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // The array for storing the approximation curve
  std::vector<cv::Point> approx;

  // We'll put the labels in this destination image
  img_dest = img_src(roi).clone();

  for(int i = 0; i < contours.size(); i++) {
    // Approximate contour with accuracy proportional to the contour perimeter
    approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);

    // Skip small or non-convex objects
    if(std::fabs(contourArea(contours[i])) < 100 || !isContourConvex(approx))
      continue;

    // Detect and label circles
    double area = contourArea(contours[i]);
    Point2f center(contours[i].size());

    Rect r = boundingRect(contours[i]);
    int radius = r.width / 2;

    if(std::abs(1 - ((double)r.width / r.height)) <= 0.2 && std::abs(1 - (area / (CV_PI * std::pow(radius, 2.0)))) <= 0.2 &&
       (r.width <= 30)) {
      printf("x : %f\n", center.x);
      printf("y : %f\n", center.y);
      printf("radius: %d\n", radius);
      line(img_dest, Point(center.x, center.y), Point(center.x, center.y), Scalar(0, 255, 0), 8, 1);
      setLabel(img_dest, "CIR", contours[i]);
    }
  }

  // imshow( "bin", img_bin );
  // imshow( "dst", img_dest );

  waitKey(0);

  return 0;
}
