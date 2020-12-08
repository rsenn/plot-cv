/*
 * File:      submission.cpp
 * Author:    Richard Purcell
 * Date:      2020-08-27
 * Version:   1.0
 * Purpose:   Align a document from a provided image
 * Usage:     $ ./submission
 * Notes:     Created for OpenCV's Computer Vision 1 Project 3
 *            Provided filename is hard coded for this project.
 *            Plenty of streamlining still possible in this code.
 *            Some functions are based on:
 *            docs.opencv.org/master/db/d00/samples_2cpp_2squares_8cpp-example.html#a20
 */

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <iostream>

using namespace std;
using namespace cv;

Mat image, mask, bgdModel, fgdModel, result, resultGrey;
int width, height;
Rect rect;

/*
 * Name:         angle
 * Purpose:      Find the angle between three points
 * Arguments:    Point pt1, pt2, pt0
 * Outputs:      none
 * Modifies:     none
 * Returns:      double
 * Assumptions:  none
 * Bugs:         ?
 * Notes:        based on samples_2cpp_2squares_8cpp-example.html#a20
 */
static double
angle(Point pt1, Point pt2, Point pt0) {
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

int
main() {
  string filename = "./scanned-form.jpg";
  image = imread(filename, IMREAD_COLOR);
  width = image.size().width;
  height = image.size().height;
  namedWindow("result", WINDOW_NORMAL);
  rect = Rect(Point(30, 100), Point(width - 30, height - 30));
  mask.create(image.size(), CV_8UC1);

  // separate bg from fg;
  grabCut(image, mask, rect, bgdModel, fgdModel, 1, GC_INIT_WITH_RECT);
  mask = mask & 1;
  image.copyTo(result, mask);

  // process image for contour detection
  cvtColor(result, resultGrey, COLOR_BGR2GRAY);
  int thresh = 20;
  Mat canny_output;
  Canny(resultGrey, canny_output, thresh, thresh * 2);

  // find contours
  vector<vector<Point>> contours, cornerPoints;
  vector<Point> approx;
  findContours(canny_output, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  for(size_t i = 0; i < contours.size(); i++) {
    approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.02, true);

    if(approx.size() == 4 && fabs(contourArea(approx)) > 1000 && isContourConvex(approx)) {
      double maxCosine = 0;
      for(int j = 2; j < 5; j++) {
        double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
        maxCosine = MAX(maxCosine, cosine);
      }

      if(maxCosine < 0.3)
        cornerPoints.push_back(approx);
    }
  }
  // order points by y value
  for(int i = 0; i < cornerPoints[0].size() - 1; i++) {
    Point temp = cornerPoints[0][i];
    if(cornerPoints[0][i].y > cornerPoints[0][i + 1].y) {
      cornerPoints[0][i] = cornerPoints[0][i + 1];
      cornerPoints[0][i + 1] = temp;
    }
  }
  // order top points
  if(cornerPoints[0][0].x > cornerPoints[0][1].x) {
    Point temp = cornerPoints[0][0];
    cornerPoints[0][0] = cornerPoints[0][1];
    cornerPoints[0][1] = temp;
  }
  // order bottom points
  if(cornerPoints[0][3].x < cornerPoints[0][4].x) {
    Point temp = cornerPoints[0][3];
    cornerPoints[0][3] = cornerPoints[0][4];
    cornerPoints[0][3] = temp;
  }

  // create correctly sized document template given 500px width
  Mat alignedDoc(647, 500, CV_8UC3, Scalar(126, 0, 255));
  vector<Point2f> targetPoints = {Point(0, 0), Point(500, 0), Point(500, 647), Point(0, 647)};

  // copy points to new vector for clarity
  vector<Point2f> sourcePoints;
  for(int i = 0; i < 4; i++) {
    sourcePoints.push_back(cornerPoints[0][i]);
  }

  // align document using homography
  Mat h = findHomography(sourcePoints, targetPoints, RANSAC);
  warpPerspective(image, alignedDoc, h, alignedDoc.size());

  // display result image
  imshow("result", alignedDoc);
  waitKey(0);

  return 0;
}