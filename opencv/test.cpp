#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int
main(int argc, char* argv[]) {
  Mat src = imread(argv[1], 1);

  if(!src.data) {
    cerr << "Problem loading image!!!" << endl;
    return EXIT_FAILURE;
  }

  // imshow("src", src);

  // resizing for practical reasons
  Mat rsz;
  Size size(600, 125);
  resize(src, rsz, size);

  // imshow("rsz", rsz);

  Mat gray;
  cvtColor(rsz, gray, cv::COLOR_BGR2GRAY);

  // Apply adaptiveThreshold at the bitwise_not of gray, notice the ~ symbol
  Mat bw;
  adaptiveThreshold(~gray, bw, 255, cv::ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 15, -2);

  // Dilate a bit in order to correct possible gaps
  Mat kernel = Mat::ones(2, 2, CV_8UC1);
  dilate(bw, bw, kernel);

  // Show binary image
  imshow("bin", bw);

  // Create the images that will use to extract the horizontal lines
  Mat horizontal = bw.clone();

  // Specify size on horizontal axis
  int horizontalsize = horizontal.cols / 30;

  // Create structure element for extracting horizontal lines through morphology operations
  Mat horizontalStructure = getStructuringElement(MORPH_RECT, Size(horizontalsize, 1));

  // Apply morphology operations
  erode(horizontal, horizontal, horizontalStructure, Point(-1, -1));
  dilate(horizontal, horizontal, horizontalStructure, Point(-1, -1));

  // Show extracted horizontal lines
  imshow("horizontal", horizontal);

  // Find external contour
  vector<Vec4i> hierarchy;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bw, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<vector<Point>> contours_poly(contours.size());
  vector<Rect> boundRect(contours.size());
  for(size_t i = 0; i < contours.size(); i++) {
    approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
    boundRect[i] = boundingRect(Mat(contours_poly[i]));
  }

  // Draw the contour as a solid blob filling also any convexity defect with the extracted hulls
  for(size_t i = 0; i < contours.size(); i++) {
    // cout << boundRect[i].tl() << endl;
    // cout << boundRect[i].br() << endl << endl;
    // cout << arcLength(cv::Mat(contours[i]), true) << endl;
    double length = arcLength(cv::Mat(contours[i]), true);
    printf("length : %f\n", length);
    // skip any noise lines
    if(length < 75)
      continue;

    if(length > 200) // filter long with short lines
    {
      boundRect[i] += Size(0, -40); // expanding rectangle by a certain amount
      boundRect[i] -= Point(0, 3);  // shifting rectangle by a certain offset
    } else {
      boundRect[i] += Size(0, 40);
      boundRect[i] -= Point(0, -4);
    }

    drawContours(rsz, contours, i, Scalar(0, 0, 255), 1, 8, vector<Vec4i>(), 0, Point());
    rectangle(rsz, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 1, 8, 0);
  }

  imshow("src", rsz);

  waitKey(0);

  return 0;
}
