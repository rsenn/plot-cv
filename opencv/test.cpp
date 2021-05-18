#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace std;
//using namespace cv;

int
main(int argc, char* argv[]) {
  cv::Mat src = cv::imread(argv[1], 1);

  if(!src.data) {
    cerr << "Problem loading image!!!" << endl;
    return EXIT_FAILURE;
  }

  // cv::imshow("src", src);

  // resizing for practical reasons
  cv::Mat rsz;
  cv::Size size(600, 125);
  cv::resize(src, rsz, size);

  // cv::imshow("rsz", rsz);

  cv::Mat gray;
  cv::cvtColor(rsz, gray, cv::COLOR_BGR2GRAY);

  // Apply cv::adaptiveThreshold at the cv::bitwise_not of gray, notice the ~ symbol
  cv::Mat bw;
  cv::adaptiveThreshold(~gray, bw, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 15, -2);

  // Dilate a bit in order to correct possible gaps
  cv::Mat kernel = cv::Mat::ones(2, 2, CV_8UC1);
  cv::dilate(bw, bw, kernel);

  // Show binary image
  cv::imshow("bin", bw);

  // Create the images that will use to extract the horizontal lines
  cv::Mat horizontal = bw.clone();

  // Specify size on horizontal axis
  int horizontalsize = horizontal.cols / 30;

  // Create structure element for extracting horizontal lines through morphology operations
  cv::Mat horizontalStructure = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(horizontalsize, 1));

  // Apply morphology operations
  cv::erode(horizontal, horizontal, horizontalStructure, cv::Point(-1, -1));
  cv::dilate(horizontal, horizontal, horizontalStructure, cv::Point(-1, -1));

  // Show extracted horizontal lines
  cv::imshow("horizontal", horizontal);

  // Find external contour
  vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bw, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  vector<vector<cv::Point>> contours_poly(contours.size());
  vector<cv::Rect> boundRect(contours.size());
  for(size_t i = 0; i < contours.size(); i++) {
    cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
    boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
  }

  // Draw the contour as a solid blob filling also any convexity defect with the extracted hulls
  for(size_t i = 0; i < contours.size(); i++) {
    // cout << boundRect[i].tl() << endl;
    // cout << boundRect[i].br() << endl << endl;
    // cout << cv::arcLength(cv::Mat(contours[i]), true) << endl;
    double length = cv::arcLength(cv::Mat(contours[i]), true);
    printf("length : %f\n", length);
    // skip any noise lines
    if(length < 75)
      continue;

    if(length > 200) // filter long with short lines
    {
      boundRect[i] += cv::Size(0, -40); // expanding cv::rectangle by a certain amount
      boundRect[i] -= cv::Point(0, 3);  // shifting cv::rectangle by a certain offset
    } else {
      boundRect[i] += cv::Size(0, 40);
      boundRect[i] -= cv::Point(0, -4);
    }

    cv::drawContours(rsz, contours, i, cv::Scalar(0, 0, 255), 1, 8, vector<cv::Vec4i>(), 0, cv::Point());
    cv::rectangle(rsz, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0, 255, 0), 1, 8, 0);
  }

  cv::imshow("src", rsz);

  cv::waitKey(0);

  return 0;
}
