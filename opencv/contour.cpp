#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
// using namespace cv;
using namespace std;
std::vector<std::vector<cv::Point>> contours;
std::vector<cv::Vec4i> hierarchy;
cv::RNG rng(12345);
double area;
int
main(int argc, char** argv) {
  cv::Mat image, Result, gray;
  ;
  image = cv::imread(argv[1], 1);
  Result = image;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, gray, 177, 200, cv::THRESH_BINARY);
  cv::findContours(gray, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  cv::Mat drawing = cv::Mat::zeros(gray.size(), CV_8UC3);
  for(int i = 0; i < contours.size(); i++) {
    area = cv::contourArea(contours[i]);
    cv::Rect rect = cv::boundingRect(contours[i]);
    if((area > 3000 && area < 4500) && (rect.width > 60 && rect.width < 100) && (rect.height > 60 && rect.height < 100)) {
      cout << "contours[i]: " << area << endl;
      cout << "width" << rect.width << "height" << rect.height << endl;
      cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
      cv::drawContours(Result, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
    }
  }
  cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
  cv::imshow("Display window", image);
  cv::imshow("Result window", Result);
  cv::waitKey(0);
  return 0;
}
