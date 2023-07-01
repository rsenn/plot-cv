#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <math.h>
#include <iostream>
// using namespace cv;
using namespace std;
static void
help() {
  cout << "\nThis program illustrates the use of cv::findContours and cv::drawContours\n"
       << "The original image is put up along with the image of drawn contours\n"
       << "Usage:\n"
       << "./contours2\n"
       << "\nA trackbar is put up which controls the contour level from -3 to 3\n"
       << endl;
}
const int w = 500;
int levels = 3;
std::vector<std::vector<cv::Point>> contours;
std::vector<cv::Vec4i> hierarchy;
static void
on_trackbar(int, void*) {
  cv::Mat cnt_img = cv::Mat::zeros(w, w, CV_8UC3);
  int _levels = levels - 3;
  cv::drawContours(
      cnt_img, contours, _levels <= 0 ? 3 : -1, cv::Scalar(128, 255, 255), 3, cv::LINE_AA, hierarchy, std::abs(_levels));
  cv::imshow("contours", cnt_img);
}
int
main(int argc, char** argv) {
  cv::CommandLineParser parser(argc, argv, "{help h||}");
  if(parser.has("help")) {
    help();
    return 0;
  }
  cv::Mat img = cv::Mat::zeros(w, w, CV_8UC1);
  // Draw 6 faces
  for(int i = 0; i < 6; i++) {
    int dx = (i % 2) * 250 - 30;
    int dy = (i / 2) * 150;
    const cv::Scalar white = cv::Scalar(255);
    const cv::Scalar black = cv::Scalar(0);
    if(i == 0) {
      for(int j = 0; j <= 10; j++) {
        double angle = (j + 5) * CV_PI / 21;
        cv::line(img,
                 cv::Point(cvRound(dx + 100 + j * 10 - 80 * cos(angle)), cvRound(dy + 100 - 90 * sin(angle))),
                 cv::Point(cvRound(dx + 100 + j * 10 - 30 * cos(angle)), cvRound(dy + 100 - 30 * sin(angle))),
                 white,
                 1,
                 8,
                 0);
      }
    }
    cv::ellipse(img, cv::Point(dx + 150, dy + 100), cv::Size(100, 70), 0, 0, 360, white, -1, 8, 0);
    cv::ellipse(img, cv::Point(dx + 115, dy + 70), cv::Size(30, 20), 0, 0, 360, black, -1, 8, 0);
    cv::ellipse(img, cv::Point(dx + 185, dy + 70), cv::Size(30, 20), 0, 0, 360, black, -1, 8, 0);
    cv::ellipse(img, cv::Point(dx + 115, dy + 70), cv::Size(15, 15), 0, 0, 360, white, -1, 8, 0);
    cv::ellipse(img, cv::Point(dx + 185, dy + 70), cv::Size(15, 15), 0, 0, 360, white, -1, 8, 0);
    cv::ellipse(img, cv::Point(dx + 115, dy + 70), cv::Size(5, 5), 0, 0, 360, black, -1, 8, 0);
    cv::ellipse(img, cv::Point(dx + 185, dy + 70), cv::Size(5, 5), 0, 0, 360, black, -1, 8, 0);
    cv::ellipse(img, cv::Point(dx + 150, dy + 100), cv::Size(10, 5), 0, 0, 360, black, -1, 8, 0);
    cv::ellipse(img, cv::Point(dx + 150, dy + 150), cv::Size(40, 10), 0, 0, 360, black, -1, 8, 0);
    cv::ellipse(img, cv::Point(dx + 27, dy + 100), cv::Size(20, 35), 0, 0, 360, white, -1, 8, 0);
    cv::ellipse(img, cv::Point(dx + 273, dy + 100), cv::Size(20, 35), 0, 0, 360, white, -1, 8, 0);
  }
  // show the faces
  cv::namedWindow("image", 1);
  cv::imshow("image", img);
  // Extract the contours so that
  std::vector<std::vector<cv::Point>> contours0;
  cv::findContours(img, contours0, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  contours.resize(contours0.size());
  for(size_t k = 0; k < contours0.size(); k++) cv::approxPolyDP(cv::Mat(contours0[k]), contours[k], 3, true);
  cv::namedWindow("contours", 1);
  cv::createTrackbar("levels+3", "contours", &levels, 7, on_trackbar);
  on_trackbar(0, 0);
  cv::waitKey();
  return 0;
}
