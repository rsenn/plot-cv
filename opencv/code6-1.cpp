// Program to illustrate contour extraction and heirarchy of contours
// Author: Samarth Manoj Brahmbhatt, University of Pennsylvania

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

using namespace std;
using namespace cv;

cv::Mat img;
std::vector<std::vector<cv::Point>> contours;
std::vector<Vec4i> heirarchy;
int levels = 0;

void
on_trackbar(int, void*) {
  cv::Mat img_show = img.clone();
  drawContours(img_show, contours, -1, Scalar(0, 0, 255), 3, 8, heirarchy, levels);
  imshow("Contours", img_show);
}

int
main() {
  img = imread("bullseye.jpg");

  cv::Mat edges;
  Canny(img, edges, 50, 100);

  findContours(edges, contours, heirarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

  namedWindow("Contours", cv::WINDOW_AUTOSIZE);

  createTrackbar("Levels", "Contours", &levels, 15, on_trackbar);

  on_trackbar(0, 0);

  while(char(waitKey(1)) != 'q') {}

  return 0;
}
