#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "iostream"
using namespace cv;
using namespace std;
vector<vector<Point>> contours;
vector<Vec4i> hierarchy;
RNG rng(12345);
double area;
int
main(int argc, char** argv) {
  cv::Mat image, Result, gray;
  ;
  image = imread(argv[1], 1);
  Result = image;
  cvtColor(image, gray, COLOR_BGR2GRAY);
  threshold(gray, gray, 177, 200, THRESH_BINARY);
  findContours(gray, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
  cv::Mat drawing = cv::Mat::zeros(gray.size(), CV_8UC3);
  for(int i = 0; i < contours.size(); i++) {
    area = contourArea(contours[i]);
    Rect rect = boundingRect(contours[i]);
    if((area > 3000 && area < 4500) && (rect.width > 60 && rect.width < 100) &&
       (rect.height > 60 && rect.height < 100)) {
      cout << "contours[i]: " << area << endl;
      cout << "width" << rect.width << "height" << rect.height << endl;
      Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
      drawContours(Result, contours, i, color, 2, 8, hierarchy, 0, Point());
    }
  }
  namedWindow("Display window", WINDOW_AUTOSIZE);
  imshow("Display window", image);
  imshow("Result window", Result);
  waitKey(0);
  return 0;
}
