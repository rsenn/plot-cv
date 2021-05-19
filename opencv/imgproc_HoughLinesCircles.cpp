#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <math.h>

// using namespace cv;
using namespace std;

int
main(int argc, char** argv) {
  cv::Mat cv::img, gray;
  if(argc != 2 || !(cv::img = cv::imread(argv[1], 1)).data)
    return -1;
  cv::cvtColor(cv::img, gray, cv::COLOR_BGR2GRAY);
  // smooth cv::it, otherwise a lot of false circles may be detected
  cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);
  vector<cv::Vec3f> circles;
  cv::HoughCircles(gray, circles, HOUGH_GRADIENT, 2, gray.rows / 4, 200, 100);
  for(size_t i = 0; i < circles.size(); i++) {
    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);
    // draw the cv::circle center
    cv::circle(cv::img, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
    // draw the cv::circle outline
    cv::circle(cv::img, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
  }
  cv::namedWindow("circles", 1);
  cv::imshow("circles", cv::img);

  cv::waitKey(0);
  return 0;
}
