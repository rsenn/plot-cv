#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int
main(int argc, char* argv[]) {
  Mat src = imread(argc > 1 ? argv[1] : "ukwaudion.png", 0);

  Mat dst, cdst;
  Canny(src, dst, 50, 200, 3);
  cvtColor(dst, cdst, cv::COLOR_GRAY2BGR);

  vector<Vec4i> lines;
  // detect the lines
  HoughLinesP(dst, lines, 1, CV_PI / 180, 50, 50, 10);
  for(size_t i = 0; i < lines.size(); i++) {
    Vec4i l = lines[i];
    // draw the lines
    line(cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, cv::LINE_AA);
  }

  imshow("source", src);
  imshow("detected lines", cdst);

  waitKey();

  return 0;
}
