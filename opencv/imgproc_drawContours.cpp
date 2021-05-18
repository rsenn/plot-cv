#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//using namespace cv;
using namespace std;

int
main(int argc, char** argv) {
  cv::Mat src;
  // the first command-cv::line parameter must be a filename of the binary
  // (black-n-white) image
  if(argc != 2 || !(src = cv::imread(argv[1], 0)).data)
    return -1;

  cv::Mat dst = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);

  src = src > 1;
  cv::namedWindow("Source", 1);
  cv::imshow("Source", src);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::findContours(src, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

  // iterate through all the top-level contours,
  // draw each connected component with its own random color
  int idx = 0;
  for(; idx >= 0; idx = hierarchy[idx][0]) {
    cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
    cv::drawContours(dst, contours, idx, color, cv::FILLED, 8, hierarchy);
  }

  cv::namedWindow("Components", 1);
  cv::imshow("Components", dst);
  cv::waitKey(0);
}
