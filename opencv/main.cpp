#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <cmath>
#include "detectLines.hpp"
#include "Measure_Origins.hpp"

using namespace std;
// using namespace cv;

int
main(int argc, char* argv[]) {
  // cv::read input image
  cv::Mat image_org = cv::imread(argv[1], 0);

  vector<cv::Point> c;
  // Measure_Origins(image_org);
  detectCurve(image_org);
}
