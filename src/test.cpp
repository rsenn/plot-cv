#include "opencv2/opencv.hpp"
#include <stdio.h>

using namespace cv;

int
main() {
  // printf("asfdsdfsdf\n");
  VideoCapture cap(0);
  Mat img;
  cap.read(img);
  imwrite("alpha.jpg", img);

  return 0;
}
