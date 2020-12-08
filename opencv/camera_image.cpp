#include <opencv2/opencv.hpp>
#include <unistd.h>
using namespace cv;

Mat
cameraCapture(VideoCapture& cap) {
  Mat img;
  cap >> img;
  Mat img_filtered = Mat::zeros(img.size(), img.type());
  medianBlur(img, img_filtered, 5);
  return img_filtered;
}

int
main(int argc, char** argv) {
  VideoCapture cap;

  double k = 0.9;

  if(!cap.open(0)) {
    return 0;
  }

  Mat ref = cameraCapture(cap);

  while(true) {

    Mat current = cameraCapture(cap);

    Mat diff = Mat::zeros(ref.size(), ref.type());
    absdiff(ref, current, diff);
    threshold(diff, diff, 20, 1, THRESH_BINARY);
    float different_pixels = sum(diff)[0];
    std::cout << "Detected " << different_pixels << " different pixels";

    // imshow("Current", current);
    if(different_pixels > 100) {
      std::cout << "   [Intruder alert!!]" << std::endl;
      imshow("Intruder", current);
      waitKey(1);
    } else {
      std::cout << std::endl;
    }
    ref = k * ref + (1 - k) * current;

    usleep(100000);
  }

  while(waitKey(1) != 27) {
  }

  return 0;
}
