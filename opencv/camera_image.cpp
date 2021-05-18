#include <opencv2/opencv.hpp>
#include <unistd.h>
//using namespace cv;

Mat
cameraCapture(cv::VideoCapture& cap) {
  cv::Mat img;
  cap >> img;
  cv::Mat img_filtered = cv::Mat::zeros(img.size(), img.type());
  cv::medianBlur(img, img_filtered, 5);
  return img_filtered;
}

int
main(int argc, char** argv) {
  cv::VideoCapture cap;

  double k = 0.9;

  if(!cap.open(0)) {
    return 0;
  }

  cv::Mat ref = cameraCapture(cap);

  while(true) {

    cv::Mat current = cameraCapture(cap);

    cv::Mat diff = cv::Mat::zeros(ref.size(), ref.type());
    cv::absdiff(ref, current, diff);
    cv::threshold(diff, diff, 20, 1, THRESH_BINARY);
    float different_pixels = cv::sum(diff)[0];
    std::cout << "Detected " << different_pixels << " different pixels";

    // cv::imshow("Current", current);
    if(different_pixels > 100) {
      std::cout << "   [Intruder alert!!]" << std::endl;
      cv::imshow("Intruder", current);
      cv::waitKey(1);
    } else {
      std::cout << std::endl;
    }
    ref = k * ref + (1 - k) * current;

    usleep(100000);
  }

  while(cv::waitKey(1) != 27) {}

  return 0;
}
