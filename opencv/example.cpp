#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>

//using namespace cv;
using namespace std;

void drawText(cv::Mat& image);

int
main() {
  cout << "Built with OpenCV " << CV_VERSION << endl;
  cv::Mat image;
  cv::VideoCapture capture;
  capture.open(0);
  if(capture.isOpened()) {
    cout << "Capture is opened" << endl;
    for(;;) {
      capture >> image;
      if(image.empty())
        break;
      drawText(image);
      cv::imshow("Sample", image);
      if(cv::waitKey(10) >= 0)
        break;
    }
  } else {
    cout << "No capture" << endl;
    image = cv::Mat::zeros(480, 640, CV_8UC1);
    drawText(image);
    cv::imshow("Sample", image);
    cv::waitKey(0);
  }
  return 0;
}

void
drawText(cv::Mat& image) {
  cv::putText(image,
          "Hello OpenCV",
          cv::Point(20, 50),
          FONT_HERSHEY_COMPLEX,
          1,                     // font cv::face and scale
          cv::Scalar(255, 255, 255), // white
          1,
          cv::LINE_AA); // cv::line thickness and type
}
