// Code to check the OpenCV installation on Raspberry Pi and mesaure frame rate
// Author: Samarth Manoj Brahmbhatt, University of Pennsyalvania

#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/types_c.h>

using namespace std;
//using namespace cv;

int
main() {
  cv::namedWindow("Hello");

  cv::VideoCapture cap(0);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

  cv::Mat im, im_g;
  double time = 0;
  unsigned int frames = 0;
  while(char(cv::waitKey(1)) != 'q') {
    double t0 = cv::getTickCount();
    cap >> im;
    cv::cvtColor(im, im_g, cv::COLOR_BGR2GRAY);
    frames++;
    cv::imshow("Hello", im_g);
    time += (cv::getTickCount() - t0) / cv::getTickFrequency();
    cout << frames / time << " fps" << endl;
  }

  return 0;
}
