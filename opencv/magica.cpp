#include <iostream>
#include <opencv2/opencv.hpp>
#include "/usr/local/include/firmataplus/firmata.h"
#include "/usr/local/include/firmataplus/arduino.h"

int
main(int argc, char* argv[]) {
  Firmata firmata("/dev/ttyACM0");
  firmata.openPort("/dev/ttyACM0");
  firmata.setPinMode(13, 0x01);

  cv::VideoCapture camera(0);
  cv::Mat image;
  cv::namedWindow("camera");

  // while(cv::waitKey(30) != 27)
  for(int i = 0; i < 6; i++) {
    // camera >> image;
    // cv::imshow("camera", image);

    // Blink sketch
    firmata.writeDigitalPin(13, 0x01);
    cv::waitKey(1000);
    firmata.writeDigitalPin(13, 0x00);
    cv::waitKey(1000);
  }
}
