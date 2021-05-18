#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

//using namespace cv;
using namespace std;

int
main() {
  cv::VideoCapture capture(cv::samples::findFile("vtest.avi"));
  if(!capture.isOpened()) {
    // cv::error in opening the video input
    cerr << "Unable to open file!" << endl;
    return 0;
  }

  cv::Mat frame1, prvs;
  capture >> frame1;
  cv::cvtColor(frame1, prvs, cv::COLOR_BGR2GRAY);

  while(true) {
    cv::Mat frame2, next;
    capture >> frame2;
    if(frame2.empty())
      break;
    cv::cvtColor(frame2, next, cv::COLOR_BGR2GRAY);

    cv::Mat flow(prvs.size(), CV_32FC2);
    cv::calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

    // visualization
    cv::Mat flow_parts[2];
    cv::split(flow, flow_parts);
    cv::Mat cv::magnitude, angle, magn_norm;
    cv::cartToPolar(flow_parts[0], flow_parts[1], cv::magnitude, angle, true);
    cv::normalize(cv::magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
    angle *= ((1.f / 360.f) * (180.f / 255.f));

    // build hsv image
    cv::Mat _hsv[3], hsv, hsv8, bgr;
    _hsv[0] = angle;
    _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magn_norm;
    cv::merge(_hsv, 3, hsv);
    hsv.convertTo(hsv8, CV_8U, 255.0);
    cv::cvtColor(hsv8, bgr, cv::COLOR_HSV2BGR);

    cv::imshow("frame2", bgr);

    int keyboard = cv::waitKey(30);
    if(keyboard == 'q' || keyboard == 27)
      break;

    prvs = next;
  }
}
