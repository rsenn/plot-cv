#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

//using namespace cv;
using namespace std;

static void
help() {
  cout << "\nThis program demonstrates dense optical flow algorithm by Gunnar Farneback\n"
          "Mainly the function: cv::calcOpticalFlowFarneback()\n"
          "Call:\n"
          "./fback\n"
          "This reads from video camera 0\n"
       << endl;
}
static void
drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step, double, const cv::Scalar& color) {
  for(int y = 0; y < cflowmap.rows; y += step)
    for(int x = 0; x < cflowmap.cols; x += step) {
      const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
      cv::line(cflowmap, cv::Point(x, y), cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), color);
      cv::circle(cflowmap, cv::Point(x, y), 2, color, -1);
    }
}

int
main(int, char**) {
  cv::VideoCapture cap(0);
  help();
  if(!cap.isOpened())
    return -1;

  cv::Mat prevgray, gray, flow, cflow, frame;
  cv::namedWindow("flow", 1);

  for(;;) {
    cap >> frame;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    if(prevgray.data) {
      cv::calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
      cv::cvtColor(prevgray, cflow, cv::COLOR_GRAY2BGR);
      drawOptFlowMap(flow, cflow, 16, 1.5, cv::Scalar(0, 255, 0));
      cv::imshow("flow", cflow);
    }
    if(cv::waitKey(30) >= 0)
      break;
    std::cv::swap(prevgray, gray);
  }
  return 0;
}
