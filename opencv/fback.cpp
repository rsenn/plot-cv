#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>

using namespace cv;
using namespace std;

static void
help() {
  cout << "\nThis program demonstrates dense optical flow algorithm by Gunnar Farneback\n"
          "Mainly the function: calcOpticalFlowFarneback()\n"
          "Call:\n"
          "./fback\n"
          "This reads from video camera 0\n"
       << endl;
}
static void
drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step, double, const Scalar& color) {
  for(int y = 0; y < cflowmap.rows; y += step)
    for(int x = 0; x < cflowmap.cols; x += step) {
      const Point2f& fxy = flow.at<Point2f>(y, x);
      line(cflowmap, Point(x, y), Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), color);
      circle(cflowmap, Point(x, y), 2, color, -1);
    }
}

int
main(int, char**) {
  VideoCapture cap(0);
  help();
  if(!cap.isOpened())
    return -1;

  Mat prevgray, gray, flow, cflow, frame;
  namedWindow("flow", 1);

  for(;;) {
    cap >> frame;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    if(prevgray.data) {
      calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
      cvtColor(prevgray, cflow, COLOR_GRAY2BGR);
      drawOptFlowMap(flow, cflow, 16, 1.5, Scalar(0, 255, 0));
      imshow("flow", cflow);
    }
    if(waitKey(30) >= 0)
      break;
    std::swap(prevgray, gray);
  }
  return 0;
}
