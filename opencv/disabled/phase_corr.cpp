#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//using namespace cv;

int
main(int, char*[]) {
  cv::VideoCapture video(0);
  cv::Mat frame, curr, prev, curr64f, prev64f, hann;
  int key = 0;

  do {
    video >> frame;
    cv::cvtColor(frame, curr, cv::COLOR_RGB2GRAY);

    if(prev.empty()) {
      prev = curr.clone();
      createHanningWindow(hann, curr.size(), CV_64F);
    }

    prev.convertTo(prev64f, CV_64F);
    curr.convertTo(curr64f, CV_64F);

    Point2d shift = phaseCorrelate(prev64f, curr64f, hann);
    double radius = cv::sqrt(shift.x * shift.x + shift.y * shift.y);

    if(radius > 5) {
      // draw a cv::circle and cv::line indicating the shift direction...
      cv::Point center(curr.cols >> 1, curr.rows >> 1);
      cv::circle(frame, center, (int)radius, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
      cv::line(frame, center, cv::Point(center.x + (int)shift.x, center.y + (int)shift.y), cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
    }

    cv::imshow("phase shift", frame);
    key = cv::waitKey(2);

    prev = curr.clone();
  } while((char)key != 27); // Esc to exit...

  return 0;
}
