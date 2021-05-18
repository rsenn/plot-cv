// Program to illustrate frame capture from a USB stereo camera
// Author: Samarth Manoj Brahmbhatt, University of Pennsylvania

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>

//using namespace cv;
using namespace std;

int
main() {
  cv::VideoCapture capr(1), capl(2);
  // reduce frame size
  capl.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
  capl.set(cv::CAP_PROP_FRAME_WIDTH, 320);
  capr.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
  capr.set(cv::CAP_PROP_FRAME_WIDTH, 320);

  cv::namedWindow("Left");
  cv::namedWindow("Right");

  while(char(cv::waitKey(1)) != 'q') {
    // grab raw frames first
    capl.grab();
    capr.grab();
    // decode later so the grabbed frames are less apart in time
    cv::Mat framel, framer;
    capl.retrieve(framel);
    capr.retrieve(framer);

    if(framel.empty() || framer.empty())
      break;

    cv::imshow("Left", framel);
    cv::imshow("Right", framer);
  }
  capl.release();
  capr.release();
  return 0;
}
