#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <stdlib.h>

using namespace std;
//using namespace cv;

/** Function Headers */
void on_low_r_thresh_trackbar(int, void*);
void on_high_r_thresh_trackbar(int, void*);
void on_low_g_thresh_trackbar(int, void*);
void on_high_g_thresh_trackbar(int, void*);
void on_low_b_thresh_trackbar(int, void*);
void on_high_b_thresh_trackbar(int, void*);

/** Global Variables */
int low_r = 30, low_g = 30, low_b = 30;
int high_r = 100, high_g = 100, high_b = 100;

/** @function main */
int
main() {
  //! [mat]
  cv::Mat frame, frame_threshold;
  //! [mat]
  //! [cap]
  cv::VideoCapture cap(0);
  //! [cap]
  //! [window]
  cv::namedWindow("Video Capture", cv::WINDOW_NORMAL);
  cv::namedWindow("Object Detection", cv::WINDOW_NORMAL);
  //! [window]
  //! [trackbar]
  //-- Trackbars to set thresholds for RGB values
  cv::createTrackbar("Low R", "Object Detection", &low_r, 255, on_low_r_thresh_trackbar);
  cv::createTrackbar("High R", "Object Detection", &high_r, 255, on_high_r_thresh_trackbar);
  cv::createTrackbar("Low G", "Object Detection", &low_g, 255, on_low_g_thresh_trackbar);
  cv::createTrackbar("High G", "Object Detection", &high_g, 255, on_high_g_thresh_trackbar);
  cv::createTrackbar("Low B", "Object Detection", &low_b, 255, on_low_b_thresh_trackbar);
  cv::createTrackbar("High B", "Object Detection", &high_b, 255, on_high_b_thresh_trackbar);
  //! [trackbar]
  while((char)cv::waitKey(1) != 'q') {
    //! [while]
    cap >> frame;
    if(frame.empty())
      break;
    //-- Detect the object based on RGB cv::Range Values
    cv::inRange(frame, cv::Scalar(low_b, low_g, low_r), cv::Scalar(high_b, high_g, high_r), frame_threshold);
    //! [while]
    //! [show]
    //-- Show the frames
    cv::imshow("Video Capture", frame);
    cv::imshow("Object Detection", frame_threshold);
    //! [show]
  }
  return 0;
}
//! [low]
/** @function on_low_r_thresh_trackbar */
void
on_low_r_thresh_trackbar(int, void*) {
  low_r = min(high_r - 1, low_r);
  cv::setTrackbarPos("Low R", "Object Detection", low_r);
}
//! [low]
//! [high]
/** @function on_high_r_thresh_trackbar */
void
on_high_r_thresh_trackbar(int, void*) {
  high_r = max(high_r, low_r + 1);
  cv::setTrackbarPos("High R", "Object Detection", high_r);
}
//![high]
/** @function on_low_g_thresh_trackbar */
void
on_low_g_thresh_trackbar(int, void*) {
  low_g = min(high_g - 1, low_g);
  cv::setTrackbarPos("Low G", "Object Detection", low_g);
}

/** @function on_high_g_thresh_trackbar */
void
on_high_g_thresh_trackbar(int, void*) {
  high_g = max(high_g, low_g + 1);
  cv::setTrackbarPos("High G", "Object Detection", high_g);
}

/** @function on_low_b_thresh_trackbar */
void
on_low_b_thresh_trackbar(int, void*) {
  low_b = min(high_b - 1, low_b);
  cv::setTrackbarPos("Low B", "Object Detection", low_b);
}

/** @function on_high_b_thresh_trackbar */
void
on_high_b_thresh_trackbar(int, void*) {
  high_b = max(high_b, low_b + 1);
  cv::setTrackbarPos("High B", "Object Detection", high_b);
}
