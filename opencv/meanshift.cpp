#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

//using namespace cv;
using namespace std;

int
main(int argc, char** argv) {
  const string about = "This sample demonstrates the meanshift algorithm.\n"
                       "The example file can be downloaded from:\n"
                       "  "
                       "https://www.bogotobogo.com/python/OpenCV_Python/images/mean_shift_tracking/"
                       "slow_traffic_small.mp4";
  const string keys = "{ h help |      | print this help message }"
                      "{ @image |<none>| path to image file }";
  cv::CommandLineParser parser(argc, argv, keys);
  parser.about(about);
  if(parser.has("help")) {
    parser.printMessage();
    return 0;
  }
  string filename = parser.get<string>("@image");
  if(!parser.check()) {
    parser.printErrors();
    return 0;
  }

  cv::VideoCapture capture(filename);
  if(!capture.isOpened()) {
    // cv::error in opening the video input
    cerr << "Unable to open file!" << endl;
    return 0;
  }

  cv::Mat frame, roi, hsv_roi, mask;
  // take first frame of the video
  capture >> frame;

  // setup initial location of window
  cv::Rect track_window(300, 200, 100, 50); // simply hardcoded the values

  // set up the ROI for tracking
  roi = frame(track_window);
  cv::cvtColor(roi, hsv_roi, COLOR_BGR2HSV);
  cv::inRange(hsv_roi, cv::Scalar(0, 60, 32), cv::Scalar(180, 255, 255), mask);

  float range_[] = {0, 180};
  const float* range[] = {range_};
  cv::Mat roi_hist;
  int histSize[] = {180};
  int channels[] = {0};
  cv::calcHist(&hsv_roi, 1, channels, mask, roi_hist, 1, histSize, range);
  cv::normalize(roi_hist, roi_hist, 0, 255, NORM_MINMAX);

  // Setup the termination criteria, either 10 iteration or move by atleast 1 pt
  TermCriteria term_crit(TermCriteria::EPS | TermCriteria::COUNT, 10, 1);

  while(true) {
    cv::Mat hsv, dst;
    capture >> frame;
    if(frame.empty())
      break;
    cv::cvtColor(frame, hsv, COLOR_BGR2HSV);
    cv::calcBackProject(&hsv, 1, channels, roi_hist, dst, range);

    // apply meanshift to get the new location
    cv::meanShift(dst, track_window, term_crit);

    // Draw it on image
    cv::rectangle(frame, track_window, 255, 2);
    cv::imshow("img2", frame);

    int keyboard = cv::waitKey(30);
    if(keyboard == 'q' || keyboard == 27)
      break;
  }
}
