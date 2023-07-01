// Program to illustrate histogram backprojection
// Author: Samarth Manoj Brahmbhatt, University of Pennsylvania

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// using namespace cv;
using namespace std;

cv::Mat frame_hsv, frame, mask;
MatND hist; // 2D histogram
int conn = 4, val = 255, flags = conn + (val << 8) + CV_FLOODFILL_MASK_ONLY;

bool selected = false;

// hue and saturation histogram ranges
float hrange[] = {0, 179}, srange[] = {0, 255};
const float* ranges[] = {hrange, srange};

void
on_mouse(int event, int x, int y, int, void*) {
  if(event != EVENT_LBUTTONDOWN)
    return;

  selected = true;

  // floodFill
  cv::Point p(x, y);
  mask = cv::Scalar::all(0);
  cv::floodFill(frame, mask, p, cv::Scalar(255, 255, 255), 0, cv::Scalar(10, 10, 10), cv::Scalar(10, 10, 10), flags);
  cv::Mat _mask = mask.rowRange(1, mask.rows - 1).colRange(1, mask.cols - 1);

  // number of bins in the histogram for each channel
  int histSize[] = {50, 50}, channels[] = {0, 1};

  // calculate and cv::normalize histogram
  cv::calcHist(&frame_hsv, 1, channels, _mask, hist, 2, histSize, ranges);
  cv::normalize(hist, hist, 0, 255, NORM_MINMAX, -1, cv::Mat());
}

int
main() {
  // Create a cv::VideoCapture object to cv::read from video file
  // 0 is the ID of the built-in laptop camera, change if you want to use other camera
  cv::VideoCapture cap(0);

  // check if the file was opened properly
  if(!cap.isOpened()) {
    cout << "Capture could not be opened succesfully" << endl;
    return -1;
  }

  cv::namedWindow("Video");
  cv::namedWindow("Backprojection");

  cv::setMouseCallback("Video", on_mouse);

  while(char(cv::waitKey(1)) != 'q' && cap.isOpened()) {
    cap >> frame;
    if(!selected)
      mask.create(frame.rows + 2, frame.cols + 2, CV_8UC1);
    // Check if the video is over
    if(frame.empty()) {
      cout << "Video over" << endl;
      break;
    }
    cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);

    // backproject on the HSV image
    cv::Mat frame_backprojected = cv::Mat::zeros(frame.size(), CV_8UC1);
    if(selected) {
      int channels[] = {0, 1};
      cv::calcBackProject(&frame_hsv, 1, channels, hist, frame_backprojected, ranges);
    }

    cv::imshow("Video", frame);
    cv::imshow("Backprojection", frame_backprojected);
  }

  return 0;
}
