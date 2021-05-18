// Program to automate the color-based object detector using floodfill
// Author: Samarth Manoj Brahmbhatt, University of Pennsylvania

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

//using namespace cv;
using namespace std;

cv::Mat frame_hsv, frame, mask;

int low_diff = 10, high_diff = 10, conn = 4, val = 255, flags = conn + (val << 8) + CV_FLOODFILL_MASK_ONLY;
double h_h = 0, l_h = 0, h_s = 0, l_s = 0;

bool selected = false;

void
on_low_diff_trackbar(int, void*) {
}

void
on_high_diff_trackbar(int, void*) {
}

void
on_mouse(int event, int x, int y, int, void*) {
  if(event != cv::EVENT_LBUTTONDOWN)
    return;

  selected = true;

  // seed point
  cv::Point p(x, y);

  // make mask using floodFill
  mask = cv::Scalar::all(0);
  cv::floodFill(frame,
            mask,
            p,
            cv::Scalar(255, 255, 255),
            0,
            cv::Scalar(low_diff, low_diff, low_diff),
            cv::Scalar(high_diff, high_diff, high_diff),
            flags);

  // find the H and S range of piexels selected by floodFill
  cv::Mat channels[3];
  cv::split(frame_hsv, channels);
  cv::minMaxLoc(channels[0], &l_h, &h_h, NULL, NULL, mask.rowRange(1, mask.rows - 1).colRange(1, mask.cols - 1));
  cv::minMaxLoc(channels[1], &l_s, &h_s, NULL, NULL, mask.rowRange(1, mask.rows - 1).colRange(1, mask.cols - 1));
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
  cv::namedWindow("Segmentation");

  cv::createTrackbar("Low Diff", "Segmentation", &low_diff, 50, on_low_diff_trackbar);
  cv::createTrackbar("High Diff ", "Segmentation", &high_diff, 50, on_high_diff_trackbar);

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

    // extract the hue and saturation channels
    int from_to[] = {0, 0, 1, 1};
    cv::Mat hs(frame.size(), CV_8UC2);
    cv::mixChannels(&frame_hsv, 1, &hs, 1, from_to, 2);

    // check for the range of H and S obtained from floodFill
    cv::Mat frame_thresholded;
    cv::inRange(hs, cv::Scalar(l_h, l_s), cv::Scalar(h_h, h_s), frame_thresholded);

    // open and close to remove noise
    cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(frame_thresholded, frame_thresholded, cv::MORPH_OPEN, str_el);
    cv::morphologyEx(frame_thresholded, frame_thresholded, cv::MORPH_CLOSE, str_el);

    cv::imshow("Video", frame);
    cv::imshow("Segmentation", frame_thresholded);
  }

  return 0;
}
