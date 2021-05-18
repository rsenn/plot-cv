// Program to display a video from attached default camera device and detect colored blobs using H
// and S thresholding Remove noise using opening and closing morphological operations Author:
// Samarth Manoj Brahmbhatt, University of Pennsylvania

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//using namespace cv;
using namespace std;

int hs_slider = 0, low_slider = 30, high_slider = 100;
int low_h = 30, low_s = 30, high_h = 100, high_s = 100;

void
on_hs_trackbar(int, void*) {
  switch(hs_slider) {
    case 0:
      cv::setTrackbarPos("Low cv::threshold", "Segmentation", low_h);
      cv::setTrackbarPos("High cv::threshold", "Segmentation", high_h);
      break;
    case 1:
      cv::setTrackbarPos("Low cv::threshold", "Segmentation", low_s);
      cv::setTrackbarPos("High cv::threshold", "Segmentation", high_s);
      break;
  }
}

void
on_low_thresh_trackbar(int, void*) {
  switch(hs_slider) {
    case 0:
      low_h = min(high_slider - 1, low_slider);
      cv::setTrackbarPos("Low cv::threshold", "Segmentation", low_h);
      break;
    case 1:
      low_s = min(high_slider - 1, low_slider);
      cv::setTrackbarPos("Low cv::threshold", "Segmentation", low_s);
      break;
  }
}

void
on_high_thresh_trackbar(int, void*) {
  switch(hs_slider) {
    case 0:
      high_h = max(low_slider + 1, high_slider);
      cv::setTrackbarPos("High cv::threshold", "Segmentation", high_h);
      break;
    case 1:
      high_s = max(low_slider + 1, high_slider);
      cv::setTrackbarPos("High cv::threshold", "Segmentation", high_s);
      break;
  }
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

  cv::createTrackbar("0. H\n1. S", "Segmentation", &hs_slider, 1, on_hs_trackbar);
  cv::createTrackbar("Low cv::threshold", "Segmentation", &low_slider, 255, on_low_thresh_trackbar);
  cv::createTrackbar("High cv::threshold", "Segmentation", &high_slider, 255, on_high_thresh_trackbar);

  while(char(cv::waitKey(1)) != 'q' && cap.isOpened()) {
    cv::Mat frame, frame_thresholded, frame_hsv;

    cap >> frame;

    cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);

    // Check if the video is over
    if(frame.empty()) {
      cout << "Video over" << endl;
      break;
    }

    // extract the Hue and Saturation channels
    int from_to[] = {0, 0, 1, 1};
    cv::Mat hs(frame.size(), CV_8UC2);
    cv::mixChannels(&frame_hsv, 1, &hs, 1, from_to, 2);

    // check the image for a specific range of H and S
    cv::inRange(hs, cv::Scalar(low_h, low_s), cv::Scalar(high_h, high_s), frame_thresholded);

    // open and close to remove noise
    cv::Mat str_el = cv::getStructuringElement(MORPH_ELLIPSE, cv::Size(7, 7));
    cv::morphologyEx(frame_thresholded, frame_thresholded, MORPH_OPEN, str_el);
    cv::morphologyEx(frame_thresholded, frame_thresholded, MORPH_CLOSE, str_el);

    cv::imshow("Video", frame);
    cv::imshow("Segmentation", frame_thresholded);
  }

  return 0;
}
