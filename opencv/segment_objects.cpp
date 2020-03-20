#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/video/background_segm.hpp>
#include <cstdio>
#include <string>
#include <std::vector>

using namespace cv;

static void
help() {
  printf("\n"
         "This program demonstrated a simple method of connected components clean up of background "
         "subtraction\n"
         "When the program starts, it begins learning the background.\n"
         "You can toggle background learning on and off by hitting the space bar.\n"
         "Call\n"
         "./segment_objects [video file, else it reads camera 0]\n\n");
}

static void
refineSegments(const cv::Mat& img, cv::Mat& mask, cv::Mat& dst) {
  int niters = 3;

  std::vector<std::vector<cv::Point>> contours;
  std::vector<Vec4i> hierarchy;

  cv::Mat temp;

  dilate(mask, temp, cv::Mat(), cv::Point(-1, -1), niters);
  erode(temp, temp, cv::Mat(), cv::Point(-1, -1), niters * 2);
  dilate(temp, temp, cv::Mat(), cv::Point(-1, -1), niters);

  findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

  dst = cv::Mat::zeros(img.size(), CV_8UC3);

  if(contours.size() == 0)
    return;

  // iterate through all the top-level contours,
  // draw each connected component with its own random color
  int idx = 0, largestComp = 0;
  double maxArea = 0;

  for(; idx >= 0; idx = hierarchy[idx][0]) {
    const std::vector<cv::Point>& c = contours[idx];
    double area = fabs(contourArea(cv::Mat(c)));
    if(area > maxArea) {
      maxArea = area;
      largestComp = idx;
    }
  }
  Scalar color(0, 0, 255);
  drawContours(dst, contours, largestComp, color, CV_FILLED, 8, hierarchy);
}

int
main(int argc, char** argv) {
  VideoCapture cap;
  bool update_bg_model = true;

  help();

  if(argc < 2)
    cap.open(0);
  else
    cap.open(std::string(argv[1]));

  if(!cap.isOpened()) {
    printf("\nCan not open camera or video file\n");
    return -1;
  }

  cv::Mat tmp_frame, bgmask, out_frame;

  cap >> tmp_frame;
  if(!tmp_frame.data) {
    printf("can not read data from the video source\n");
    return -1;
  }

  namedWindow("video", 1);
  namedWindow("segmented", 1);

  cv::BackgroundSubtractorMOG2 bgsubtractor;
  bgsubtractor.set("noiseSigma", 10);

  for(;;) {
    cap >> tmp_frame;
    if(!tmp_frame.data)
      break;
    bgsubtractor(tmp_frame, bgmask, update_bg_model ? -1 : 0);
    refineSegments(tmp_frame, bgmask, out_frame);
    imshow("video", tmp_frame);
    imshow("segmented", out_frame);
    int keycode = waitKey(30);
    if(keycode == 27)
      break;
    if(keycode == ' ') {
      update_bg_model = !update_bg_model;
      printf("Learn background is in state = %d\n", update_bg_model);
    }
  }

  return 0;
}
