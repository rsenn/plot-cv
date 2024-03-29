/*
 *
 * Hybrid Tracking in OpenCV
 * Usage: ./hybridtrackingsample live
 *
 * For Benchmarking against the Bonn benchmark dataset
 * wget http://www.iai.uni-bonn.de/~kleind/tracking/datasets/seqG.zip
 * unzip seqG.zip -d ./seqG
 * ffmpeg -i seqG/Vid_G_rubikscube.avi seqG/%04d.png
 * ./hytrack seqG/Vid_G_rubikscube.txt
 *
 */

#include <stdio.h>
#include <time.h>
#include <iostream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/hybridtracker.hpp"

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

// using namespace cv;
using namespace std;

cv::Mat frame, image;
cv::Rect selection;
cv::Point origin;
bool selectObject = false;
int trackObject = 0;
int live = 1;

static void
drawRectangle(cv::Mat* img, cv::Rect win) {
  cv::rectangle(
      *img, cv::Point(win.x, win.y), cv::Point(win.x + win.width, win.y + win.height), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
}

static void
onMouse(int event, int x, int y, int, void*) {
  if(selectObject) {
    selection.x = MIN(x, origin.x);
    selection.y = MIN(y, origin.y);
    selection.width = std::abs(x - origin.x);
    selection.height = std::abs(y - origin.y);
    selection &= cv::Rect(0, 0, image.cols, image.rows);
  }

  switch(event) {
    case cv::EVENT_LBUTTONDOWN:
      origin = cv::Point(x, y);
      selection = cv::Rect(x, y, 0, 0);
      selectObject = true;
      break;
    case cv::EVENT_LBUTTONUP:
      selectObject = false;
      trackObject = -1;
      break;
  }
}

static void
help() {
  printf("Usage: ./hytrack live or ./hytrack <test_file> \n\
For Live View or Benchmarking. Read documentation is source code.\n\n");
}

int
main(int argc, char** argv) {
  if(argc != 2) {
    help();
    return 1;
  }

  FILE* f = 0;
  cv::VideoCapture cap;
  char test_file[20] = "";

  if(strcmp(argv[1], "live") != 0) {
    sprintf(test_file, "%s", argv[1]);
    f = fopen(test_file, "r");
    char vid[20];
    int values_read = fscanf(f, "%s\n", vid);
    CV_Assert(values_read == 1);
    cout << "Benchmarking against " << vid << endl;
    live = 0;
  } else {
    cap.open(0);
    if(!cap.isOpened()) {
      cout << "Failed to open camera" << endl;
      return 0;
    }
    cout << "Opened camera" << endl;
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap >> frame;
  }

  HybridTrackerParams params;
  // motion model params
  params.motion_model = CvMotionModel::LOW_PASS_FILTER;
  params.low_pass_gain = 0.1f;
  // cv::mean shift params
  params.ms_tracker_weight = 0.8f;
  params.ms_params.tracking_type = CvMeanShiftTrackerParams::HS;
  // feature tracking params
  params.ft_tracker_weight = 0.2f;
  params.ft_params.feature_type = CvFeatureTrackerParams::OPTICAL_FLOW;
  params.ft_params.window_size = 0;

  HybridTracker tracker(params);
  char img_file[20] = "seqG/0001.png";
  char img_file_num[10];
  cv::namedWindow("Win", 1);

  cv::setMouseCallback("Win", onMouse, 0);

  int i = 0;
  float w[4];
  for(;;) {
    i++;
    if(live) {
      cap >> frame;
      if(frame.empty())
        break;
      frame.copyTo(image);
    } else {
      int values_read = fscanf(f, "%d %f %f %f %f\n", &i, &w[0], &w[1], &w[2], &w[3]);
      CV_Assert(values_read == 5);
      sprintf(img_file, "seqG/%04d.png", i);
      image = cv::imread(img_file, cv::LOAD_IMAGE_COLOR);
      if(image.empty())
        break;
      selection = cv::Rect(cvRound(w[0] * image.cols),
                           cvRound(w[1] * image.rows),
                           cvRound(w[2] * image.cols),
                           cvRound(w[3] * image.rows));
    }

    sprintf(img_file_num, "Frame: %d", i);
    cv::putText(image, img_file_num, cv::Point(10, image.rows - 20), FONT_HERSHEY_PLAIN, 0.75, cv::Scalar(255, 255, 255));
    if(!image.empty()) {

      if(trackObject < 0) {
        tracker.newTracker(image, selection);
        trackObject = 1;
      }

      if(trackObject) {
        tracker.updateTracker(image);
        drawRectangle(&image, tracker.getTrackingWindow());
      }

      if(selectObject && selection.width > 0 && selection.height > 0) {
        cv::Mat roi(image, selection);
        cv::bitwise_not(roi, roi);
      }

      drawRectangle(&image,
                    cv::Rect(cvRound(w[0] * image.cols),
                             cvRound(w[1] * image.rows),
                             cvRound(w[2] * image.cols),
                             cvRound(w[3] * image.rows)));
      cv::imshow("Win", image);

      cv::waitKey(100);
    } else
      i = 0;
  }

  fclose(f);
  return 0;
}
