/**
 * @file objectDetection2.cpp
 * @author A. Huaman ( based in the classic facedetect.cpp in cv::samples/c )
 * @brief A simplified version of facedetect.cpp, show how to load a cascade classifier and how to
 * find objects (Face + eyes) in a video stream - Using LBP here
 */
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>

using namespace std;
// using namespace cv;

/** Function Headers */
void detectAndDisplay(cv::Mat frame);

/** Global variables */
cv::String face_cascade_name = "lbpcascade_frontalface.xml";
cv::String eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
cv::CascadeClassifier face_cascade;
cv::CascadeClassifier eyes_cascade;
string window_name = "Capture - Face detection";

cv::RNG rng(12345);

/**
 * @function main
 */
int
main(void) {
  cv::VideoCapture capture;
  cv::Mat frame;

  //-- 1. Load the cascade
  if(!face_cascade.load(face_cascade_name)) {
    printf("--(!)cv::Error loading\n");
    return -1;
  };
  if(!eyes_cascade.load(eyes_cascade_name)) {
    printf("--(!)cv::Error loading\n");
    return -1;
  };

  //-- 2. Read the video stream
  capture.open(-1);
  if(capture.isOpened()) {
    for(;;) {
      capture >> frame;

      //-- 3. Apply the classifier to the frame
      if(!frame.empty()) {
        detectAndDisplay(frame);
      } else {
        printf(" --(!) No captured frame -- Break!");
        break;
      }

      int c = cv::waitKey(10);
      if((char)c == 'c') {
        break;
      }
    }
  }
  return 0;
}

/**
 * @function detectAndDisplay
 */
void
detectAndDisplay(cv::Mat frame) {
  std::vector<cv::Rect> faces;
  cv::Mat frame_gray;

  cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
  cv::equalizeHist(frame_gray, frame_gray);

  //-- Detect faces
  face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0, cv::Size(80, 80));

  for(size_t i = 0; i < faces.size(); i++) {
    cv::Mat faceROI = frame_gray(faces[i]);
    std::vector<cv::Rect> eyes;

    //-- In each cv::face, detect eyes
    eyes_cascade.detectMultiScale(faceROI, eyes, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
    if(eyes.size() == 2) {
      //-- Draw the face
      cv::Point center(faces[i].x + faces[i].width / 2, faces[i].y + faces[i].height / 2);
      cv::ellipse(frame, center, cv::Size(faces[i].width / 2, faces[i].height / 2), 0, 0, 360, cv::Scalar(255, 0, 0), 2, 8, 0);

      for(size_t j = 0; j < eyes.size(); j++) {
        //-- Draw the eyes
        cv::Point eye_center(faces[i].x + eyes[j].x + eyes[j].width / 2, faces[i].y + eyes[j].y + eyes[j].height / 2);
        int radius = cvRound((eyes[j].width + eyes[j].height) * 0.25);
        cv::circle(frame, eye_center, radius, cv::Scalar(255, 0, 255), 3, 8, 0);
      }
    }
  }
  //-- Show what you got
  cv::imshow(window_name, frame);
}
