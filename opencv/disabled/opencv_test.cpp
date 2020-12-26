//**** Written by Eddie Strandberg ****//

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/features2d/features2d.hpp"

#include <iostream>
#include <boost/thread.hpp>

#include <stdio.h>

#include "ServoController.h"
#include "PsVoiceRec.h"

#include <string.h>
#include <vector>

using namespace cv;

#define TURN_ON_FACE_TRACK 1

#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240

int difference[2];

std::vector<Rect> faces;

double min_face_size = 30;
double max_face_size = 70;

int counter = 0;

int
main(int argc, char** argv) {

#if TURN_ON_FACE_TRACK
  VideoCapture cap(-1);
  if(!cap.isOpened()) {
    std::cout << "Cannot open camera" << std::endl;
    return -1;
  }

  cap.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
  cap.set(cv::CAP_PROP_FPS, 30);
  cv::Point2f frameCenter(FRAME_WIDTH / 2, FRAME_HEIGHT / 2);
  CascadeClassifier frontalface =
      CascadeClassifier("/home/pi/projects/git/cvtest/classifiers/haarcascade_frontalface_alt2.xml");
  // CascadeClassifier profileface =
  // CascadeClassifier("/home/pi/projects/git/cvtest/classifiers/haarcascade_profileface.xml");

  // namedWindow("Output",cv::WINDOW_AUTOSIZE);

  ServoController* servoController = new ServoController();
  servoController->MovePanServoTo(100);
  servoController->MoveTiltServoTo(120);

#endif

  PsVoiceRec* voiceRecognition = new PsVoiceRec();

  boost::thread voiceThread(&PsVoiceRec::ListenForKeyword, voiceRecognition);

#if TURN_ON_FACE_TRACK
  while(1) {

    Mat frame;

    bool bSuccess = cap.read(frame);
    if(bSuccess) {
      if(counter % 2 == 0) {
        frontalface.detectMultiScale(frame,
                                     faces,
                                     1.2,
                                     2,
                                     0 | cv::CASCADE_SCALE_IMAGE,
                                     Size(min_face_size, min_face_size),
                                     Size(max_face_size, max_face_size));

        if(!faces.empty()) {
          Rect face = faces[0];
          min_face_size = face.width * 0.8;
          max_face_size = face.width * 1.2;
          // rectangle(frame,Point(face.x, face.y),Point(face.x+face.width/2,
          // face.y+face.height/2),Scalar(255,0,0),1,4);
          cv::Point2f faceCenter(face.x + face.width / 2, face.y + face.height / 2);
          difference[0] = (face.x + face.width / 2) - frameCenter.x;
          difference[1] = (face.y + face.height / 2) - frameCenter.y;
          servoController->MovePanServoBy(difference[0] * 0.08);
          servoController->MoveTiltServoBy(-difference[1] * 0.08);
          // putText(frame, boost::to_string(difference[0]) + ", " +
          // boost::to_string(difference[1]), Point(50,50), FONT_HERSHEY_SIMPLEX, 1,
          // Scalar(0,200,200), 4);  putText(frame, boost::to_string(frameCenter.x) + ", " +
          // boost::to_string(frameCenter.y), Point(50,50), FONT_HERSHEY_SIMPLEX, 1,
          // Scalar(0,200,200), 4);
        }
        counter = 0;
      }

      if(faces.empty()) {
        min_face_size = 30;
        max_face_size = 60;
      }
      counter++;
      // imshow("Output", frame);
    }

    if(waitKey(1) == 27) {
      break;
    }
  }
#endif
#if !TURN_ON_FACE_TRACK
  sleep(15);
#endif
  return 0;
}
