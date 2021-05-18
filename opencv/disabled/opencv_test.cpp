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

//using namespace cv;

#define TURN_ON_FACE_TRACK 1

#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240

int difference[2];

std::vector<cv::Rect> faces;

double min_face_size = 30;
double max_face_size = 70;

int counter = 0;

int
main(int argc, char** argv) {

#if TURN_ON_FACE_TRACK
  cv::VideoCapture cap(-1);
  if(!cap.isOpened()) {
    std::cout << "Cannot open camera" << std::endl;
    return -1;
  }

  cap.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
  cap.set(cv::CAP_PROP_FPS, 30);
  cv::Point2f frameCenter(FRAME_WIDTH / 2, FRAME_HEIGHT / 2);
  cv::CascadeClassifier frontalface =
      cv::CascadeClassifier("/home/pi/projects/git/cvtest/classifiers/haarcascade_frontalface_alt2.xml");
  // cv::CascadeClassifier profileface =
  // cv::CascadeClassifier("/home/pi/projects/git/cvtest/classifiers/haarcascade_profileface.xml");

  // cv::namedWindow("Output",cv::WINDOW_AUTOSIZE);

  ServoController* servoController = new ServoController();
  servoController->MovePanServoTo(100);
  servoController->MoveTiltServoTo(120);

#endif

  PsVoiceRec* voiceRecognition = new PsVoiceRec();

  boost::thread voiceThread(&PsVoiceRec::ListenForKeyword, voiceRecognition);

#if TURN_ON_FACE_TRACK
  while(1) {

    cv::Mat frame;

    bool bSuccess = cap.cv::read(frame);
    if(bSuccess) {
      if(counter % 2 == 0) {
        frontalface.detectMultiScale(frame,
                                     faces,
                                     1.2,
                                     2,
                                     0 | cv::CASCADE_SCALE_IMAGE,
                                     cv::Size(min_face_size, min_face_size),
                                     cv::Size(max_face_size, max_face_size));

        if(!faces.empty()) {
          cv::Rect cv::face = faces[0];
          min_face_size = cv::face.width * 0.8;
          max_face_size = cv::face.width * 1.2;
          // cv::rectangle(frame,cv::Point(cv::face.x, cv::face.y),Point(face.x+face.width/2,
          // cv::face.y+face.height/2),cv::Scalar(255,0,0),1,4);
          cv::Point2f faceCenter(cv::face.x + cv::face.width / 2, cv::face.y + cv::face.height / 2);
          difference[0] = (cv::face.x + cv::face.width / 2) - frameCenter.x;
          difference[1] = (cv::face.y + cv::face.height / 2) - frameCenter.y;
          servoController->MovePanServoBy(difference[0] * 0.08);
          servoController->MoveTiltServoBy(-difference[1] * 0.08);
          // cv::putText(frame, boost::to_string(difference[0]) + ", " +
          // boost::to_string(difference[1]), cv::Point(50,50), FONT_HERSHEY_SIMPLEX, 1,
          // cv::Scalar(0,200,200), 4);  cv::putText(frame, boost::to_string(frameCenter.x) + ", " +
          // boost::to_string(frameCenter.y), cv::Point(50,50), FONT_HERSHEY_SIMPLEX, 1,
          // cv::Scalar(0,200,200), 4);
        }
        counter = 0;
      }

      if(faces.empty()) {
        min_face_size = 30;
        max_face_size = 60;
      }
      counter++;
      // cv::imshow("Output", frame);
    }

    if(cv::waitKey(1) == 27) {
      break;
    }
  }
#endif
#if !TURN_ON_FACE_TRACK
  sleep(15);
#endif
  return 0;
}
