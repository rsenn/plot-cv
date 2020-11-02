/*
 * FaceDetector.cpp
 *
 *  Created on: 20/05/2017
 *  Author: Luis Rocha
 */

#include "FaceDetector.h"

using namespace std;
using namespace cv;

/* FACE_XML_CLASSIFIER is set in the cmake */
const string face_cascade_name = FACE_XML_CLASSIFIER;

void
DetectAndDisplayFaces(Mat frame, CascadeClassifier face_cascade) {
  vector<Rect> faces;
  Mat frame_gray;
  string text;
  stringstream sstm;

  /* Current Face Variables */
  Rect CurrentRegion;
  uint8_t CurrentFaceIndex = 0;
  uint32_t CurrentFaceArea = 0;

  /* Biggest Face Variables */
  Rect BiggestRegion;
  int16_t BiggestFaceIndex = 0;
  uint32_t BiggestFaceArea = 0;

  cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
  equalizeHist(frame_gray, frame_gray);

  /* Face dectection algorithm */
  face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));

  /* Add the rectangle to the faces detected */
  for(CurrentFaceIndex = 0; CurrentFaceIndex < faces.size(); CurrentFaceIndex++) {
    CurrentRegion.x = faces[CurrentFaceIndex].x;
    CurrentRegion.y = faces[CurrentFaceIndex].y;
    CurrentRegion.width = faces[CurrentFaceIndex].width;
    CurrentRegion.height = faces[CurrentFaceIndex].height;

    /* Calculate the area of the current face detected */
    CurrentFaceArea = CurrentRegion.width * CurrentRegion.height;

    /* Discover the biggest face area detected */
    if(CurrentFaceArea > BiggestFaceArea) {
      BiggestFaceIndex = CurrentFaceIndex;
      BiggestFaceArea = CurrentFaceArea;
    }

    /* Creation of the two points with the face detected coordenates */
    Point pt1(faces[CurrentFaceIndex].x, faces[CurrentFaceIndex].y);
    Point pt2((faces[CurrentFaceIndex].x + faces[CurrentFaceIndex].height), (faces[CurrentFaceIndex].y + faces[CurrentFaceIndex].width));

    /* Create a green rectangle box to attach to the frame */
    rectangle(frame, pt1, pt2, Scalar(0, 255, 0), 2, 8, 0);
  }

  if(faces.size() > 0) {
    /*If inside this it means that at least one face was detected */
    BiggestRegion.width = faces[BiggestFaceIndex].width;
    BiggestRegion.height = faces[BiggestFaceIndex].height;

    text = "Bigest face area: " + to_string(BiggestRegion.width) + "x" + to_string(BiggestRegion.height);
    printf("Face detected\n");
  }

  /* Add the text to the frame */
  putText(frame, text, cvPoint(30, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 255), 1, CV_AA);

  /* Show the image captured with green rectangles on the faces detected */
  imshow("Raspberry Pi Camera", frame);
  /* Give time to OpenCV to update the image being displayed */
  waitKey(30);
}

void
FaceDetectorThread(StopProgram& condition) {
  CascadeClassifier face_cascade;
  Mat frame;

  /* Open the camera for the video stream capture */
  VideoCapture capture(0);

  /* Reduce the size of the window and the FPS */
  capture.set(CAP_PROP_FRAME_HEIGHT, 500);
  capture.set(CAP_PROP_FRAME_WIDTH, 400);
  capture.set(CAP_PROP_FPS, 3);

  /* Check if succeeded camera openning */
  if(!capture.isOpened()) {
    printf("Could not open Camera\n");
    return;
  }

  /* Load the XML face classifier */
  if(!face_cascade.load(face_cascade_name)) {
    printf("Could not load the xml classifier\n");
    return;
  }

  while(true) {
    if(condition.get() == false) {
      capture >> frame;

      if(!frame.empty()) {
        /* Apply the classifier to the frame */
        DetectAndDisplayFaces(frame, face_cascade);
      } else {
        printf("No captured frame\n");
      }
    } else {
      /* Finish the execution of this thread */
      break;
    }
  }
}
