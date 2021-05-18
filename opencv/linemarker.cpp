#include <linemarker.h>
#include <mainwindow.h>
#include <ui_mainwindow.h>
#include <janeladecalibragem.h>
#include <QWidget>
#include <cv.h>
#include <highgui.h>
#include <camera.h>
#include <linemarkercontrols.h>
#include <iostream>
#include <string>

int cameraIndexh;

// to be used during measuring
lineSet markedLines;

// Line marking window name
string lineMarkerName = "Line marker";

cv::Mat cv::img, workingImage;
string draft = "draft.jpg";
bool isMarkerNoob = true;

// While the program is running, the webcam is always on

cv::VideoCapture capt(2);
cv::VideoCapture capo(1);
cv::VideoCapture capz(0);

// Bools to control if user is still
// marking lines and what cv::line the user is marking
bool wichLine = false;  // false=bottom, true=top
bool isMarking = false; // is marking lines?

LineMarker::LineMarker(int camIndex) {
  cameraIndexh = camIndex;
}

void
LineMarker::chooseLine(bool top) {
  wichLine = top;
}

void
LineMarker::stopMarking() {
  isMarking = false;
}

// Mouse callback event to mark the lines
void
onMouse(int event, int x, int y, int flags, void* userdata) {
  if(event == cv::EVENT_LBUTTONDOWN) {
    /*
    //sets upper line
    cv::img = cv::imread(draft);
    aLine templine(0, y, cv::img.size().width, y);
    templine.drawaLine(cv::img, cv::Scalar(0, 0, 255));
    cv::imshow(lineMarkerName, cv::img);
    markedLines.setLine(1,templine);
    */
    if(isMarking) {
      if(wichLine) {
        cv::img = cv::imread(draft);
        aLine templine(0, y, cv::img.size().width, y);
        templine.drawaLine(cv::img, cv::Scalar(0, 0, 255));
        cv::imshow(lineMarkerName, cv::img);
        markedLines.setLine(1, templine);
      } else if(!wichLine) {
        cv::img = cv::imread(draft);
        aLine templine(0, y, cv::img.size().width, y);
        templine.drawaLine(cv::img, cv::Scalar(0, 0, 255));
        cv::imshow(lineMarkerName, cv::img);
        markedLines.setLine(0, templine);
      }
    }
  } else if(event == cv::EVENT_RBUTTONDOWN) {
    // sets bottom line
    // cv::img = cv::imread(draft);
    // aLine templine(0, y, cv::img.size().width, y);
    // templine.drawaLine(cv::img, cv::Scalar(0, 0, 255));
    // cv::imshow(lineMarkerName, cv::img);
    // markedLines.setLine(0,templine);
  } else if(event == cv::EVENT_MBUTTONDOWN) {
    // take another picture
    // cv::destroyWindow(lineMarkerName);
    // cv::img = cv::imread(draft);
    // aLine templine(0, y, cv::img.size().width, y);
    // templine.drawaLine(cv::img, cv::Scalar(0, 0, 255));
    // cv::imshow(lineMarkerName, cv::img);
    // markedLines.setLine(1,templine);
  } else if(event == cv::EVENT_MOUSEMOVE) {
    // draws mouse line
    cv::img = cv::imread(draft);
    aLine templine(0, y, cv::img.size().width, y);
    templine.drawaLine(cv::img, cv::Scalar(0, 255, 255));
    if(markedLines.hasBottom()) {
      markedLines.set[0].drawaLine(cv::img, cv::Scalar(0, 255, 0));
    }
    if(markedLines.hasTop()) {
      markedLines.set[1].drawaLine(cv::img, cv::Scalar(255, 0, 0));
    }
    cv::imshow(lineMarkerName, cv::img);
  }
}

// Line marking function. User defines a top and a bottom cv::line with the mouse
lineSet
LineMarker::markLines() {

  // Clean markedLines
  lineSet zeroSet;
  markedLines = zeroSet;

  // Read image from file
  cv::img = cv::imread(draft);

  // if fail to cv::read the image
  if(workingImage.empty()) {
    cout << "cv::Error loading the image" << endl;
    // return -1;
  }

  // Create a window
  cv::namedWindow(lineMarkerName, 1);

  // set the callback function for any mouse event
  cv::setMouseCallback(lineMarkerName, onMouse, NULL);

  // show the image
  cv::imshow(lineMarkerName, cv::img);

  // Wait until user press some key
  cv::waitKey(0);

  // cout<<"Top cv::line: "<<markedLines.set[0].p1.y<<" Bottom cv::line: "<<markedLines.set[1].p1.y<<endl;
  // //debug purpose only
  cv::destroyWindow(lineMarkerName);
  return markedLines; // this index will be used to save different lineSets on the fiveLineSets
                      // calibration variable
}

// Open camera to take picture by pressing any key. Picture is used for cv::line marking
lineSet
LineMarker::displayCamera(string windowName) {
  cv::VideoCapture cv::cap;
  if(cameraIndexh == 0) {
    cv::cap = capz;
  } else if(cameraIndexh == 1) {
    cv::cap = capo;
  } else {
    cv::cap = capt;
  }
  cv::namedWindow(windowName);
  cv::Mat frame, messageFrame;

  do {
    cv::cap >> frame;
    cv::imshow(windowName, frame);
  } while(cv::waitKey(10) < 0);

  cv::cap >> frame;

  messageFrame = frame;
  /*
      int fontFace = FONT_HERSHEY_TRIPLEX;
      //int fontFace = SANS_SERIF;
      double fontScale = 0.5;
      int thickness = 1;
      cv::Point textOrg(10, 50);
      string text0 = "Control wich cv::line you're marking on the 'ToolBox'";
      string text1 = "Blue cv::line = Top Line";
      string text2 = "Green cv::line = Bottom Line.";
      string text3 = "Left mouse button to mark a cv::line.";
      cv::putText(messageFrame, text0, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness,8);
      cv::Point textOrg1(20, 70);
      cv::putText(messageFrame, text1, textOrg1, fontFace, fontScale, cv::Scalar::all(255),
     thickness,8); cv::Point textOrg2(20, 90); cv::putText(messageFrame, text2, textOrg2, fontFace,
     fontScale, cv::Scalar::all(255), thickness,8); cv::Point textOrg3(20, 110);
      cv::putText(messageFrame, text3, textOrg3, fontFace, fontScale, cv::Scalar::all(255),
     thickness,8);
  */
  cv::imwrite(draft, frame); // saves image on disk

  cv::destroyWindow(windowName);
  isMarking = true;

  lineMarkerControls* controlBox = new lineMarkerControls;
  controlBox->setTarget(this);
  controlBox->show();
  controlBox->activateWindow();
  controlBox->raise();

  return markLines();
}
