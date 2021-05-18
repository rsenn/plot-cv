//
//  trackFilteredObject.cpp
//  main
//
//  Created by 陳家麒 on 2017/8/4.
//
//
/*
 Green arguments
 Morning
    SMAX 256,
    SMIN 150~127,
    HMAX 256,
    HMIN 17~23,
    VMAX 170~182,
    VMIN 36~20
 Night
    HMAX 80
    HMIN 24
    SMAX 256
    SMIN 84
    VMAX 240
    VMIN 113

 */

#include <trackFilteredObject.hpp>
using namespace std;
//using namespace cv;
void static on_trackbar(int, void*) {
  // This function gets called whenever a
  // trackbar position is changed
}

string
trackFiliteredObject::intToString(int number) {

  std::stringstream ss;
  ss << number;
  return ss.str();
}
void
trackFiliteredObject::writeXY(int x, int y) {
  this->x = x;
  this->y = y;
}
void
trackFiliteredObject::writeRange(int Range) {
  this->Range = Range;
}
int
trackFiliteredObject::getX() {
  return this->x;
}
int
trackFiliteredObject::getY() {
  return this->y;
}
int
trackFiliteredObject::getRange() {
  return this->Range;
}

void
trackFiliteredObject::Multiple_inRanage(cv::Mat& hsv, cv::Mat& cv::threshold, int arguments) {

  switch(arguments) {
    case default_value:
      H_MIN = 0;
      H_MAX = 256;
      S_MIN = 0;
      S_MAX = 256;
      V_MIN = 0;
      V_MAX = 256;
      // cv::inRange(hsv,cv::Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX), cv::threshold);
      break;
    case morring:
      H_MIN = 17;
      H_MAX = 256;
      S_MIN = 150;
      S_MAX = 256;
      V_MIN = 36;
      V_MAX = 170;
      // cv::inRange(hsv, cv::Scalar(17,150,36), cv::Scalar(256,256,170), cv::threshold);
      break;
    case morring_pi:
      H_MIN = 26;
      H_MAX = 256;
      S_MIN = 158;
      S_MAX = 256;
      V_MIN = 73;
      V_MAX = 249;
      break;
    case noon: break;
    case morring_demo:
      H_MIN = 24;
      H_MAX = 104;
      S_MIN = 80;
      S_MAX = 170;
      V_MIN = 52;
      V_MAX = 256;
      break;

    case night:
      H_MIN = 24;
      H_MAX = 80;
      S_MIN = 84;
      S_MAX = 256;
      V_MIN = 113;
      V_MAX = 240;
      // cv::inRange(hsv, cv::Scalar(24,84,113), cv::Scalar(80,256,240), cv::threshold);
      break;
    case night2: break;

    default: break;
  }
}
void
trackFiliteredObject::drawObject(int x, int y, cv::Mat& frame) {

  // use some of the openCV drawing functions to draw crosshairs
  // on your tracked image!

  // UPDATE:JUNE 18TH, 2013
  // added 'if' and 'else' statements to prevent
  // memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

  cv::circle(frame, cv::Point(x, y), 20, cv::Scalar(0, 255, 0), 2);
  if(y - 25 > 0)
    cv::line(frame, cv::Point(x, y), cv::Point(x, y - 25), cv::Scalar(0, 255, 0), 2);
  else
    cv::line(frame, cv::Point(x, y), cv::Point(x, 0), cv::Scalar(0, 255, 0), 2);
  if(y + 25 < FRAME_HEIGHT)
    cv::line(frame, cv::Point(x, y), cv::Point(x, y + 25), cv::Scalar(0, 255, 0), 2);
  else
    cv::line(frame, cv::Point(x, y), cv::Point(x, FRAME_HEIGHT), cv::Scalar(0, 255, 0), 2);
  if(x - 25 > 0)
    cv::line(frame, cv::Point(x, y), cv::Point(x - 25, y), cv::Scalar(0, 255, 0), 2);
  else
    cv::line(frame, cv::Point(x, y), cv::Point(0, y), cv::Scalar(0, 255, 0), 2);
  if(x + 25 < FRAME_WIDTH)
    cv::line(frame, cv::Point(x, y), cv::Point(x + 25, y), cv::Scalar(0, 255, 0), 2);
  else
    cv::line(frame, cv::Point(x, y), cv::Point(FRAME_WIDTH, y), cv::Scalar(0, 255, 0), 2);

  cv::putText(frame, intToString(x) + "," + intToString(y), cv::Point(x, y + 30), 1, 1, cv::Scalar(0, 255, 0), 2);
}
void
trackFiliteredObject::trackObjcet(int& x, int& y, cv::Mat cv::threshold, cv::Mat& cameraFeed) {
  cv::Mat temp;
  cv::threshold.copyTo(temp);
  // these two std::vectors need for output findcontours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  // find contours of filtered image using Opencv find Contours function
  cv::findContours(temp, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
  // use cv::moments method to find our filtered object
  double refArea = 0;
  bool objectFound = false;
  if(hierarchy.size() > 0) {
    int numObjects = hierarchy.size();
    // if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
    if(numObjects < MAX_NUM_OBJECTS) {
      for(int index = 0; index >= 0; index = hierarchy[index][0]) {

        cv::Moments moment = cv::moments((cv::Mat)contours[index]);
        double area = moment.m00;

        // if the area is less than 20 px by 20px then it is probably just noise
        // if the area is the same as the 3/2 of the image size, probably just a bad filter
        // we only want the object with the largest area so we safe a reference area each
        // iteration and compare it to the area in the next iteration.
        if(area > MIN_OBJECT_AREA && area < MAX_OBJECT_AREA && area > refArea) {
          x = moment.m10 / area;
          y = moment.m01 / area;

          objectFound = true;
          refArea = area;
        } else if(area > MAX_OBJECT_AREA) {
          cv::putText(cameraFeed, "Tracking TOO CLOSE!", cv::Point(0, 50), 2, 1, cv::Scalar(0, 0, 255), 2);
          cout << "TOO CLOSE" << endl;
          objectFound = false;
        } else
          objectFound = false;
      }
      // let user know you found an object
      if(objectFound == true) {
        cv::putText(cameraFeed, "Tracking Object", cv::Point(0, 50), 2, 1, cv::Scalar(0, 255, 0), 2);
        // cv::write x,y in object
        writeXY(x, y);

        // draw object location on screen
        drawObject(x, y, cameraFeed);
      }

    } else
      cv::putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", cv::Point(0, 50), 1, 2, cv::Scalar(0, 0, 255), 2);
  }
}
void
trackFiliteredObject::createTrackbars() {
  // create window for trackbars

  cv::namedWindow(trackbarWindowName, 0);
  // create memory to store trackbar name on window
  char TrackbarName[50];
  sprintf(TrackbarName, "H_MIN", H_MIN);
  sprintf(TrackbarName, "H_MAX", H_MAX);
  sprintf(TrackbarName, "S_MIN", S_MIN);
  sprintf(TrackbarName, "S_MAX", S_MAX);
  sprintf(TrackbarName, "V_MIN", V_MIN);
  sprintf(TrackbarName, "V_MAX", V_MAX);
  // create trackbars and insert them into window
  // 3 parameters are: the address of the variable that is changing when the trackbar is
  // moved(eg.H_LOW), the max value the trackbar can move (eg. H_HIGH), and the function that is
  // called whenever the trackbar is moved(eg. on_trackbar)
  //                                  ---->    ---->     ---->
  cv::createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar);
  cv::createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar);
  cv::createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar);
  cv::createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar);
  cv::createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar);
  cv::createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar);
}
/*Morphological operations*/
void
trackFiliteredObject::morphOps(cv::Mat& thresh) {
  // create structuring element that will be used to "cv::dilate" and "erod" image.
  // the element chosen here is a 3px by 3px rectangle
  cv::Mat erodeElement = cv::getStructuringElement(MORPH_RECT, cv::Size(3, 3));
  // cv::dilate with larger element so make sure object is nicely visable
  cv::Mat dilateElement = cv::getStructuringElement(MORPH_RECT, cv::Size(8, 8));

  cv::erode(thresh, thresh, erodeElement);
  cv::erode(thresh, thresh, erodeElement);

  cv::dilate(thresh, thresh, dilateElement);
  cv::dilate(thresh, thresh, dilateElement);
}
#ifdef __unix
/*in order to cv::read H,S,V value*/
void
trackFiliteredObject::test_hsv(raspicam::RaspiCam_Cv video) {
  // cv::namedWindow("image");
  cv::namedWindow("testing on HSV");
  cv::namedWindow("camerafeed");

  while(true) {
    video.grab();
    video.retrieve(frame);
    if(frame.empty())
      return;

    frame.copyTo(image_frame);
    cv::cvtColor(image_frame, hsv, cv::COLOR_BGR2HSV);

    Multiple_inRanage(hsv, cv::threshold, morring_pi);
    cv::inRange(hsv, cv::Scalar(H_MIN, S_MIN, V_MIN), cv::Scalar(H_MAX, S_MAX, V_MAX), cv::threshold);

    if(useMorphOps)
      morphOps(cv::threshold);

    if(track)
      trackObjcet(x, y, cv::threshold, image_frame);

    // cv::imshow("image", image_frame);
    cv::imshow("testing on HSV", cv::threshold);
    cv::imshow("camerafeed", image_frame);

    char c = (char)cv::waitKey(1000 / 15.0);
    if(c == 27)
      break;
  }
}

#elif __APPLE__
/*in order to cv::read H,S,V value*/
void
trackFiliteredObject::test_hsv(cv::VideoCapture video) {
  cv::namedWindow("image");
  cv::namedWindow("testing on HSV");
  cv::namedWindow("camerafeed");

  while(true) {
    video.grab();
    video.retrieve(frame);
    if(frame.empty())
      return;

    frame.copyTo(image_frame);
    cv::cvtColor(image_frame, hsv, cv::COLOR_BGR2HSV);

    Multiple_inRanage(hsv, cv::threshold, night);
    cv::inRange(hsv, cv::Scalar(H_MIN, S_MIN, V_MIN), cv::Scalar(H_MAX, S_MAX, V_MAX), cv::threshold);

    if(useMorphOps)
      morphOps(cv::threshold);

    if(track)
      trackObjcet(x, y, cv::threshold, image_frame);

    // cv::imshow("image", image_frame);
    cv::imshow("testing on HSV", cv::threshold);
    cv::imshow("camerafeed", image_frame);

    char c = (char)cv::waitKey(1000 / 15.0);
    if(c == 27)
      break;
  }
}
#else

#endif
