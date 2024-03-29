// motionTracking.cpp
// Written by  Kyle Hounslow, January 2014

#include <opencv/cv.hpp>
//#include <opencv/highgui.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>

using namespace std;
// using namespace cv;

// our sensitivity value to be used in the cv::threshold() function
const static int SENSITIVITY_VALUE = 20;
// size of cv::blur used to smooth the image to remove possible noise and
// increase the size of the object we are trying to track. (Much like cv::dilate and cv::erode)
const static int BLUR_SIZE = 10;
// we'll have just one object to search for
// and keep track of its position.
int theObject[2] = {0, 0};
// bounding cv::rectangle of the object, we will use the center of this as its position.
Rect2d objectBoundingRectangle = Rect2d(0, 0, 0, 0);
// cv::Rect objectBoundingRectangle = cv::Rect(0,0,0,0);

// int to string helper function
string
intToString(int number) {

  // this function has a number input and string output
  std::stringstream ss;
  ss << number;
  return ss.str();
}

void
searchForMovement(cv::Mat thresholdImage, cv::Mat& cameraFeed) {
  // notice how we use the '&' cv::operator for the cameraFeed. This is because we wish
  // to take the values passed into the function and manipulate them, rather than just working with
  // a copy. eg. we draw to the cameraFeed in this function which is then displayed in the main()
  // function.
  bool objectDetected = false;
  cv::Mat temp;
  thresholdImage.copyTo(temp);
  // these two std::vectors needed for output of findContours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  // find contours of filtered image using openCV cv::findContours function
  // cv::findContours(temp,contours,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_SIMPLE );// retrieves all
  // contours
  cv::findContours(temp, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE); // retrieves external contours

  // if contours std::vector is not empty, we have found some objects
  if(contours.size() > 0)
    objectDetected = true;
  else
    objectDetected = false;

  if(objectDetected) {
    // the largest contour is found at the end of the contours std::vector
    // we will simply assume that the biggest contour is the object we are looking for.
    std::vector<std::vector<cv::Point>> largestContourVec;
    largestContourVec.push_back(contours.at(contours.size() - 1));
    // make a bounding cv::rectangle around the largest contour then find its centroid
    // this will be the object's final estimated position.
    objectBoundingRectangle = cv::boundingRect(largestContourVec.at(0));
    int xpos = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
    int ypos = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;

    // update the objects positions by changing the 'theObject' array values
    theObject[0] = xpos, theObject[1] = ypos;
  }
  // make some temp x and y variables so we dont have to type out so much
  int x = theObject[0];
  int y = theObject[1];
  // draw some crosshairs on the object
  cv::circle(cameraFeed, cv::Point(x, y), 20, cv::Scalar(0, 255, 0), 2);
  cv::line(cameraFeed, cv::Point(x, y), cv::Point(x, y - 25), cv::Scalar(0, 255, 0), 2);
  cv::line(cameraFeed, cv::Point(x, y), cv::Point(x, y + 25), cv::Scalar(0, 255, 0), 2);
  cv::line(cameraFeed, cv::Point(x, y), cv::Point(x - 25, y), cv::Scalar(0, 255, 0), 2);
  cv::line(cameraFeed, cv::Point(x, y), cv::Point(x + 25, y), cv::Scalar(0, 255, 0), 2);
  cv::putText(cameraFeed,
              "Tracking object at (" + intToString(x) + "," + intToString(y) + ")",
              cv::Point(x, y),
              1,
              1,
              cv::Scalar(255, 0, 0),
              2);
}
int
main() {

  int key = 0;
  // some boolean variables for added functionality
  bool objectDetected = false;
  // these two can be toggled by pressing 'd' or 't'
  bool debugMode = false;
  bool trackingEnabled = false;
  // pause and resume code
  bool pause = false;
  // set up the matrices that we will need
  // the two frames we will be comparing
  cv::Mat frame1, frame2;
  // their grayscale images (needed for cv::absdiff() function)
  cv::Mat grayImage1, grayImage2;
  // resulting difference image
  cv::Mat differenceImage;
  // thresholded difference image (for use in cv::findContours() function)
  cv::Mat thresholdImage;
  // video capture object.
  cv::VideoCapture capture;

  while(1) {

    // we can loop the video by re-opening the capture every time the video reaches its last frame

    capture.open("bouncingBall.avi");

    if(!capture.isOpened()) {
      cout << "ERROR ACQUIRING VIDEO FEED\n";
      getchar();
      return -1;
    }

    // check if the video has reach its last frame.
    // we cv::add '-1' because we are reading two frames from the video at a time.
    // if this is not included, we get a memory cv::error!
    while(capture.get(cv::CAP_PROP_POS_FRAMES) < capture.get(cv::CAP_PROP_FRAME_COUNT) - 1) {
      bool success;
      // cv::read first frame
      success = capture.read(frame1);
      if(!success) {
        cout << endl << "ERROR: frame 1 failed to be cv::read" << endl;
        exit(1);
      }
      // convert frame1 to gray scale for frame differencing
      cv::cvtColor(frame1, grayImage1, COLOR_BGR2GRAY);
      // copy second frame
      success = capture.read(frame2);
      if(!success) {
        cout << endl << "ERROR: frame 2 failed to be cv::read" << endl;
        exit(1);
      }
      // convert frame2 to gray scale for frame differencing
      cv::cvtColor(frame2, grayImage2, COLOR_BGR2GRAY);

      // perform frame differencing with the sequential images. This will output an "intensity
      // image" do not confuse this with a cv::threshold image, we will need to perform thresholding
      // afterwards.
      cv::absdiff(grayImage1, grayImage2, differenceImage);
      // cv::threshold intensity image at a given sensitivity value
      cv::threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
      (differenceImage, thresholdImage, cv::Size(BLUR_SIZE, BLUR_SIZE));
      if(debugMode == true) {
        // show the difference image and cv::threshold image
        cv::imshow("Difference Image", differenceImage);
        cv::imshow("Threshold Image", thresholdImage);
      } else {
        // if not in debug mode, destroy the windows so we don't see them anymore
        cv::destroyWindow("Difference Image");
        cv::destroyWindow("Threshold Image");
      }
      // use cv::blur() to smooth the image, remove possible noise and
      // increase the size of the object we are trying to track. (Much like cv::dilate and cv::erode)
      cv::blur(thresholdImage, thresholdImage, cv::Size(BLUR_SIZE, BLUR_SIZE));

      // cv::threshold again to obtain binary image from cv::blur output
      cv::threshold(thresholdImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);

      if(debugMode == true) {
        // show the cv::threshold image after it's been "blurred"
        cv::imshow("Final Threshold Image", thresholdImage);
      } else {
        // if not in debug mode, destroy the windows so we don't see them anymore
        cv::destroyWindow("Final Threshold Image");
      }

      // if tracking enabled, search for contours in our thresholded image
      if(trackingEnabled) {
        searchForMovement(thresholdImage, frame1);
      }

      // show our captured frame
      cv::imshow("Frame1", frame1);
      // check to see if a button has been pressed.
      // this 10ms delay is necessary for proper operation of this program
      // if removed, frames will not have enough time to referesh and a blank
      // image will appear.
      // key = cv::waitKey(10);
      // cout << key << endl;
      // switch(key){
      switch(cv::waitKey(10)) {
        case 1048603:
          // case 27: //'esc' key has been pressed, exit program.
          return 0;
        case 1048692:
          // case 116: //'t' has been pressed. this will toggle tracking
          trackingEnabled = !trackingEnabled;
          if(trackingEnabled == false)
            cout << "Tracking disabled." << endl;
          else
            cout << "Tracking enabled." << endl;
          break;
        case 1048676:
          // case 100: //'d' has been pressed. this will debug mode
          debugMode = !debugMode;
          if(debugMode == false)
            cout << "Debug mode disabled." << endl;
          else
            cout << "Debug mode enabled." << endl;
          break;
        case 1048688:
          // case 112: //'p' has been pressed. this will pause/resume the code.
          pause = !pause;
          if(pause == true) {
            cout << "Code paused, press 'p' again to resume" << endl;
            while(pause == true) {
              // stay in this loop until
              switch(cv::waitKey()) {
                // a switch statement inside a switch statement? Mind blown.
                case 1048688:
                  // case 112:
                  // change pause back to false
                  pause = false;
                  cout << "Code resumed." << endl;
                  break;
              }
            }
          }
      }
    }
    // release the capture before re-opening and looping again.
    capture.release();
  }

  return 0;
}