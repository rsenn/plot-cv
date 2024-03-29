// motionTracking.cpp

// Written by  Kyle Hounslow, January 2014
// modified by Brian Gravelle to track multiple objects - August 2016

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>

using namespace std;
// using namespace cv;

// our sensitivity value to be used in the cv::threshold() function
static int SENSITIVITY_VALUE = 50; // original 20
// size of cv::blur used to smooth the image to remove possible noise and
// increase the size of the object we are trying to track. (Much like cv::dilate and cv::erode)
static int BLUR_SIZE = 200; // original 10
static double MIN_OBJ_AREA = 1000;
// we'll have just one object to search for
// and keep track of its position.
int theObject[2] = {0, 0};
// cv::Rect objectBoundingRectangle = cv::Rect(0,0,0,0);

void searchForMovement(cv::Mat thresholdImage, cv::Mat& cameraFeed);
int char_to_int(char* c);
string intToString(int number);
void show_help();

int
main(int argc, char** argv) {

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
  // if frame reads work or fail
  bool success;

  if(argc < 2)
    show_help();

  string vid_name = argv[1];

  if(argc > 2)
    SENSITIVITY_VALUE = char_to_int(argv[2]);

  if(argc > 3)
    BLUR_SIZE = char_to_int(argv[3]);

  if(argc > 4)
    MIN_OBJ_AREA = char_to_int(argv[4]);

  cv::namedWindow("Frame1", cv::WINDOW_NORMAL);

  while(1) {

    // we can loop the video by re-opening the capture every time the video reaches its last frame
    capture.open(vid_name);
    if(!capture.isOpened()) {
      cout << "ERROR ACQUIRING VIDEO FEED\n";
      getchar();
      return -1;
    }

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

    // while( !(frame2.rows == 0 || frame2.cols ==0) ) {
    while(success) {

      // convert frame2 to gray scale for frame differencing
      cv::cvtColor(frame2, grayImage2, COLOR_BGR2GRAY);

      // perform frame differencing with the sequential images. This will output an "intensity
      // image" do not confuse this with a cv::threshold image, we will need to perform thresholding
      // afterwards.
      cv::absdiff(grayImage1, grayImage2, differenceImage);
      // cv::threshold intensity image at a given sensitivity value
      cv::threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
      if(debugMode) {
        // show the difference image and cv::threshold image
        cv::namedWindow("Difference Image", cv::WINDOW_NORMAL);
        cv::imshow("Difference Image", differenceImage);
        cv::resizeWindow("Difference Image", 512, 384);
        cv::namedWindow("Threshold Image", cv::WINDOW_NORMAL);
        cv::imshow("Threshold Image", thresholdImage);
        cv::resizeWindow("Threshold Image", 512, 384);
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

      if(debugMode) {
        // show the cv::threshold image after it's been "blurred"
        cv::namedWindow("Final Threshold Image", cv::WINDOW_NORMAL);
        cv::imshow("Final Threshold Image", thresholdImage);
        cv::resizeWindow("Final Threshold Image", 512, 384);
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
      cv::resizeWindow("Frame1", 512, 384);
      // check to see if a button has been pressed.
      // this 10ms delay is necessary for proper operation of this program
      // if removed, frames will not have enough time to referesh and a blank
      // image will appear.
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
      } // big switch statement

      frame2.copyTo(frame1);
      // convert frame1 to gray scale for frame differencing
      cv::cvtColor(frame1, grayImage1, COLOR_BGR2GRAY);
      // copy second frame
      success = capture.read(frame2);
    } // inner while loop

    // release the capture before re-opening and looping again.
    capture.release();
  } // outer while loop (infinite)

  return 0;
}

void
searchForMovement(cv::Mat thresholdImage, cv::Mat& cameraFeed) {

  bool objectDetected = false;
  int obj_count = 0, i = 0;
  double obj_area = 0;
  cv::Mat temp;
  Rect2d temp_rect;
  std::vector<Rect2d> obj_rects;
  thresholdImage.copyTo(temp);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  // retrieves external contours
  cv::findContours(temp, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if(contours.size() > 0) {
    i = contours.size() - 1;
    do {
      temp_rect = cv::boundingRect(contours.at(i));
      obj_area = temp_rect.area();

      if(obj_area >= MIN_OBJ_AREA) {
        obj_count++;
        obj_rects.push_back(Rect2d(temp_rect));
      }

      i--;
    } while(i >= 0);
  }

  for(unsigned j = 0; j < obj_rects.size(); j++) {
    cv::rectangle(cameraFeed, obj_rects[j], cv::Scalar(255, 0, 0), 2, 1); // draw cv::rectangle around object
    int mid_x = obj_rects[j].x + (obj_rects[j].width / 2);
    int mid_y = obj_rects[j].y - (obj_rects[j].height / 2);
  }
}

int
char_to_int(char* c) {
  int i = 0;
  int ret = 0;
  int mult = 1;

  while(c[i] != '\0') i++;
  i--;

  for(i; i >= 0; i--) {
    ret += mult * ((int)c[i] - (int)48);
    mult *= 10;
  }
  return ret;
}

// int to string helper function
string
intToString(int number) {
  std::stringstream ss;
  ss << number;
  return ss.str();
}

void
show_help() {
  cout << endl
       << " Usage: ./motionTracking_modified.out <video_name> [SENSITIVITY_VALUE] [BLUR_SIZE] "
          "[MIN_OBJ_AREA]\n"
          " examples:\n"
          " ./motionTracking_modified.out /home/pi/videos/my_vid.h264\n"
          " ./motionTracking_modified.out /home/pi/videos/my_vid.h264 20 10 10\n"
       << endl
       << endl;
  exit(1);
}
