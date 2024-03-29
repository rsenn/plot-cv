/*
 *  targeting.cpp
 *
 *  Created on: Feb 11, 2013
 *      Author: Alec Robinson
 *
 * Allows a user to select pixels cv::samples and have similar pixels detected.
 * Also will now find the centers of various shapes.
 *
 */
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>

// using namespace cv;
using namespace std;

bool editcurrent = false; // is true just after the user selects a pixel. when true, the program
                          // will get the rgb values of the selected pixel

int thresh = 30;
int catnumx[99], catnumy[99];  // x and y of pixels selected by the user
int currentpix = 0;            // number of selected pixels
int bee[99], jee[99], are[99]; // rgb values of selected pixels (at the time of selection)

bool findcenter = true;
int runningx, runningy, avrgx,
    avrgy; // for averaging; running total of x values, y values, and the calculated averages

void my_mouse_callback(int event, int x, int y, int flags, void* param);

namespace {

void
my_mouse_callback(int event, int x, int y, int flags, void* param) {
  IplImage* image = (IplImage*)param;

  switch(event) {

    case cv::EVENT_LBUTTONDOWN:
      catnumx[currentpix] = y;
      catnumy[currentpix] = x;
      currentpix++;
      editcurrent = true;
      break;

    case cv::EVENT_RBUTTONDOWN:
      for(int g = 0; g <= currentpix; g++) {
        catnumx[g] = 0;
        catnumy[g] = 0;
      }
      currentpix = 0;
      break;
  }
}

void
help(char** av) {
  cout << "Usage:\n"
       << av[0] << " <video device number>\n"

       << "\tThis is a program to select pixels and find others similar in color.\n"
       << "\tLeft click in the original image to select pixels.\n"

       << "\tRight click to clear selection.\n"

       << "\tUse 'c' to toggle center finding.\n"

       << "\tThe program captures frames from a camera connected to your computer.\n"
       << "\tTo find the video device number, try ls /dev/video* \n"
       << "\tYou may also pass a video file, like my_vide.avi instead of a device number\n"
       << "\tIf you honestly have no idea what your device number is, I'd try 0.\n"
       << endl;
}

int
process(cv::VideoCapture& capture) {

  cv::namedWindow("Original Image", cv::WINDOW_KEEPRATIO);
  cv::namedWindow("Threshold Image", cv::WINDOW_KEEPRATIO);
  cv::Mat frame;    // mat of the original image
  cv::Mat newframe; // mat of the threshholded (threshheld?) image

  cvCreateTrackbar("Threshold", "Threshold Image", &thresh, 100, NULL);
  for(;;) {

    capture >> newframe; // I'm not certain I actually need this cv::line anymore

    if(editcurrent) { // reads the rgb of the newly selected pixel
      int z = currentpix - 1;
      bee[z] = newframe.at<cv::Vec3b>(catnumx[z], catnumy[z])[0];
      jee[z] = newframe.at<cv::Vec3b>(catnumx[z], catnumy[z])[1];
      are[z] = newframe.at<cv::Vec3b>(catnumx[z], catnumy[z])[2];
      editcurrent = false;
    }

    int found = 0; // number of matches found (used for averaging)
    runningx = 0;
    runningy = 0;
    for(int i = 0; i < frame.rows; i++) {
      for(int j = 0; j < frame.cols; j++) {
        int thisbee = newframe.at<cv::Vec3b>(i, j)[0];
        int thisjee = newframe.at<cv::Vec3b>(i, j)[1];
        int thisare = newframe.at<cv::Vec3b>(i, j)[2];
        int matches = 0;
        for(int h = 0; h <= currentpix; h++) {

          if(catnumx[h] + catnumy[h] != 0)
            if((thisbee < bee[h] + thresh && thisbee > bee[h] - thresh) &&
               (thisjee < jee[h] + thresh && thisjee > jee[h] - thresh) &&
               (thisare < are[h] + thresh && thisare > are[h] - thresh)) { // if it's a match
              newframe.at<cv::Vec3b>(i, j)[0] = 255;                       // set to white
              newframe.at<cv::Vec3b>(i, j)[1] = 255;
              newframe.at<cv::Vec3b>(i, j)[2] = 255;
              if(findcenter && matches == 0) {
                runningx += i;
                runningy += j;
                found++;
              }
              matches++;
            }
        }
        if(matches == 0) {
          newframe.at<cv::Vec3b>(i, j)[0] = 0;
          newframe.at<cv::Vec3b>(i, j)[1] = 0;
          newframe.at<cv::Vec3b>(i, j)[2] = 0;
        }
      }
    }
    if(findcenter) {
      if(found > 0) {
        avrgx = runningx / found; // average the x
        avrgy = runningy / found;
      } // average the y
      runningx = 0;
      runningy = 0;
      cv::Point thecenter = cv::Point(avrgy, avrgx);
      cv::ellipse(newframe, thecenter, cv::Size(20, 20), 0, 0, 360, cv::Scalar(255, 0, 238), 2, 8); // draw circle
    }
    cv::imshow("Threshold Image", newframe);
    capture >> frame;
    if(findcenter) {
      cv::Point thecenter = cv::Point(avrgy, avrgx);
      cv::ellipse(frame, thecenter, cv::Size(20, 20), 0, 0, 360, cv::Scalar(255, 0, 238), 2, 8);
    }

    cv::imshow("Original Image", frame);
    IplImage ipl = frame;
    cvSetMouseCallback("Original Image", my_mouse_callback, (void*)&ipl); // get mouse input

    char key = (char)cv::waitKey(5); // get keyboard input
    if(key == 'q')
      break;
    if(key == 'c') {
      findcenter = !findcenter;
      runningx = 0;
      runningy = 0;
      avrgx = 0;
      avrgy = 0;
    }
  }
  return 0;
}

} // namespace
int
main(int ac, char** av) {

  if(ac != 2) {
    help(av);
    return 1;
  }

  cout << "Left click to collect cv::samples, right click to clear them.\n";
  cout << "'c' to toggle center finding.\n";
  cout << "Press q to quit, but only if you really cv::mean it.\n";

  std::string arg = av[1];
  cv::VideoCapture capture(arg);
  if(!capture.isOpened())
    capture.open(atoi(arg.c_str()));
  capture.set(cv::CAP_PROP_FRAME_WIDTH,
              240); // this cv::line and the following cv::line is necessary only for the raspberry pi.
                    // if you were to delete these on the pi, you would get timeout errors
  capture.set(cv::CAP_PROP_FRAME_HEIGHT, 320);
  if(!capture.isOpened()) {
    cerr << "Failed to open a video device or video file!\n" << endl;
    help(av);
    return 1;
  }
  return process(capture);
}
