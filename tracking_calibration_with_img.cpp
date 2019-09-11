#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

string path = ros::package::getPath("rpi_object_tracking");
YAML::Node calibration_data = YAML::LoadFile(path + "/cfg/cv/tracking_calibration.yaml");
string calib = "";

Mat kernel, frame, mask, hsv;

int x_start = 0;
int y_start = 0;
int x_end = 0;
int y_end = 0;

bool sampling = false;
bool getROI = true;
bool saveROI = false;
bool Undistort = false;

int Lower[3] = {
    calibration_data["h_low"].as<int>(), 
    calibration_data["s_low"].as<int>(), 
    calibration_data["v_low"].as<int>()
};

int Upper[3] = {
    calibration_data["h_up"].as<int>(), 
    calibration_data["s_up"].as<int>(), 
    calibration_data["v_up"].as<int>()
};

double lower[3], upper[3];
int intLow[3], intUp[3];

static void leftClick( int event, int x, int y, int, void* ) {
  if (event == EVENT_LBUTTONDOWN) {
    x_start = x;
    y_start = y;
    x_end = x;
    y_end = y;
    sampling = true;
    saveROI = false;
    getROI = false;
  }
  else if (event == EVENT_MOUSEMOVE) {
    if (sampling) {
      x_end = x;
      y_end = y;
    }
  }
  else if (event == EVENT_LBUTTONUP) {
    x_end = x;
    y_end = y;
    if (x_start > x_end) {
      int x_temp = x_start;
      x_start = x_end;
      x_end = x_temp;
    }
    if (y_start > y_end) {
      int y_temp = y_start;
      y_start = y_end;
      y_end = y_temp;
    }

    if (x_start == x_end || y_start == y_end) saveROI = false;
    else saveROI = true;
    sampling = false;
  }
}

void on_trackbar(int, void*) {
  ofstream fo(path + "/cfg/cv/tracking_calibration.yaml");
  if (!fo.is_open()) {
      cout << "unable to save calibration data to " << path << endl;
  }
  else {
    calibration_data["h_low"] = intLow[0]; calibration_data["h_up"] = intUp[0];
    calibration_data["s_low"] = intLow[1]; calibration_data["s_up"] = intUp[1];
    calibration_data["v_low"] = intLow[2]; calibration_data["v_up"] = intUp[2];

    fo << calibration_data;
    fo.close();
  }
}

int main(int argc, char* argv[]) {
  kernel = Mat::ones(1, 1, CV_8UC1);

  for (int i = 0; i < 3; i++) {
    intLow[i] = Lower[i];
    intUp[i] = Upper[i];
  } 

  namedWindow("frame");
  namedWindow("mask");
  namedWindow("trackbar", CV_WINDOW_FREERATIO);
  setMouseCallback("frame", leftClick);

  createTrackbar("H Low", "trackbar", &intLow[0], 255, on_trackbar);
  createTrackbar("S Low", "trackbar", &intLow[1], 255, on_trackbar);
  createTrackbar("V Low", "trackbar", &intLow[2], 255, on_trackbar);

  createTrackbar("H Up", "trackbar", &intUp[0], 255, on_trackbar);
  createTrackbar("S Up", "trackbar", &intUp[1], 255, on_trackbar);
  createTrackbar("V Up", "trackbar", &intUp[2], 255, on_trackbar);

  for(;;) {
    frame = imread(path + "/image_capture/image.jpg", CV_LOAD_IMAGE_COLOR);

    if (!getROI) {
      if (!sampling && !saveROI) {
        imshow("frame", frame);
      }
      else if (sampling && !saveROI) {
        rectangle(frame, Point(x_start, y_start), Point(x_end, y_end), Scalar(0, 255, 0), 2);
        imshow("frame", frame);
      }
      else if (saveROI) {
        rectangle(frame, Point(x_start, y_start), Point(x_end, y_end), Scalar(0, 255, 0), 2);
        imshow("frame", frame);

        Mat roi = frame(Range(y_start, y_end), Range(x_start, x_end));
        Mat hsvRoi, splitHsv[3];
        cvtColor(roi, hsvRoi, COLOR_BGR2HSV);
        split(hsvRoi, splitHsv);
        minMaxLoc(splitHsv[0], &lower[0], &upper[0]);
        minMaxLoc(splitHsv[1], &lower[1], &upper[1]);
        minMaxLoc(splitHsv[2], &lower[2], &upper[2]);

        setTrackbarPos("H Low", "trackbar", (int)lower[0]);
        setTrackbarPos("S Low", "trackbar", (int)lower[1]);
        setTrackbarPos("V Low", "trackbar", (int)lower[2]);

        setTrackbarPos("H Up", "trackbar", (int)upper[0]);
        setTrackbarPos("S Up", "trackbar", (int)upper[1]);
        setTrackbarPos("V Up", "trackbar", (int)upper[2]);

        saveROI = false;
        getROI = true;
      }
    }
    else {
      for (int i = 0; i < 3; i++) {
          lower[i] = (double)intLow[i];
          upper[i] = (double)intUp[i];
      }

      cvtColor(frame, hsv, COLOR_BGR2HSV);
      inRange(hsv, Scalar(lower[0],lower[1],lower[2]), Scalar(upper[0],upper[1],upper[2]), mask);
      erode(mask, mask, 0, Point(-1,-1), 2);
      dilate(mask, mask, 0, Point(-1, -1), 2);
      morphologyEx(mask ,mask, MORPH_CLOSE, kernel);

      //Find Contours
      Mat maskColone = mask.clone();
      vector<vector<Point> > cnts;
      findContours(maskColone, cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

      //Drawing
      if (cnts.size() > 0) {
        Moments M;
        Point2f center;
        float radius;
        int largest_area = 0;
        int largest_contour_index = 0;

        for (int i = 0; i < cnts.size(); i++) {
          double a = contourArea(cnts[i], false);
          if (a > largest_area) {
            largest_area = a;
            largest_contour_index = i;
          }
        }

        minEnclosingCircle(cnts[largest_contour_index], center, radius);
        M = moments(cnts[largest_contour_index]);

        if (radius > 0) {
          circle(frame, center, int(radius), Scalar(0, 255, 0), 2);
        }
      }

      imshow("mask", mask);
      imshow("frame", frame);
    }

    if ((char)27 == (char)waitKey(1)) break;
  }

  destroyAllWindows();
  return 0;
}