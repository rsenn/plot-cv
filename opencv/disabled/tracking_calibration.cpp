#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// using namespace cv;
using namespace std;

string path = ros::package::getPath("rpi_object_tracking");
YAML::Node calibration_data = YAML::LoadFile(path + "/cfg/cv/tracking_calibration.yaml");
string calib = "";

cv::Mat kernel, frame, mask, hsv;

int x_start = 0;
int y_start = 0;
int x_end = 0;
int y_end = 0;

bool sampling = false;
bool getROI = true;
bool saveROI = false;
bool Undistort = false;

int Lower[3] = {calibration_data["h_low"].as<int>(), calibration_data["s_low"].as<int>(), calibration_data["v_low"].as<int>()};

int Upper[3] = {calibration_data["h_up"].as<int>(), calibration_data["s_up"].as<int>(), calibration_data["v_up"].as<int>()};

double lower[3], upper[3];
int intLow[3], intUp[3];

static void
leftClick(int event, int x, int y, int, void*) {
  if(event == EVENT_LBUTTONDOWN) {
    x_start = x;
    y_start = y;
    x_end = x;
    y_end = y;
    sampling = true;
    saveROI = false;
    getROI = false;
  } else if(event == EVENT_MOUSEMOVE) {
    if(sampling) {
      x_end = x;
      y_end = y;
    }
  } else if(event == EVENT_LBUTTONUP) {
    x_end = x;
    y_end = y;
    if(x_start > x_end) {
      int x_temp = x_start;
      x_start = x_end;
      x_end = x_temp;
    }
    if(y_start > y_end) {
      int y_temp = y_start;
      y_start = y_end;
      y_end = y_temp;
    }

    if(x_start == x_end || y_start == y_end)
      saveROI = false;
    else
      saveROI = true;
    sampling = false;
  }
}

void
on_trackbar(int, void*) {
  ofstream fo(path + "/cfg/cv/tracking_calibration.yaml");
  if(!fo.is_open()) {
    cout << "unable to save calibration data to " << path << endl;
  } else {
    calibration_data["h_low"] = intLow[0];
    calibration_data["h_up"] = intUp[0];
    calibration_data["s_low"] = intLow[1];
    calibration_data["s_up"] = intUp[1];
    calibration_data["v_low"] = intLow[2];
    calibration_data["v_up"] = intUp[2];

    fo << calibration_data;
    fo.close();
  }
}

void
imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    frame = cv_bridge::toCvShare(msg, "8UC3")->image;

    if(!getROI) {
      if(!sampling && !saveROI) {
        cv::imshow("frame", frame);
      } else if(sampling && !saveROI) {
        cv::rectangle(frame, cv::Point(x_start, y_start), cv::Point(x_end, y_end), cv::Scalar(0, 255, 0), 2);
        cv::imshow("frame", frame);
      } else if(saveROI) {
        cv::rectangle(frame, cv::Point(x_start, y_start), cv::Point(x_end, y_end), cv::Scalar(0, 255, 0), 2);
        cv::imshow("frame", frame);

        cv::Mat roi = frame(Range(y_start, y_end), Range(x_start, x_end));
        cv::Mat hsvRoi, splitHsv[3];
        cv::cvtColor(roi, hsvRoi, COLOR_BGR2HSV);
        cv::split(hsvRoi, splitHsv);
        cv::minMaxLoc(splitHsv[0], &lower[0], &upper[0]);
        cv::minMaxLoc(splitHsv[1], &lower[1], &upper[1]);
        cv::minMaxLoc(splitHsv[2], &lower[2], &upper[2]);

        cv::setTrackbarPos("H Low", "trackbar", (int)lower[0]);
        cv::setTrackbarPos("S Low", "trackbar", (int)lower[1]);
        cv::setTrackbarPos("V Low", "trackbar", (int)lower[2]);

        cv::setTrackbarPos("H Up", "trackbar", (int)upper[0]);
        cv::setTrackbarPos("S Up", "trackbar", (int)upper[1]);
        cv::setTrackbarPos("V Up", "trackbar", (int)upper[2]);

        saveROI = false;
        getROI = true;
      }
    } else {
      for(int i = 0; i < 3; i++) {
        lower[i] = (double)intLow[i];
        upper[i] = (double)intUp[i];
      }

      cv::cvtColor(frame, hsv, COLOR_BGR2HSV);
      cv::inRange(hsv, cv::Scalar(lower[0], lower[1], lower[2]), cv::Scalar(upper[0], upper[1], upper[2]), mask);
      cv::erode(mask, mask, 0, cv::Point(-1, -1), 2);
      cv::dilate(mask, mask, 0, cv::Point(-1, -1), 2);
      cv::morphologyEx(mask, mask, MORPH_CLOSE, kernel);

      // Find Contours
      cv::Mat maskColone = mask.clone();
      std::vector<std::vector<cv::Point>> cnts;
      cv::findContours(maskColone, cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

      // Drawing
      if(cnts.size() > 0) {
        cv::Moments M;
        cv::Point2f center;
        float radius;
        int largest_area = 0;
        int largest_contour_index = 0;

        for(int i = 0; i < cnts.size(); i++) {
          double a = cv::contourArea(cnts[i], false);
          if(a > largest_area) {
            largest_area = a;
            largest_contour_index = i;
          }
        }

        cv::minEnclosingCircle(cnts[largest_contour_index], center, radius);
        M = cv::moments(cnts[largest_contour_index]);

        if(radius > 0) {
          cv::circle(frame, center, int(radius), cv::Scalar(0, 255, 0), 2);
        }
      }

      cv::imshow("mask", mask);
      cv::imshow("frame", frame);
    }

    if((char)27 == (char)cv::waitKey(1))
      ros::shutdown();
  } catch(cv_bridge::Exception& e) { ROS_ERROR("Could not convert from '%s' to '8UC3'.", msg->encoding.c_str()); }
}

int
main(int argc, char* argv[]) {
  ros::init(argc, argv, "tracking_calibration");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber camSub = it.subscribe("raspi_cam_img", 1, imageCallback);

  kernel = cv::Mat::ones(1, 1, CV_8UC1);

  for(int i = 0; i < 3; i++) {
    intLow[i] = Lower[i];
    intUp[i] = Upper[i];
  }

  cv::namedWindow("frame");
  cv::namedWindow("mask");
  cv::namedWindow("trackbar", cv::WINDOW_FREERATIO);
  cv::setMouseCallback("frame", leftClick);

  cv::createTrackbar("H Low", "trackbar", &intLow[0], 255, on_trackbar);
  cv::createTrackbar("S Low", "trackbar", &intLow[1], 255, on_trackbar);
  cv::createTrackbar("V Low", "trackbar", &intLow[2], 255, on_trackbar);

  cv::createTrackbar("H Up", "trackbar", &intUp[0], 255, on_trackbar);
  cv::createTrackbar("S Up", "trackbar", &intUp[1], 255, on_trackbar);
  cv::createTrackbar("V Up", "trackbar", &intUp[2], 255, on_trackbar);

  ros::spin();
  cv::destroyAllWindows();
  return 0;
}