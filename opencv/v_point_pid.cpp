#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <time.h>
using namespace std;
using namespace cv;

int
main(int argc, char* argv[]) {
  VideoCapture cap(1);
  cap.set(CAP_PROP_FPS, 60);
  cv::Mat src, hsv, threshold_blue;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<Vec4i> hierarchy;
  short int old_i = 0;
  cv::Point2f center;
  float radius;
  bool ifExist = false;

  ros::init(argc, argv, "middlecv::Point_node");
  ros::NodeHandle nh;
  geometry_msgs::Twist msg_pid_xy;
  ros::Publisher pub_msg_pid_xy = nh.advertise<geometry_msgs::Twist>("pid_xy", 1000000);
  double kp_roll = 0, ki_roll = 0, kd_roll = 0, kp_pitch = 0, ki_pitch = 0, kd_pitch = 0;
  double error_x = 0, error_y = 0;
  double previous_error_x = 0, previous_error_y = 0, integral_x = 0, integral_y = 0, derivative_x = 0, derivative_y = 0;
  double control_value_x = 0, control_value_y = 0;
  double finish_time0 = 0.0;
  ros::param::set("pid_kp_roll", 0);
  ros::param::set("pid_kp_pitch", 0);
  ros::param::set("pid_ki_roll", 0);
  ros::param::set("pid_ki_pitch", 0);
  ros::param::set("pid_kd_roll", 0);
  ros::param::set("pid_kd_pitch", 0);

  ros::Rate r(100000);

  while(ros::ok()) {
    ros::param::getCached("pid_kp_roll", kp_roll);
    ros::param::getCached("pid_kp_pitch", kp_pitch);
    ros::param::getCached("pid_ki_roll", ki_roll);
    ros::param::getCached("pid_ki_pitch", ki_pitch);
    ros::param::getCached("pid_kd_roll", kd_roll);
    ros::param::getCached("pid_kd_pitch", kd_pitch);
    cap >> src;
    hsv = src.clone();
    cvtColor(hsv, hsv, CV_BGR2HSV);
    if(!hsv.empty()) {
      inRange(hsv, Scalar(105, 114, 0), Scalar(120, 255, 255), threshold_blue);
      if(!threshold_blue.empty()) {
        findContours(threshold_blue, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_NONE, cv::Point(0, 0));
        if(!contours.empty()) {
          for(int i = 0; i < contours.size(); i++) {
            if(hierarchy[i][0] == -1) {
              if(hierarchy[i][1] == -1) {
                if(hierarchy[i][2] != -1) {
                  if(hierarchy[i][3] != -1) {
                    line(src, cv::Point(300, 240), cv::Point(340, 240), Scalar(0, 255, 0), 3);
                    line(src, cv::Point(320, 220), cv::Point(320, 260), Scalar(0, 255, 0), 3);
                    old_i = i;
                    ifExist = true;
                  }
                }
              }
            }
          }
          if(ifExist) {
            minEnclosingCircle(contours[old_i], center, radius);
            if(!contours[old_i].empty()) {
              if(radius <= 11) {
                error_x = center.x - 320.0;
                error_y = 240.0 - center.y;
              } else if(radius <= 16) {
                error_x = (center.x - 320.0) / 1.25;
                error_y = (240.0 - center.y) / 1.25;
              } else if(radius <= 21) {
                error_x = (center.x - 320.0) / 2;
                error_y = (240.0 - center.y) / 2;
              } else {
                error_x = (center.x - 320.0) / 10;
                error_y = (240.0 - center.y) / 10;
              }
            }
          } else {
            error_x = contours[0][0].x - 320.0;
            error_y = 240.0 - contours[0][0].y;
          }
          line(src, center, center, Scalar(0, 0, 255), 10);
          ifExist = false;
        }
      }
    }

    double finish_time1 = clock() / 1000000.0;
    // cout << finish_time1 <<endl;
    double dt = finish_time1 - finish_time0;
    // cout << "dt  "<<dt  <<"  0"<< finish_time0<<endl;
    finish_time0 = finish_time1;
    integral_x = integral_x + error_x * dt;
    integral_y = integral_y + error_y * dt;
    derivative_x = (error_x - previous_error_x) / dt;
    derivative_y = (error_y - previous_error_y) / dt;
    // cout<< kd_roll << "  "<<derivative_x<<"  "<<kd_roll*derivative_x<<endl;
    control_value_x = kp_roll * error_x + ki_roll * integral_x + kd_roll * derivative_x;
    control_value_y = kp_pitch * error_y + ki_pitch * integral_y + kd_pitch * derivative_y;
    msg_pid_xy.linear.x = control_value_x;
    msg_pid_xy.linear.y = control_value_y;
    previous_error_x = error_x;
    previous_error_y = error_y;
    // ROS_INFO_STREAM("kp: "<<kp_roll<<" ki: "<<ki_roll<<" kd: "<<kd_roll);
    // ROS_INFO_STREAM("kp: "<<kp_pitch<<" ki: "<<ki_pitch<<" kd: "<<kd_pitch);
    // double cpp_count = clock()/1000000.0;
    // ROS_INFO_STREAM("  "<<cpp_count<<"    "<<ros::Time::now());
    pub_msg_pid_xy.publish(msg_pid_xy);
    integral_x = 0.0;
    integral_y = 0.0;
    r.sleep();
  }
  return 0;
}
