//#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>

#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
using namespace std;
//using namespace cv;
#define pi 3.14159265358979323846

int
main(int argc, char** argv) {
  cv::VideoCapture cap(2);
  // cv::VideoWriter writer;
  // writer.open("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 15,
  // cv::Size((int)cap.get(cv::CAP_PROP_FRAME_WIDTH), (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT)));
  cv::Mat src, src_gray, hsv, threshold_blue;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  short int old_i = 0;
  cv::Point2f center;
  float radius;
  double area;
  bool ifExist = false;

  // ros::init(argc, argv, "middlecv::Point_node");
  // ros::NodeHandle nh;
  // geometry_msgs::Twist msg_xy;
  // ros::Publisher pub_msg_xy = nh.advertise<geometry_msgs::Twist>("mid_xy", 1000);

  ////std_msgs::Bool msg_if_image;

  while(1) {
    cap >> src;
    hsv = src.clone();
    cv::cvtColor(hsv, hsv, cv::COLOR_BGR2HSV);

    if(!hsv.empty()) {
      // cv::Canny(threshold_blue, threshold_blue, 90, 60, 3);
      // cv::imshow("canny_blue", threshold_blue);
      cv::inRange(hsv, cv::Scalar(72, 119, 75), cv::Scalar(138, 255, 255), threshold_blue);
      cv::imshow("threshold_blue", threshold_blue);

      if(!threshold_blue.empty()) {
        cv::findContours(threshold_blue, contours, hierarchy, cv::RETR_TREE, CHAIN_APPROX_NONE, cv::Point(0, 0));
        if(!contours.empty()) {
          for(int i = 0; i < contours.size(); i++) {
            if(hierarchy[i][0] == -1) {
              if(hierarchy[i][1] == -1) {
                if(hierarchy[i][2] != -1) {
                  if(hierarchy[i][3] != -1) {
                    cv::line(src, cv::Point(300, 240), cv::Point(340, 240), cv::Scalar(0, 255, 0), 3);
                    cv::line(src, cv::Point(320, 220), cv::Point(320, 260), cv::Scalar(0, 255, 0), 3);
                    old_i = i;
                    ifExist = true;
                  }
                }
              }
            }
            // std::vector<cv::Moments> mu(contours.size());
            // mu[i] = cv::moments((contours[i], true));
            // double area = mu[i].m00;
            // cout << "area:  " << area << endl;
          }

          if(ifExist) {
            cv::minEnclosingCircle(contours[old_i], center, radius);
            // area = cv::contourArea(contours[old_i]);
            // cout << "area:  " << area<< endl;
            // cout << "radius:  " << radius << endl;
            if(!contours[old_i].empty()) {
              if(radius <= 11) {
                // cout << "1  " << endl;
                // msg_xy.linear.z
              } else if(radius <= 16) {
                // cout << "1/0.8" << endl;
                // msg_xy.linear.z
              } else if(radius <= 21) {
                // cout << "1/0.5" << endl;
                // msg_xy.linear.z
              } else {
                // cout <<"1/0.1"<<endl;
                // msg_xy.linear.z
              }
            }
            // 300  31 cm         10 cm
            //   1
            // 600 31-6cm 25     14.6
            //   1/0.8
            //  1200  25- 6cm   19     20.4
            //   1/0.5
            //  2800  19 -6     13     31.7
            //   1/0.1
            // cout << "center:  " << center << endl;
            // contours[0][0].x = -contours[0][0].x;
            // contours[0][0].y = -contours[0][0].y;
            // msg_xy.linear.x = center.x; msg_xy.linear.y = center.y;
            // msg_xy.angular.x = double(320.0); msg_xy.angular.y = double(240.0);

          } else {
            // cout <<contours[0][0].x << contours[0][0].y << endl;
            // center = -center;
            // msg_xy.angular.x = contours[0][0].x; msg_xy.angular.y = contours[0][0].y;
            // msg_xy.linear.x = double(320.0); msg_xy.linear.y = double(240.0);
            cv::line(src,
                 cv::Point(contours[0][0].x, contours[0][0].y),
                 cv::Point(contours[0][0].x, contours[0][0].y),
                 cv::Scalar(0, 255, 0),
                 5);
          }
          cv::line(src, center, center, cv::Scalar(0, 0, 255), 10);
          // contours[0][0].x = 0; contours[0][0].y = 0;

          ifExist = false;
        }
      }
    }
    // msg_xy.linear.z = 1;
    // pub_msg_xy.publish(msg_xy);
    cv::imshow("src", src);
    // cv::imshow("threshold_contour", cv::threshold);
    // writer.cv::write(src);
    if(cv::waitKey(16) == 27)
      break;
  }
  return 0;
}
