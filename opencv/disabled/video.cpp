//節點名稱"circle_tracking_node"
//發布訊息"center_position"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
//using namespace cv;
using namespace std;

ros::Publisher pub;
geometry_msgs::cv::Point msg;
std::deque<cv::Point2f> trac(32);

int
main(int argc, char** argv) {

  cv::VideoCapture capture(0);
  capture.set(cv::CAP_PROP_FRAME_WIDTH, 320);
  capture.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
  ros::init(argc, argv, "circle_node");
  ros::NodeHandle nh;

  pub = nh.advertise<geometry_msgs::cv::Point>("center_position", 1000);
  while(/*(char(cv::waitKey(1)) != 27) ||*/ (ros::ok())) {

    cv::Mat frame;
    capture >> frame;
    cv::Mat midImage;
    // cv::resize(frame, frame, cv::Size(240, 320), 0, 0, cv::INTER_AREA);
    // cv::cvtColor(frame, midImage, COLOR_BGR2HSV);
    // cv::flip(frame,frame,1);
    cv::cvtColor(frame, midImage, COLOR_BGR2GRAY);
    cv::GaussianBlur(midImage, midImage, cv::Size(9, 9), 2, 2);
    cv::threshold(midImage, midImage, 50, 255, THRESH_BINARY_INV);
    // cv::inRange(midImage, cv::Scalar(0, 0, 0, 0), cv::Scalar(360, 255, 30, 0), midImage);
    cv::Mat element = cv::getStructuringElement(MORPH_RECT, cv::Size(10, 10)); // 15,15
    cv::dilate(midImage, midImage, element);
    cv::erode(midImage, midImage, element);
    cv::imshow("cv::erode", midImage);
    cv::Mat not_img;
    cv::bitwise_not(midImage, not_img);
    cv::imshow("Not Image", not_img);
    cv::Mat edge;
    cv::Canny(not_img, edge, 20, 160, 3); // trackbar_test

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    RNG rng(12345);
    cv::findContours(edge, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point2f> center(contours.size());
    std::vector<float> radius(contours.size());

    for(int i = 0; i < contours.size(); i++) {
      cv::minEnclosingCircle(contours[i], center[i], radius[i]);
      if(radius[i] > 40) {
        cv::circle(frame, center[i], radius[i], cv::Scalar(255, 0, 0), 3, cv::LINE_AA, 0);
        cv::circle(frame, center[i], 2, cv::Scalar(0, 0, 255), 3, cv::LINE_AA, 0);
        msg.x = (int)center[i].x;
        msg.y = (int)center[i].y;
        msg.z = 1;
        cout << "x= " << center[i].x << ", y= " << center[i].y << endl;
        ROS_INFO_STREAM("Found a ball at (" << msg.x << ", " << msg.y << ")");

      } // if
      center[0].x = 0;
      center[0].y = 0;
      msg.z = 0;
    } // for
    std::deque<cv::Point2f>::iterator it = trac.begin();
    int cnt = 0;
    while(it != trac.end()) {
      int thickness = 4 - cnt++ / 8;
      cv::Point2f p = *it;
      cv::Point2f p2 = *(it + 1);
      if(p.x == 0 || p2.x == 0) {
        it++;
        continue;
      } else if(thickness > 0)
        cv::line(frame, *it, *(it + 1), cv::Scalar(0, 255, 0), thickness);
      else {
        trac.pop_back();
        continue;
      }
      it++;
    }
    if(trac.size() >= 32)
      trac.pop_back();
    ROS_DEBUG_STREAM(trac.size());
    if(msg.z == 0)
      ROS_ERROR_STREAM("Where's the ball???");
    pub.publish(msg);
    cv::line(frame, cv::Point(320, 220), cv::Point(320, 260), cv::Scalar(0, 0, 0), 2, 8);
    cv::line(frame, cv::Point(300, 240), cv::Point(340, 240), cv::Scalar(0, 0, 0), 2, 8);
    cv::imshow("From Webcam", frame);
    cv::waitKey(1);
    // cv::imshow("cv::erode",midImage);
    // cv::imshow("contours", frame);

  } // while
  return 0;
}
