#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/types_c.h>

//using namespace cv;
using namespace std;
#define pi 3.14159265358979323846 // 20170420
short int old_area = 0;
short int now_area = 0;
short int resis_area;
short int old_i;
short int now_i;
short int resis_i;
short int old_2i;
short int now_2i;
double center_x;
double center_y;
bool x;
int
main(int argc, char** argv) {
  cv::VideoCapture cap(0);
  cv::Mat src, src_copy;
  std::vector<std::vector<cv::Point>> contours, contours_2;
  std::vector<cv::Vec4i> hierarchy, hierarchy_2;
  cv::Point2f center, center_2; // onject_center, red_center;
  float radius;
  float radius_2;

  cv::Mat hsv_element_dilate = cv::getStructuringElement(MORPH_DILATE, cv::Size(15, 15));  // 20170420
  cv::Mat hsv_element_erode = cv::getStructuringElement(MORPH_ERODE, cv::Size(3, 3));      // 20170420
  cv::Mat center_element_dilate = cv::getStructuringElement(MORPH_RECT, cv::Size(10, 10)); // 20170420
  cv::Mat center_element_erode = cv::getStructuringElement(MORPH_RECT, cv::Size(7, 7));    // 20170420
  cv::Point2f vec_center_minus_red, vec_horizontal;                                // 20170420
  double arc_theta;
  double theta;

  // ros::init(argc, argv, "middlecv::Point_node");
  // ros::NodeHandle nh; //20170414
  // geometry_msgs::cv::Point msg_xy_angle; //20170414
  // ros::Publisher pub_xy_angle = nh.advertise<geometry_msgs::cv::Point>("xy_angle", 1000);
  // //20170409 std_msgs::Int64 msg_area; //20170414 ros::Publisher pub_area =
  // nh.advertise<std_msgs::Int64>("area", 1000); //20170409

  // std_msgs::Bool msg_if_image;
  // ros::Publisher pub_if_image = nh.advertise<std_msgs::Bool>("if_image", 1000);
  while(1) {
    if((contours.empty()) || (contours_2.empty())) {
      x = 0;
      // msg_if_image.data = x;
    } else if((!contours.empty()) && (!contours_2.empty())) {
      x = 1;
      // msg_if_image.data = x;
    }
    cv::Mat src_HSV;       // 20170420
    cv::Mat hsv_threshold; // 20170420
    cv::Mat src_add_mask;  // 20170420
    cap >> src;
    src.copyTo(src_copy);
    // src.copyTo(src_HSV); //20170420  *************src_HSV ���ӷ�(�O�����m)�Msrc�@��
    src_HSV = src.clone(); ////20170420*************** ����ƻs�@�Msrc�X�� ,output���ӷ�(�O�����m)�M���P
    cv::cvtColor(src_copy, src_copy, cv::COLOR_BGR2GRAY);
    cv::cvtColor(src_HSV, src_HSV, cv::COLOR_BGR2HSV); // 20170420
    // cv::threshold(src_copy, src_copy, 70, 255, THRESH_BINARY_INV); //20170420
    // cv::threshold(src_copy, src_copy, 100, 255, THRESH_OTSU);      //20170420
    cv::inRange(src_copy, 0, 89, src_copy); // 20170420
    // cv::erode(src_copy, src_copy, cv::Mat());//20170420
    // cv::dilate(src_copy, src_copy, cv::Mat());//20170420
    /////cv::imshow("src_copy", src_copy);//binanry image //20170420
    cv::inRange(src_HSV, cv::Scalar(153, 121, 91), cv::Scalar(192, 255, 255), hsv_threshold); // 20170420
    cv::dilate(hsv_threshold, hsv_threshold, hsv_element_dilate);                     // 20170420
    cv::erode(hsv_threshold, hsv_threshold, hsv_element_erode);                       // 20170420
    cv::add(src_HSV, src_HSV, src_add_mask, hsv_threshold);                           // 20170420
    int num = 0;
    if(!src_HSV.empty()) { // 20170420
      cv::findContours(hsv_threshold, contours_2, hierarchy_2, cv::RETR_TREE, CHAIN_APPROX_NONE, cv::Point(0, 0));
      if(!contours_2.empty()) {
        //#pragma omp parallel for
        for(int i = 0; i < contours_2.size(); i++) {
          cv::minEnclosingCircle(contours_2[i], center_2, radius_2);
          cv::line(src, center_2, center_2, cv::Scalar(120, 0, 255), 5);
        }
      }
    }
    if(!src_copy.empty()) {
      cv::findContours(src_copy, contours, hierarchy, cv::RETR_TREE, CHAIN_APPROX_NONE, cv::Point(0, 0));
      if(!contours.empty()) {
        //#pragma omp parallel for
        for(int i = 0; i < contours.size(); i++) {
          if(hierarchy[i][0] == -1) {
            if(hierarchy[i][1] == -1) {
              if(hierarchy[i][2] != -1) {
                if(hierarchy[i][3] != -1) {
                  if(!contours.empty()) {
                    cv::line(src, cv::Point(300, 240), cv::Point(340, 240), cv::Scalar(0, 255, 0), 3);
                    cv::line(src, cv::Point(320, 220), cv::Point(320, 260), cv::Scalar(0, 255, 0), 3);

                    now_i = i;
                    if(old_i > now_i) {
                      old_i = now_i;
                    } else if(old_i < now_i) {
                      old_i = now_i;
                    }

                    now_area = cv::contourArea(contours[i]);
                    if(old_area > now_area) {
                      old_area = now_area;
                    } else if(old_area < now_area) {
                      old_area = now_area;
                    }
                  }
                }
              }
            }
          }
        }
        resis_i = old_i;
        if(old_2i > resis_i) {
          old_2i = resis_i;
          cv::minEnclosingCircle(contours[old_2i], center, radius);
          cv::line(src, center, center, cv::Scalar(0, 0, 255), 5);
        } else if(old_2i < resis_i) {
          old_2i = resis_i;
          cv::minEnclosingCircle(contours[old_2i], center, radius);
          cv::line(src, center, center, cv::Scalar(0, 0, 255), 5);
        }
        vec_center_minus_red = center_2 - center;                   // 20170420
        vec_horizontal = cv::Point(300, 240) - cv::Point(200, 240); // 20170420
        ///////////////////�|(��)��!!!!!!!!!!! 20170420
        arc_theta = acos(vec_center_minus_red.dot(vec_horizontal) / (cv::pow(vec_center_minus_red.dot(vec_center_minus_red), 0.5) *
                                                                     cv::pow(vec_horizontal.dot(vec_horizontal), 0.5)));
        theta = (arc_theta * 360) / (2 * pi); // 20170420

        if((center_2.x > center.x) && (center_2.y < center.y)) { // 20170420
          // cout << "arc_theta:  " << theta << endl;
        } else if((center_2.x < center.x) && (center_2.y < center.y)) { // 20170420
          // cout << "arc_theta:  " << theta << endl;
        } else if((center_2.x < center.x) && (center_2.y > center.y)) { // 20170420
          theta = 360 - theta;
          // cout << "arc_theta:  " << theta << endl;
        } else if((center_2.x > center.x) && (center_2.y > center.y)) { // 20170420
          theta = 360 - theta;
          // cout << "arc_theta:  " << theta << endl;
        }
        // ReSharper disable once CppExpressionStatementsWithoudSideEffects
        if(theta > 180)
          theta -= 360;
        cv::line(src, center, center_2, cv::Scalar(250, 50, 120), 5); // 20170420
        /*msg_xy_angle.x = center.x;
        msg_xy_angle.y = center.y;
        msg_xy_angle.z = theta;*/
        // ReSharper disable once CppExpressionStatementsWithoudSideEffects
        if((resis_area - old_area > 200) && (num == 0) && (resis_area != 0)) {
          num = 1;
        } else if((old_area - resis_area > 200) && (num == 0) && (resis_area != 0)) {
          // cout << "Too low" << "  old_area  " << old_area << "�@resis_area�@" << resis_area << "
          // delta  " << old_area
          // - resis_area << endl; //20170414  cv::putText(src, "Too low", cv::Point(20, 50), 2, 1,
          // cv::Scalar(0, 0, 255)); ReSharper disable once CppExpressionStatementsWithoudSideEffects
          num = 1;
        } else {
        }
        resis_area = old_area;
        // msg_area.data = old_area;
      }
    }

    // pub_xy_angle.publish(msg_xy_angle); //20170420
    // pub_area.publish(msg_area); //20170420
    // pub_if_image.publish(msg_if_image);
    // ROS_INFO_STREAM("cv::Point (" << msg_xy_angle.x << ", " << msg_xy_angle.y << ")\tangle " <<
    // msg_xy_angle.z << "\t Area " << msg_area.data); //20170420
    cv::imshow("src", src);
    ////cv::imshow("inRange_HSV", hsv_threshold);   //20170420
    ////cv::imshow("cv::add:  ", src_add_mask);//20170420
    if(cv::waitKey(16) == 27)
      break;
  }
  return 0;
}
