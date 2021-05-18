// Program illustrate distance measurement using a stereo camera
// Author: Samarth Manoj Brahmbhatt, University of Pennsylvania

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "Config.h"

//using namespace cv;
using namespace std;

class disparity {
private:
  cv::Mat map_l1, map_l2, map_r1, map_r2, Q;
  StereoSGBM stereo;
  int min_disp, num_disp;

public:
  disparity(string, cv::Size);
  void
  set_minDisp(int minDisp) {
    stereo.minDisparity = minDisp;
  }
  void
  set_numDisp(int numDisp) {
    stereo.numberOfDisparities = numDisp;
  }
  void show_disparity(cv::Size);
};

void
on_minDisp(int min_disp, void* _disp_obj) {
  disparity* disp_obj = (disparity*)_disp_obj;
  disp_obj->set_minDisp(min_disp - 30);
}

void
on_numDisp(int num_disp, void* _disp_obj) {
  disparity* disp_obj = (disparity*)_disp_obj;
  num_disp = (num_disp / 16) * 16;
  cv::setTrackbarPos("numDisparity", "Disparity", num_disp);
  disp_obj->set_numDisp(num_disp);
}

disparity::disparity(string filename, cv::Size image_size) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  fs["map_l1"] >> map_l1;
  fs["map_l2"] >> map_l2;
  fs["map_r1"] >> map_r1;
  fs["map_r2"] >> map_r2;
  fs["Q"] >> Q;

  if(map_l1.empty() || map_l2.empty() || map_r1.empty() || map_r2.empty() || Q.empty())
    cout << "WARNING: Loading of mapping matrices not successful" << endl;

  stereo.preFilterCap = 63;
  stereo.SADWindowSize = 3;
  stereo.P1 = 8 * 3 * stereo.SADWindowSize * stereo.SADWindowSize;
  stereo.P2 = 32 * 3 * stereo.SADWindowSize * stereo.SADWindowSize;
  stereo.uniquenessRatio = 10;
  stereo.speckleWindowSize = 100;
  stereo.speckleRange = 32;
  stereo.disp12MaxDiff = 1;
  stereo.fullDP = true;
}

void
disparity::show_disparity(cv::Size image_size) {
  cv::VideoCapture capr(1), capl(2);
  // reduce frame size
  capl.set(cv::CAP_PROP_FRAME_HEIGHT, image_size.height);
  capl.set(cv::CAP_PROP_FRAME_WIDTH, image_size.width);
  capr.set(cv::CAP_PROP_FRAME_HEIGHT, image_size.height);
  capr.set(cv::CAP_PROP_FRAME_WIDTH, image_size.width);

  min_disp = 30;
  num_disp = ((image_size.width / 8) + 15) & -16;

  cv::namedWindow("Disparity", cv::WINDOW_NORMAL);
  cv::namedWindow("Left", cv::WINDOW_NORMAL);
  cv::createTrackbar("minDisparity + 30", "Disparity", &min_disp, 60, on_minDisp, (void*)this);
  cv::createTrackbar("numDisparity", "Disparity", &num_disp, 150, on_numDisp, (void*)this);

  on_minDisp(min_disp, this);
  on_numDisp(num_disp, this);

  while(char(cv::waitKey(1)) != 'q') {
    // grab raw frames first
    capl.grab();
    capr.grab();
    // decode later so the grabbed frames are less apart in time
    cv::Mat framel, framel_rect, framer, framer_rect;
    capl.retrieve(framel);
    capr.retrieve(framer);

    if(framel.empty() || framer.empty())
      break;

    cv::remap(framel, framel_rect, map_l1, map_l2, INTER_LINEAR);
    cv::remap(framer, framer_rect, map_r1, map_r2, INTER_LINEAR);

    cv::Mat disp, disp_show, disp_compute, pointcloud;
    stereo(framel_rect, framer_rect, disp);
    disp.convertTo(disp_show, CV_8U, 255 / (stereo.numberOfDisparities * 16.));
    disp.convertTo(disp_compute, CV_32F, 1.f / 16.f);

    // Calculate 3D co-ordinates from disparity image
    reprojectImageTo3D(disp_compute, pointcloud, Q, true);

    // Draw red cv::rectangle around 40 px wide square area im image
    int xmin = framel.cols / 2 - 20, xmax = framel.cols / 2 + 20, ymin = framel.rows / 2 - 20, ymax = framel.rows / 2 + 20;
    cv::rectangle(framel_rect, cv::Point(xmin, ymin), cv::Point(xmax, ymax), cv::Scalar(0, 0, 255));

    // Extract depth of 40 px cv::rectangle and print out their mean
    pointcloud = pointcloud(Range(ymin, ymax), Range(xmin, xmax));
    cv::Mat z_roi(pointcloud.size(), CV_32FC1);
    int from_to[] = {2, 0};
    cv::mixChannels(&pointcloud, 1, &z_roi, 1, from_to, 1);

    cout << "Depth: " << cv::mean(z_roi) << " mm" << endl;

    cv::imshow("Disparity", disp_show);
    cv::imshow("Left", framel_rect);
  }
  capl.release();
  capr.release();
}

int
main() {
  string filename = DATA_FOLDER + string("stereo_calib.xml");

  cv::Size image_size(320, 240);
  disparity disp(filename, image_size);
  disp.show_disparity(image_size);

  return 0;
}
