#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

#include <Tensile.h>

//using namespace cv;
using namespace std;

cv::Mat
run(Tensile* obj, int count) {

  std::vector<cv::Point2f> mc;
  std::vector<std::vector<cv::Point>> contours;
  cv::Mat original = obj->getCurrentFrame();

  obj->getCurrentFrame() = obj->imageToBinary(obj->getCurrentFrame());

  contours = obj->getContours(obj->getCurrentFrame()); // get the contours

  mc = obj->getMassCenters(contours); // get the mass centers

  cout << endl << "Frame: " << count << endl;
  obj->printMassCenters(mc, contours); // print mass centers

  // cv::waitKey(0);

  for(int i = 0; i < contours.size(); i++) cv::circle(original, mc[i], 1, CV_RGB(255, 0, 0), 3, 8, 0);

  return original;
}

int
main() {

  cv::VideoCapture capture("C:/Users/Jameson/Desktop/data/specvid.avi");
  Tensile* object = new Tensile(capture.get(cv::CAP_PROP_FRAME_COUNT));

  cv::Mat currentFrame;

  cv::namedWindow("Contour", cv::WINDOW_AUTOSIZE);

  for(int i = 0; i < object->getFrameNumber() - 1; i++) {
    capture >> currentFrame;
    object->setCurrentFrame(currentFrame);

    cv::imshow("Contour", run(object, i));
    cv::waitKey(10);
  }

  cout << endl << endl << "total amount of frames: " << object->getFrameNumber() << endl << endl;

  return 0;
}
