#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

//using namespace cv;
using namespace std;

int
main() {
  cv::VideoCapture cap(0);
  if(!cap.isOpened()) {
    printf("ERROR: khong the mo camera\r\n");
    return -1;
  }
  cv::Mat frame;
  /*--- Bat mot so khung hinh dau tien de doi camera on dinh --*/
  int count = 0;
  while(count < 10) {
    cap >> frame;
    if(frame.empty()) {
      printf("ERROR: khong the bat hinh\r\n");
      break;
    }
    cv::imshow("Camera", frame);
    /*-----------------------------------------------*/
    cv::waitKey(1);
    count++;
  }
  /*--- Chon khung hinh tu camera ---------------------*/
  // Select ROI
  bool showCrosshair = false;
  bool fromCenter = false;
  Rect2d r = cv::selectROI("Camera", frame, fromCenter, showCrosshair);
  // Crop image
  cv::Mat imageCrop = frame(r);
  cv::namedWindow("Crop", WINDOW_NORMAL);
  cv::resizeWindow("Crop", 300, 300);
  cv::imshow("Crop", imageCrop); // Hien thi ma tran du lieu hinh anh
  /*------- Doc lien tuc ------------------------------*/
  while(1) {
    cap >> frame;
    if(frame.empty()) {
      printf("ERROR: khong the bat hinh\r\n");
      break;
    }
    cv::imshow("Camera", frame);
    /*-----------------------------------------------*/
    cv::waitKey(1);
  }
  printf("Tat camera\r\n");
  cap.release();
  cv::destroyAllWindows();
  return 0;
}
