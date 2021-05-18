#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <time.h>

//using namespace cv;
using namespace std;

int
main() {
  cv::VideoCapture cap(0);
  if(!cap.isOpened()) {
    printf("ERROR: khong the mo camera\r\n");
    return -1;
  }
  cv::namedWindow("Camera", cv::WINDOW_NORMAL);
  cv::resizeWindow("Camera", 300, 300);
  cv::namedWindow("GRAY", cv::WINDOW_NORMAL);
  cv::resizeWindow("GRAY", 300, 300);
  cv::namedWindow("Equalizing", cv::WINDOW_NORMAL);
  cv::resizeWindow("Equalizing", 300, 300);
  while(1) {
    cv::Mat frame;
    cap >> frame;
    if(frame.empty()) {
      printf("ERROR: khong the bat hinh\r\n");
      return -1;
    }
    /*--- Chuyen ve thang xam -----------------------*/
    cv::Mat frame_gray;
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    /*---- Equalizing Histogram ---------------------*/
    cv::Mat frame_result;
    cv::equalizeHist(frame_gray, frame_result);
    /*-----------------------------------------------*/
    cv::imshow("Camera", frame);
    cv::imshow("GRAY", frame_gray);
    cv::imshow("Equalizing", frame_result);
    cv::waitKey(1);
  }
  printf("Tat camera\r\n");
  cap.release();
  cv::destroyAllWindows();
  return 0;
}
