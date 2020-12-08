#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <time.h>

using namespace cv;
using namespace std;

int
main() {
  VideoCapture cap(0);
  if(!cap.isOpened()) {
    printf("ERROR: khong the mo camera\r\n");
    return -1;
  }
  namedWindow("Camera", WINDOW_NORMAL);
  resizeWindow("Camera", 300, 300);
  namedWindow("GRAY", WINDOW_NORMAL);
  resizeWindow("GRAY", 300, 300);
  namedWindow("Equalizing", WINDOW_NORMAL);
  resizeWindow("Equalizing", 300, 300);
  while(1) {
    Mat frame;
    cap >> frame;
    if(frame.empty()) {
      printf("ERROR: khong the bat hinh\r\n");
      return -1;
    }
    /*--- Chuyen ve thang xam -----------------------*/
    Mat frame_gray;
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    /*---- Equalizing Histogram ---------------------*/
    Mat frame_result;
    equalizeHist(frame_gray, frame_result);
    /*-----------------------------------------------*/
    imshow("Camera", frame);
    imshow("GRAY", frame_gray);
    imshow("Equalizing", frame_result);
    waitKey(1);
  }
  printf("Tat camera\r\n");
  cap.release();
  destroyAllWindows();
  return 0;
}
