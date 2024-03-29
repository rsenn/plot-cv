#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

// using namespace cv;
using namespace std;

int
main() {
  cv::VideoCapture cap(0);
  if(!cap.isOpened()) {
    printf("ERROR: khong the mo camera\r\n");
    return -1;
  }
  cv::Mat frame;
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
