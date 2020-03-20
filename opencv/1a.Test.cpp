#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

int
main() {
  VideoCapture cap(0);
  if(!cap.isOpened()) {
    printf("ERROR: khong the mo camera \r\n");
    return 0;
  }

  Mat frame;
  while(1) {
    cap >> frame;
    if(frame.empty()) {
      printf("ERROR: khong the bat hinh\r\n");
      break;
    }
    // This function should be followed by waitKey function which displays the image for specified
    // milliseconds. Otherwise, it won’t display the image. For example, waitKey(0) will display the
    // window infinitely until any keypress (it is suitable for image display). waitKey(25) will
    // display a frame for 25 ms, after which display will be automatically closed.
    //(If you put it in a loop to read videos, it will display the video frame-by-frame)
    imshow("Camera", frame);
    cv::waitKey(5); // 5ms
  }
  printf("Tat camera\r\n");
  cap.release();
  destroyAllWindows();
  return 0;
}