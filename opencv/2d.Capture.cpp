#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

int
main() {
  VideoCapture cap(0);
  if(!cap.isOpened()) {
    printf("ERROR: khong the mo camera 0\r\n");
    return 0;
  }
  int count = 0;
  Mat frame;
  while(1) {
    cap >> frame;       // capture frame hien tai tu camera
    if(frame.empty()) { // kiem tra frame co du lieu hay ko
      printf("ERROR: khong the bat hinh\r\n");
      break;
    }
    char path[32];
    snprintf(path, 32, "/tmp/capture_%d.jpg", count);
    imwrite(path, frame);
    count++;
    imshow("Camera", frame);
    char c = waitKey(5); // 5ms
    if(c == 'c')
      break;
  }
  printf("Tat camera\r\n");
  cap.release();
  destroyAllWindows();
  return 0;
}
