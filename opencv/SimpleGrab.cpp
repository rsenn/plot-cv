#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

//using namespace cv;
using namespace std;

int
main(int argc, char** argv) {
  cv::VideoCapture cap("James.mp4");
  if(!cap.isOpened()) {
    cerr << "ERROR: Unable to open the camera" << endl;
    return 0;
  }

  cv::Mat frame;
  cout << "Start grabbing, press a key on Live window to terminate" << endl;
  while(1) {
    cap >> frame;
    if(frame.empty()) {
      cerr << "ERROR: Unable to grab from the camera" << endl;
      break;
    }
    cv::imshow("Live", frame);
    int key = cv::waitKey(20);
    key = (key == 255) ? -1 : key;
    if(key >= 0)
      break;
  }

  cout << "Closing the camera" << endl;
  cap.release();
  cv::destroyAllWindows();
  cout << "bye!" << endl;
  return 0;
}
