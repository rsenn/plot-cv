// Program to display a video from attached default camera device
// Author: Samarth Manoj Brahmbhatt, University of Pennsylvania

#include <opencv2/opencv.hpp>

//using namespace cv;
using namespace std;

int
main() {
  // Create a cv::VideoCapture object to cv::read from video file
  // 0 is the ID of the built-in laptop camera, change if you want to use other camera
  cv::VideoCapture cap(0);

  // check if the file was opened properly
  if(!cap.isOpened()) {
    cout << "Capture could not be opened succesfully" << endl;
    return -1;
  }

  cv::namedWindow("Video");

  // Play the video in a loop till it ends
  while(char(cv::waitKey(1)) != 'q' && cap.isOpened()) {
    cv::Mat frame;
    cap >> frame;
    // Check if the video is over
    if(frame.empty()) {
      cout << "Video over" << endl;
      break;
    }
    cv::imshow("Video", frame);
  }

  return 0;
}
