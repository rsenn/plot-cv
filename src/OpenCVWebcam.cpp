#include "opencv2/opencv.hpp"

using namespace cv;

int
main(int argc, char** argv) {
  // Create new video capture.
  VideoCapture cap;

  // If video capture not opened return.
  if(!cap.open(0)) {
    return 0;
  }

  while(1) {
    // Create new frame.
    Mat frame;

    // Capture data into frame.
    cap >> frame;

    // If frame empty end video stream.
    if(frame.empty()) {
      break;
    }

    // Display captured image.
    imshow("this is you, smile! :)", frame);

    // Stop capture by pressing key 10 ( ESC )
    if(waitKey(10) == 27)
      break;
  }

  // the camera will be closed automatically upon exit
  // cap.close();
  return 0;
}