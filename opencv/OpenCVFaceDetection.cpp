//////////////////////////////////////////
// INCLUDES
//////////////////////////////////////////

#include <opencv2/opencv.hpp>

//////////////////////////////////////////
// DEFINES
//////////////////////////////////////////

#define VIDEO_CAPTURE_DEVICE (0)

//////////////////////////////////////////
// FUNCTIONS
//////////////////////////////////////////

int
main(int argc, char** argv) {
  // Create videoCapture
  cv::VideoCapture videoCapture;

  // If open video capture device fails, return.
  if(!videoCapture.open(VIDEO_CAPTURE_DEVICE)) {
    return 0;
  }

  // Create frame to save video capture in.
  cv::Mat videoFrame;

  // Main loop.
  while(1) {
    // Put video capture in frame.
    videoCapture >> videoFrame;

    // If video capture is empty, break.
    if(videoFrame.empty()) {
      break;
    }

    // Show video frame.
    cv::imshow("Video frame.", videoFrame);

    // Stop capture by pressing key 27 ( ESC )
    if(cv::waitKey(10) == 27)
      break;
  }

  // Return 0.
  return 0;
}
