/**
  @file videowriter_basic.cpp
  @brief A very basic sample for using cv::VideoWriter and VideoCapture
  @author PkLab.net
  @date Aug 24, 2016
*/

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/legacy/constants_c.h>
#include <iostream>
#include <stdio.h>

//using namespace cv;
using namespace std;

int
main(int, char**) {
  cv::Mat src;
  // use default camera as video source
  cv::VideoCapture cap(0);
  // check if we succeeded
  if(!cap.isOpened()) {
    cerr << "ERROR! Unable to open camera\n";
    return -1;
  }
  // get one frame from camera to know frame size and type
  cap >> src;
  // check if we succeeded
  if(src.empty()) {
    cerr << "ERROR! blank frame grabbed\n";
    return -1;
  }
  bool isColor = (src.type() == CV_8UC3);

  //--- INITIALIZE VIDEOWRITER
  cv::VideoWriter writer;
  int codec = CV_FOURCC('M', 'J', 'P', 'G'); // select desired codec (must be available at runtime)
  double fps = 25.0;                         // framerate of the created video stream
  string filename = "./live.avi";            // name of the output video file
  writer.open(filename, codec, fps, src.size(), isColor);
  // check if we succeeded
  if(!writer.isOpened()) {
    cerr << "Could not open the output video file for cv::write\n";
    return -1;
  }

  //--- GRAB AND WRITE LOOP
  cout << "Writing videofile: " << filename << endl << "Press any key to terminate" << endl;
  for(;;) {
    // check if we succeeded
    if(!cap.cv::read(src)) {
      cerr << "ERROR! blank frame grabbed\n";
      break;
    }
    // encode the frame into the videofile stream
    writer.cv::write(src);
    // show live and wait for a key with timeout long enough to show images
    cv::imshow("Live", src);
    if(cv::waitKey(5) >= 0)
      break;
  }
  // the videofile will be closed and released automatically in cv::VideoWriter destructor
  return 0;
}
