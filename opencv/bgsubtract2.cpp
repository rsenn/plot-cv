// source: http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/videoio/legacy/constants_c.h>

#include <iostream>
#include <cstring>

#include <time.h>
#include <limits.h>

//using namespace cv;
using namespace std;

void
printusage() {
  cout << "Usage:\n";
  cout << "  ./bgsubtract2 /var/www/output.mjpeg			will cv::write at given location\n";
  cout << "  ./bgsubtract2 /var/www/output.mjpeg -bgs		showing background subtraction\n";
  cout << endl;
}

int
main(int argc, char** argv) {
  //	char* outFile = "./out.mjpg";
  //	char* outFile = "out.mp4"; // functional with ??
  char* outFile = "out.mjpg"; // functional with MJPG

  int resizeFactor = 1;
  bool doBGS = false;
  bool writeOut = true;
  bool displayWindows = true;

  int frameSizeX = 320;
  int frameSizeY = 240;

  if(argc < 2) {
    printusage();
    return -1;
  }

  if(argv[1])
    outFile = argv[1];

  if(argv[2] && strcmp(argv[2], "-bgs") == 0)
    doBGS = true;

  int camIndex = 0;
  for(int i = 0; i < 5; i++) {
    cv::VideoCapture tmpCap(i);
    if(tmpCap.isOpened()) {
      camIndex = i;
      tmpCap.release();
      cout << "MESSAGE: using /dev/video" << i << endl;
      break;
    }
    tmpCap.release();
  }

  camIndex = 0;
  cv::VideoCapture cap(camIndex);

  if(!cap.isOpened()) {
    cout << "ERROR: /dev/video" << camIndex << " fails to open!\n";
    return -1;
  }

  cap.set(cv::CAP_PROP_FRAME_WIDTH, frameSizeX);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, frameSizeY);

  printf("Resolution set\n");

  int imgSizeX = resizeFactor * frameSizeX;
  int imgSizeY = resizeFactor * frameSizeY;
  bool isOutputColored = !doBGS;

  // fps counter begin
  time_t start, end;
  int counter = 0;
  double sec;
  double fps;
  // fps counter end

  cv::Mat foreground;

  if(displayWindows)
    cv::namedWindow("Source", 1);
  //	if (displayWindows && doBGS) cv::namedWindow("Background Subtraction", 1);

  unsigned int codec_id = CV_FOURCC('D', 'I', 'V', 'X');
  codec_id = CV_FOURCC('M', 'J', 'P', 'G');

  //	codec_id = CV_FOURCC('F','L','V','1');
  //	codec_id = CV_FOURCC('U','2','6','3');

  // quit if ESC is pressed
  while((cv::waitKey(10) & 255) != 27) {
    if(counter == 0) {
      time(&start);
    }
    cv::Mat output;
    cv::Mat frame;
    cap >> frame; // get a new frame from camera

    cv::Mat frameResized;
    cv::resize(frame, frameResized, cv::Size(imgSizeX, imgSizeY));

    output = frameResized;
    cv::Mat gray;
    cv::cvtColor(output, gray, cv::COLOR_BGR2GRAY);

    output = gray;

    if(displayWindows)
      cv::imshow("Output to sent", output);

    isOutputColored = false;
    if(writeOut) {

      // DON'T mind the following warning
      // OpenCV: FFMPEG: tag 0x47504a4d/'MJPG' is not supported with codec id 8 and cv::format 'mjpeg /
      // raw MJPEG video' it should work anyways = after running mjpg_streamer and show it on
      // another pc via javascript

      cv::VideoWriter outStream(outFile, codec_id, 2, cv::Size(imgSizeX, imgSizeY), isOutputColored);
      if(outStream.isOpened()) {
        outStream.cv::write(output);
      } else {
        cout << "ERROR: Can't cv::write to " << outFile << "!\n";
        return -1;
      }
    }
    // fps counter begin
    time(&end);
    counter++;
    sec = difftime(end, start);
    fps = counter / sec;
    cout.precision(2);
    cout << fixed << fps << " fps\n";
    if(counter == (INT_MAX - 1000))
      counter = 0;
    // fps counter end
  }
  cap.release();
  return 0;
}
