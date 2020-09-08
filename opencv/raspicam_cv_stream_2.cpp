// source: http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html

//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/video/background_segm.hpp"

#include <iostream>
#include <cstring>
#include <string>

#include <time.h>
#include <limits.h>

#include "opencv2/opencv.hpp"
#include <opencv2/videoio.hpp>

#include <raspicam/raspicam_cv.h>

using namespace cv;
using namespace std;

void
printusage() {
  cout << "Usage:\n";
  cout << "  ./raspicam_cv_stream /var/www/output.mjpeg";
  cout << "\twill write stream file at given location\n";
  cout << endl;
}

raspicam::RaspiCam_Cv Camera;

int
init_raspicam(void) {
  cv::Mat image;
  int nCount = 10;
  // set camera params
  Camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
  // Open camera
  cout << "Opening Camera..." << endl;
  if(!Camera.open()) {
    cerr << "Error opening the camera" << endl;
    return -1;
  }
  // Start capture
  cout << "Capturing " << nCount << " frames ...." << endl;

  for(int i = 0; i < nCount; i++) {
    Camera.grab();
    Camera.retrieve(image);
    if(i % 2 == 0)
      cout << "\r captured " << i << " images"
           << " of size: " << image.size() << " img type: " << image.type() << std::flush;
  }
  cout << "Stop camera..." << endl;
  return 0;
}

int
main(int argc, char** argv) {

  init_raspicam();

  //  char* outFile = "./out.mjpg";
  //  char* outFile = "out.mp4"; // functional with ??
  //  char* outFile = "out.mjpg"; // worked with MJPG

  //  char* outFile = "/home/pi/DEV/bprp/raspicam_cv_stream/out.mjpg"; // functional with MJPG
  char* outFile = "/home/pi/out.mjpg";

  int resizeFactor = 1;
  bool writeOut = true;
  bool displayWindows = false;

  int frameSizeX = 320;
  int frameSizeY = 240;

  printusage();
  if(argc > 1)
    if(argv[1])
      outFile = argv[1];

  //   if (argv[2] && strcmp(argv[2], "-bgs") == 0)
  //    doBGS = true;

  int camIndex = 0;
  for(int i = 0; i < 5; i++) {
    VideoCapture tmpCap(i);
    if(tmpCap.isOpened()) {
      camIndex = i;
      tmpCap.release();
      cout << "MESSAGE: using /dev/video" << i << endl;
      break;
    }
    tmpCap.release();
  }

  camIndex = 0;
  /*
    VideoCapture cap(camIndex);

    if (!cap.isOpened()){
        cout<<"ERROR: /dev/video"<<camIndex<<" fails to open!\n";
        return -1;
    }


    cap.set(cv::CAP_PROP_FRAME_WIDTH, frameSizeX);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frameSizeY);
  */
  printf("Resolution set\n");

  int imgSizeX = resizeFactor * frameSizeX;
  int imgSizeY = resizeFactor * frameSizeY;
  bool isOutputColored = false;

  // fps counter begin
  time_t start, end;
  int counter = 0;
  double sec;
  double fps;
  // fps counter end

  Mat foreground;

  //  if (displayWindows) namedWindow("Source",1);
  //  if (displayWindows && doBGS) namedWindow("Background Subtraction", 1);

  //  unsigned int codec_id = CV_FOURCC('D','I','V','X');
  int codec_id = CV_FOURCC('D', 'I', 'V', 'X');
  codec_id = CV_FOURCC('M', 'J', 'P', 'G');

  //  codec_id = CV_FOURCC('F','L','V','1');
  //  codec_id = CV_FOURCC('U','2','6','3');

  // quit if ESC is pressed
  //    while((waitKey(10) & 255) != 27)
  VideoWriter outputVideo;
  Size S = Size((int)imgSizeX, (int)imgSizeY);

  /*
      union { int v; char c[5];} uEx ;
      char ex[] = "MJPG";
      uEx.v = (int) ex;
      uEx.c[4]='\0';
  */

  // const string NAME = source.substr(0, pAt) + argv[2][0] + ".avi";
  const string NAME = "a.avi";
  double out_fps = 30;

  Mat gray;

  while(1) {
    if(counter == 0) {
      time(&start);
    }
    Mat output;
    Mat frame;
    //        cap >> frame; // get a new frame from camera

    Camera.grab();
    Camera.retrieve(frame);

    if(frame.empty())
      printf("Retrieved empty frame!\n");
    else
      printf("Retrieved image! (%ix%i)\n", frame.cols, frame.rows);

    Mat frameResized;
    resize(frame, frameResized, Size(imgSizeX, imgSizeY));

    output = frameResized;

    /*
            if (output.type() != CV_8UC1)
            {

                cvtColor(output, gray, cv::COLOR_BGR2GRAY);

                output = gray;
            }
    */
    //        if (displayWindows) imshow("Output to sent", output);

    /*
                // original worked..
                VideoWriter outStream(outFile, codec_id, \
                            2, Size(imgSizeX, imgSizeY), isOutputColored);
                if (outStream.isOpened()){
                    outStream.write(output);
                } else {
                    cout<<"ERROR: Can't write to "<<outFile<<"!\n";
                    return -1;
                }
    */

    outputVideo.open(NAME, codec_id, out_fps, S, true);

    if(!outputVideo.isOpened()) {
      cout << "Could not open the output video for write: " << NAME << endl;
      return -1;
    }

    outputVideo << output;
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

Camera.release();
//    cap.release();
return 0;
}
