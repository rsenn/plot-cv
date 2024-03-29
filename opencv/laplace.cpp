#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/types_c.h>

#include <ctype.h>
#include <stdio.h>
#include <iostream>

// using namespace cv;
using namespace std;

static void
help() {
  cout << "\nThis program demonstrates Laplace point/edge detection using OpenCV function "
          "cv::Laplacian()\n"
          "It captures from the camera of your choice: 0, 1, ... default 0\n"
          "Call:\n"
          "./laplace [camera #, default 0]\n"
       << endl;
}

int sigma = 3;
int smoothType = CV_GAUSSIAN;

int
main(int argc, char** argv) {
  cv::VideoCapture cap;
  help();

  if(argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
    cap.open(argc == 2 ? argv[1][0] - '0' : 0);
  else if(argc >= 2) {
    cap.open(argv[1]);
    if(cap.isOpened())
      cout << "Video " << argv[1] << ": width=" << cap.get(cv::CAP_PROP_FRAME_WIDTH)
           << ", height=" << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << ", nframes=" << cap.get(cv::CAP_PROP_FRAME_COUNT) << endl;
    if(argc > 2 && isdigit(argv[2][0])) {
      int pos;
      sscanf(argv[2], "%d", &pos);
      cout << "seeking to frame #" << pos << endl;
      cap.set(cv::CAP_PROP_POS_FRAMES, pos);
    }
  }

  if(!cap.isOpened()) {
    cout << "Could not initialize capturing...\n";
    return -1;
  }

  cv::namedWindow("cv::Laplacian", 0);
  cv::createTrackbar("Sigma", "cv::Laplacian", &sigma, 15, 0);

  cv::Mat smoothed, laplace, result;

  for(;;) {
    cv::Mat frame;
    cap >> frame;
    if(frame.empty())
      break;

    int ksize = (sigma * 5) | 1;
    if(smoothType == CV_GAUSSIAN)
      cv::GaussianBlur(frame, smoothed, cv::Size(ksize, ksize), sigma, sigma);
    else if(smoothType == CV_BLUR)
      cv::blur(frame, smoothed, cv::Size(ksize, ksize));
    else
      cv::medianBlur(frame, smoothed, ksize);

    cv::Laplacian(smoothed, laplace, CV_16S, 5);
    cv::convertScaleAbs(laplace, result, (sigma + 1) * 0.25);
    cv::imshow("cv::Laplacian", result);

    int c = cv::waitKey(30);
    if(c == ' ')
      smoothType = smoothType == CV_GAUSSIAN ? CV_BLUR : smoothType == CV_BLUR ? CV_MEDIAN : CV_GAUSSIAN;
    if(c == 'q' || c == 'Q' || (c & 255) == 27)
      break;
  }

  return 0;
}
