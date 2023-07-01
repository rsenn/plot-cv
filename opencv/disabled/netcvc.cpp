// This sends the data to the other computer.
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
// for mac
#include <unistd.h>

#include <iostream>
#include <pthread.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "videoSender.h"
#include "videoSender.cpp"

using namespace std;
// using namespace cv;

cv::VideoCapture capture;
cv::Mat raw, img0, convertedColour;
cv::Mat resizedImage;

// rows by cols. We cv::resize our image before sending it over.
extern cv::Size resizeSize;
extern cv::Mat img1, img2;
extern int is_data_ready;
extern int clientSock;
extern char* server_ip;
extern int server_port;

extern pthread_mutex_t amutex;

void* streamClient(void* arg);
void quit(string msg, int retval);

int
main(int argc, char** argv) {
  pthread_t thread_c;
  int key;

  if(argc < 3) {
    quit("Usage: netcv_client <server_ip> <server_port> <input_file>(optional)", 0);
  }
  if(argc == 4) {
    capture.open(argv[3]);
  } else {
    capture.open(0);
  }

  if(!capture.isOpened()) {
    quit("\n--> cvCapture failed", 1);
  }

  server_ip = argv[1];
  server_port = atoi(argv[2]);

  capture >> raw;
  cv::resize(raw, convertedColour, resizeSize);
  cv::cvtColor(convertedColour, img0, COLOR_BGR2HLS);
  // cv::cvtColor(InputArray src, OutputArray dst, int code)

  // Image processing goes here?
  // cv::GaussianBlur(img0, img0, cv::Size(7,7), 1.5, 1.5);
  // cv::Canny(img0, img0, 0, 30, 3);

  img1 = cv::Mat::zeros(img0.rows, img0.cols, CV_8UC1);
  cout << "Image has " << img0.cols << " width  " << img0.rows << "height \n";

  // run the streaming client as a separate thread
  if(pthread_create(&thread_c, NULL, streamClient, NULL)) {
    quit("\n--> pthread_create failed.", 1);
  }

  cout << "\n--> Press 'q' to quit. \n\n" << endl;

  /* print the width and height of the frame, needed by the client */
  cout << "\n--> Transferring  (" << img0.cols << "x" << img0.rows << ")  images to the:  " << server_ip << ":" << server_port
       << endl;

  cv::namedWindow("stream_client", cv::WINDOW_AUTOSIZE);
  cv::flip(img0, img0, 1);
  cv::cvtColor(img0, img1, cv::COLOR_BGR2GRAY);

  while(key != 'q') {
    /* get a frame from camera */
    // capture >> img0;
    capture >> raw;
    cv::resize(raw, img0, resizeSize);
    if(img0.empty())
      break;

    pthread_mutex_lock(&amutex);

    cv::flip(img0, img0, 1);
    cv::cvtColor(img0, img1, cv::COLOR_BGR2GRAY);

    // Example image processing goes here?
    cv::GaussianBlur(img1, img1, cv::Size(7, 7), 1.5, 1.5);
    cv::Canny(img1, img1, 0, 30, 3);

    is_data_ready = 1;

    pthread_mutex_unlock(&amutex);

    /*also display the video here on client */

    cv::imshow("stream_client", img0);
    key = cv::waitKey(30);
  }

  /* user has pressed 'q', terminate the streaming client */
  if(pthread_cancel(thread_c)) {
    quit("\n--> pthread_cancel failed.", 1);
  }

  /* free memory */
  cv::destroyWindow("stream_client");
  quit("\n--> NULL", 0);
  return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * this function provides a way to exit nicely from the system
 */
void
quit(string msg, int retval) {
  if(retval == 0) {
    cout << (msg == "NULL" ? "" : msg) << "\n" << endl;
  } else {
    cerr << (msg == "NULL" ? "" : msg) << "\n" << endl;
  }
  if(clientSock) {
    close(clientSock);
  }
  if(capture.isOpened()) {
    capture.release();
  }
  if(!(img0.empty())) {
    (~img0);
  }
  if(!(img1.empty())) {
    (~img1);
  }
  if(!(img2.empty())) {
    (~img2);
  }
  pthread_mutex_destroy(&amutex);
  exit(retval);
}
