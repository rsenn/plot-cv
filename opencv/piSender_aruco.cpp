#include <ctime>
#include <iostream>
#include <raspicam/raspicam_cv.h>

#include <arpa/inet.h>

#include <iostream>
#include <pthread.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <aruco/aruco.h>

#include "videoSender.h"
#include "videoSender.cpp"

using namespace std;
using namespace cv;
using namespace aruco;

// rows by cols.
extern Size resizeSize;
extern Mat img1, img2;
extern int is_data_ready;
extern int clientSock;
extern char* server_ip;
extern int server_port;

extern pthread_mutex_t amutex;

raspicam::RaspiCam_Cv Camera;
cv::Mat image, img0, convertedColour, resizedImage;

// Globals for aruco are below.
MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
pair<double, double> AvrgTime(0, 0); // determines the average time required for detection
double ThresParam1, ThresParam2;

Mat TheInputImage, TheInputImageCopy;
CameraParameters TheCameraParameters;

int waitTime = 0;
int imagesAnalysed = 0;

void* streamClient(void* arg);
void quit(string msg, int retval);

int
main(int argc, char** argv) {

  time_t timer_begin, timer_end;

  pthread_t thread_c;
  int key;

  if(argc < 3) {
    quit("Usage: netcv_client <server_ip> <server_port> ", 0);
  }

  // Setup networking.
  server_ip = argv[1];
  server_port = atoi(argv[2]);

  // Setup aruco marker detection.

  MDetector.getThresholdParams(ThresParam1, ThresParam2);
  MDetector.setCornerRefinementMethod(MarkerDetector::SUBPIX);
  // james - make it go fast!
  MDetector.setDesiredSpeed(3);

  // set camera params
  Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);

  // Open camera
  cout << "Opening Camera..." << endl;
  if(!Camera.open()) {
    cerr << "Error opening the camera" << endl;
    return -1;
  }
  cout << "sleeping on startup because camera is slow to respond when first started?. Please wait." << '\n';
  sleep(1);
  // Start capture
  Camera.grab();
  Camera.retrieve(image);

  cv::resize(image, img0, resizeSize);
  img1 = cv::Mat::zeros(img0.rows, img0.cols, CV_8UC1);
  cout << "Image has " << img0.cols << " width  " << img0.rows << "height \n";

  // run the streaming client as a separate thread
  if(pthread_create(&thread_c, NULL, streamClient, NULL)) {
    quit("\n--> pthread_create failed.", 1);
  }

  cout << "\n--> Press 'q' to quit. \n\n" << endl;

  /* print the width and height of the frame, check this is set up the same on the receive side */
  cout << "\n--> Transferring  (" << img0.cols << "x" << img0.rows << ")  images to the:  " << server_ip << ":" << server_port << endl;

  /* Didn't compile with the namedwindow options because we don't have GTK.
  cv::namedWindow("stream_client", CV_WINDOW_AUTOSIZE);
                  flip(img0, img0, 1);
                  cvtColor(img0, img1, CV_BGR2GRAY);

  */

  while(key != 'q') {
    /* get a frame from camera */
    Camera.grab();
    Camera.retrieve(image);
    if(image.empty())
      break;

    TheInputImage = image;
    pthread_mutex_lock(&amutex);

    imagesAnalysed++;                     // number of images captured
    double tick = (double)getTickCount(); // for checking the speed
    // Detection of markers in the image passed
    MDetector.detect(TheInputImage, TheMarkers, TheCameraParameters, TheMarkerSize);
    // chekc the speed by calculating the mean speed of all iterations
    AvrgTime.first += ((double)getTickCount() - tick) / getTickFrequency();
    AvrgTime.second++;
    cout << "\rTime detection=" << 1000 * AvrgTime.first / AvrgTime.second << " milliseconds nmarkers=" << TheMarkers.size() << std::flush;

    // print marker info and draw the markers in image
    TheInputImage.copyTo(TheInputImageCopy);

    for(unsigned int i = 0; i < TheMarkers.size(); i++) {
      // Each element of the marker array is a marker, and the first four elements of the marker give the corners as xy
      // coordinates.  xy 0 is at the top left of the screen.  Print each of the four corners.
      // cout<<endl<<TheMarkers[i];
      circle(TheInputImageCopy, TheMarkers[i][0], 10, Scalar(0, 255, 0));
      cv::putText(TheInputImageCopy, "0", TheMarkers[i][0], fontFace, 0.8, Scalar::all(255));

      circle(TheInputImageCopy, TheMarkers[i][1], 10, Scalar(0, 255, 0));
      cv::putText(TheInputImageCopy, "1", TheMarkers[i][1], fontFace, 0.8, Scalar::all(255));

      TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255), 1);
    }
    if(TheMarkers.size() != 0)
      cout << endl;

    // Now repoint things so we can send data.
    cv::resize(TheInputImageCopy, img0, resizeSize);

    is_data_ready = 1;

    pthread_mutex_unlock(&amutex);
    usleep(1000); // sleep before sending next frame.

    // imshow("stream_client", img0);
    // key = cv::waitKey(30);
  }

  /* user has pressed 'q', terminate the streaming client */
  if(pthread_cancel(thread_c)) {
    quit("\n--> pthread_cancel failed.", 1);
  }
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
  if(Camera.isOpened()) {
    Camera.release();
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
