// original by Cristóbal Carnero Liñán <grendel.ccl@gmail.dcom>
// modified by Brian Gravelle gravelle@cs.uoregon.edu

#include <iostream>
#include <iomanip>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <cvblob.h>
#include <stdio.h>
#include <stdlib.h>

//using namespace cvb;
using namespace std;

#define VIDEO "/home/pi/test_videos/red_lego_LR.h264"

int
main() {
  CvTracks tracks;

  cvNamedWindow("red_object_tracking", cv::WINDOW_NORMAL);

  // CvCapture *capture = cvCaptureFromCAM(0);
  cv::VideoCapture capture;
  capture.open(VIDEO);
  if(!capture.isOpened()) {
    cout << "ERROR ACQUIRING VIDEO FEED\n";
    getchar();
    return -1;
  }

  IplConvKernel* morphKernel = cvCreateStructuringElementEx(5, 5, 1, 1, CV_SHAPE_RECT, NULL);

  // unsigned int frameNumber = 0;
  unsigned int blobNumber = 0;
  bool image_success = false;

  cv::Mat img_mat1, img_mat2, diff_mat, thrs_mat, bw_mat1, bw_mat2;

  image_success = capture.cv::read(img_mat1);
  if(!image_success) {
    cout << endl << "ERROR: img_mat1 failed to be cv::read" << endl;
    exit(1);
  }
  cv::cvtColor(img_mat1, bw_mat1, cv::COLOR_BGR2GRAY);

  IplImage thrs_img = bw_mat1;
  IplImage* thrs_img_p = &thrs_img;
  IplImage* label_img_p = cvCreateImage(cvSize(bw_mat1.cols, bw_mat1.rows), IPL_DEPTH_LABEL, 1);
  IplImage frame_img = img_mat1;
  IplImage* frame_img_p = &frame_img;

  // cv::Mat mat_img;
  // IplImage ipl_img = mat_img;
  // IplImage ipl_img;
  // cv::Mat mat_img(ipl_img );

  image_success = capture.cv::read(img_mat2);
  if(!image_success) {
    cout << endl << "ERROR: img_mat2 failed to be cv::read" << endl;
    exit(1);
  }
  cv::cvtColor(img_mat2, bw_mat2, cv::COLOR_BGR2GRAY);

  bool quit = false;
  while(image_success && !quit) {

    cout << "in the loop-d-loop" << endl;

    image_success = capture.cv::read(img_mat2);
    if(!image_success) {
      cout << endl << "ERROR: img_mat2 failed to be cv::read" << endl;
      exit(1);
    }
    cv::cvtColor(img_mat2, bw_mat2, cv::COLOR_BGR2GRAY);

    cv::absdiff(bw_mat1, bw_mat2, diff_mat);
    cv::threshold(diff_mat, thrs_mat, 50, 255, cv::THRESH_BINARY);
    cv::blur(thrs_mat, thrs_mat, cv::Size(200, 200));
    cv::threshold(thrs_mat, thrs_mat, 50, 255, cv::THRESH_BINARY);

    thrs_img = thrs_mat;
    thrs_img_p = &thrs_img;

    cout << "did cv::threshold stuff" << endl;

    CvBlobs blobs;
    unsigned int result = cvLabel(thrs_img_p, label_img_p, blobs); // I think this one makes the blobs
    cout << "blob 0" << endl;

    cvFilterByArea(blobs, 500,
                   1000000); // this one filters blobs by area (limited to range provided)
    cout << "blob 1" << endl;

    cvRenderBlobs(label_img_p, blobs, frame_img_p, frame_img_p, CV_BLOB_RENDER_BOUNDING_BOX);
    cout << "blob 2" << endl;

    cvUpdateTracks(blobs, tracks, 200., 5);
    cout << "blob 3" << endl;

    cvRenderTracks(tracks, frame_img_p, frame_img_p, CV_TRACK_RENDER_ID | CV_TRACK_RENDER_BOUNDING_BOX);

    cvShowImage("red_object_tracking", frame_img_p);
    cvResizeWindow("red_object_tracking", 1024, 768);

    /*std::stringstream filename;
    filename << "redobject_" << std::setw(5) << std::setfill('0') << frameNumber << ".png";
    cvSaveImage(filename.str().c_str(), frame);*/

    bw_mat1 = bw_mat2;

    char k = cvWaitKey(10) & 0xff;
    switch(k) {
      case 27:
      case 'q':
      case 'Q': quit = true; break;
    } // switch

    cvReleaseBlobs(blobs);

    image_success = capture.cv::read(img_mat2);
    if(!image_success) {
      cout << endl << "ERROR: img_mat2 failed to be cv::read" << endl;
      exit(1);
    }
    cv::cvtColor(img_mat2, bw_mat2, cv::COLOR_BGR2GRAY);

    // frameNumber++;
  } // while

  cvReleaseImage(&thrs_img_p);
  cvReleaseImage(&label_img_p);
  cvReleaseImage(&frame_img_p);
  cvReleaseStructuringElement(&morphKernel);

  cvDestroyWindow("red_object_tracking");

  return 0;
}
