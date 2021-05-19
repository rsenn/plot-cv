// include section
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
// using namespace cv;

// finds the angles for shape detection Status: Good to go
double angle(cv::Point* pt1, cv::Point* pt2, cv::Point* pt0);

// This is for color filtering Status: Doesn't Work Yet
IplImage* GetThresholdedImage(IplImage* imgHSV);

// flag definitions 1: HSV color filtering; 2: cv::Canny Edge Detect; 3: HSV Filter with canny filtering;
////Change these values to get what you need
int flag = 1;

// These values change the HSV filtering values.
int Hue_Min = 112;
int Hue_Max = 251;

int Saturation_Min = 0;
int Saturation_Max = 256;

int Value_Min = 38;
int Value_Max = 218;

int
main() {
  // Make the windows
  cv::namedWindow("Thresholded", cv::WINDOW_NORMAL);
  cv::namedWindow("Tracked", cv::WINDOW_NORMAL);
  cv::namedWindow("Original", cv::WINDOW_NORMAL);

  // Gets stuff from camera
  CvCapture* capture = cv::CaptureFromCAM(0);

  // This variable will hold all the frames. It will hold only one frame on each iteration of the loop.
  IplImage* frame;

  while(1) {
    // Gets Frame from camera
    std::cout << "frame capture\n";
    frame = cv::QueryFrame(capture);
    std::cout << "Check\n";

    // puts the original image in the window
    cv::ShowImage("Original", frame);

    std::cout << "declare imgGrayScale\n";
    IplImage* imgGrayScale;
    std::cout << "check\n";

    // Use the Pyramid thing rob has been working on here
    //
    // cv::Smooth( frame, frame, CV_GAUSSIAN, 5, 5 );

    if(flag == 1) {
      // Filter unwanted colors out
      std::cout << "HSV Flag Active\n";
      std::cout << "HSV Color Filter\n";
      imgGrayScale = GetThresholdedImage(frame);
      std::cout << "Check\n";
    }

    if(flag == 2) {
      std::cout << "Grayscale HSV Flag Active\n";
      // Making a single channel matrix so that edge detection can work properly
      std::cout << "Grayscale Image\n";
      imgGrayScale = cv::CreateImage(cvGetSize(frame), 8, 1);
      cv::CvtColor(frame, imgGrayScale, cv::COLOR_BGR2GRAY);
      std::cout << "Check\n";

      // This thresholds the grayscale image to be tested on
      std::cout << "cv::Canny Threshold Image\n";
      // cv::Threshold(imgGrayScale,imgGrayScale,100,255,cv::THRESH_BINARY | cv::THRESH_OTSU);
      cv::Canny(imgGrayScale, imgGrayScale, 200, 400, 3);
      std::cout << "Check\n";
    }
    if(flag == 3) {
      std::cout << "HSV cv::Canny Flag Active\n";
      std::cout << "HSV Color Filter\n";
      imgGrayScale = GetThresholdedImage(frame);
      std::cout << "Check\n";
      std::cout << "cv::Canny Threshold Image\n";
      // cv::Threshold(imgGrayScale,imgGrayScale,100,255,cv::THRESH_BINARY | cv::THRESH_OTSU);
      cv::Canny(imgGrayScale, imgGrayScale, 100, 100, 3);
      std::cout << "Check\n";
    }

    std::cout << "Contour Allocation\n";
    CvSeq* contours;                                 // hold the pointer to a contour in the memory block
    CvSeq* result;                                   // hold sequence of points of a contour
    CvMemStorage* storage = cv::CreateMemStorage(0); // storage area for all contours

    std::cout << "Check\n";

    std::cout << "Find Contours\n";
    // finding all contours in the image
    cv::FindContours(
        imgGrayScale, storage, &contours, sizeof(CvContour), cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    std::cout << "Check\n";
    while(contours) {

      // obtain a sequence of points of contour, pointed by the variable 'contour'
      result = cv::ApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours) * 0.02, 0);

      // Triangle Detection
      // if there are 3  vertices  in the contour(It should be a triangle)
      if(result->cv::total == 3) {
        // iterating through each point
        cv::Point* cv::pt[3];
        for(int i = 0; i < 3; i++) { cv::pt[i] = (cv::Point*)cv::GetSeqElem(result, i); }

        // This If Statement ensures that the edges are sufficiently large enough to be detected
        if(abs(cv::pt[1]->x - pt[0]->x) > 10 && abs(pt[1]->x - pt[2]->x) > 10 && abs(pt[2]->x - pt[0]->x) > 10) {
          //////////drawing lines around the triangle
          cv::line(frame, *cv::pt[0], *pt[1], cv::Scalar(255, 0, 0), 4);
          cv::line(frame, *cv::pt[1], *pt[2], cv::Scalar(255, 0, 0), 4);
          cv::line(frame, *cv::pt[2], *pt[0], cv::Scalar(255, 0, 0), 4);
          std::cout << "\nTriangle\n";
        }
      }

      // Rectangle detection
      // if there are 4 vertices in the contour(It should be a quadrilateral)
      else if(result->cv::total == 4) {
        // iterating through each point
        cv::Point* cv::pt[4];
        for(int i = 0; i < 4; i++) { cv::pt[i] = (cv::Point*)cv::GetSeqElem(result, i); }

        // finding angles
        double firstAngle = acos(angle(cv::pt[0], pt[2], pt[1]));
        double secondAngle = acos(angle(cv::pt[1], pt[3], pt[2]));
        double thirdAngle = acos(angle(cv::pt[1], pt[3], pt[2]));
        double fourthAngle = acos(angle(cv::pt[0], pt[2], pt[3]));

        // This If Statement Ensures that the edges are sufficiently large
        if(abs(cv::pt[1]->x - pt[0]->x) > 10 && abs(pt[1]->x - pt[2]->x) > 10 && abs(pt[2]->x - pt[3]->x) > 10 &&
           abs(cv::pt[3]->x - pt[0]->x) > 10) {

          // This if statement checks the angles to see if its a cv::rectangle or not (90 angles with 10%
          // uncertainty)
          if(firstAngle <= 1.884 && firstAngle >= 1.308 && secondAngle <= 1.884 && secondAngle >= 1.308 &&
             thirdAngle <= 1.884 && thirdAngle >= 1.308 && fourthAngle <= 1.884 && fourthAngle >= 1.308) {
            // drawing lines around the quadrilateral
            cv::line(frame, *cv::pt[0], *pt[1], cv::Scalar(0, 255, 0), 4);
            cv::line(frame, *cv::pt[1], *pt[2], cv::Scalar(0, 255, 0), 4);
            cv::line(frame, *cv::pt[2], *pt[3], cv::Scalar(0, 255, 0), 4);
            cv::line(frame, *cv::pt[3], *pt[0], cv::Scalar(0, 255, 0), 4);

            std::cout << "\nsquare\n";
            // cout << firstAngle; //Uncomment this to get the angles that its detecting.
          }
        }
      }
      contours = contours->h_next;
    }

    // Put the images in the frame
    cv::ShowImage("Tracked", frame);
    cv::ShowImage("Thresholded", imgGrayScale);

    char c = cv::waitKey(33);

    if(c == 27) {
      // cleaning up
      cv::destroyAllWindows();
      cv::ReleaseImage(&frame);
      cv::ReleaseImage(&imgGrayScale);
      cv::ReleaseMemStorage(&storage);
      break;
    }

    // for(int i=1;i<100000000/5;i++);
    cv::ReleaseImage(&imgGrayScale);
    cv::ReleaseMemStorage(&storage);
  }

  return 0;
}

IplImage*
GetThresholdedImage(IplImage* imgHSV) {
  IplImage* imgThresh = cv::CreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);
  cv::InRangeS(imgHSV,
               cv::Scalar(Hue_Min, Saturation_Min, Value_Min),
               cv::Scalar(Hue_Max, Saturation_Max, Value_Max),
               imgThresh);
  return imgThresh;
}

double
angle(cv::Point* pt1, cv::Point* pt2, cv::Point* pt0) {
  double dx1 = pt1->x - pt0->x;
  double dy1 = pt1->y - pt0->y;
  double dx2 = pt2->x - pt0->x;
  double dy2 = pt2->y - pt0->y;
  return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}
