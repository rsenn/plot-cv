/*****************************************************************************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************************************************************************/
#include <iostream>
#include <cmath>
#include <fstream>
#include <sstream>
#include <aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
using namespace aruco;

int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;

string TheInputVideo;
string TheIntrinsicFile;
float TheMarkerSize = -1;
int ThePyrDownLevel;
MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage, TheInputImageCopy;
CameraParameters TheCameraParameters;
void cvTackBarEvents(int pos, void*);
bool readCameraParameters(string TheIntrinsicFile, CameraParameters& CP, Size size);

pair<double, double> AvrgTime(0, 0); // determines the average time required for detection
double ThresParam1, ThresParam2;
int iThresParam1, iThresParam2;
int waitTime = 0;

/************************************
 *
 *
 *
 *
 ************************************/
bool
readArguments(int argc, char** argv) {
  if(argc < 2) {
    cerr << "Invalid number of arguments" << endl;
    cerr << "Usage: (in.avi|live[:idx_cam=0]) [intrinsics.yml] [size]" << endl;
    return false;
  }
  TheInputVideo = argv[1];
  if(argc >= 3)
    TheIntrinsicFile = argv[2];
  if(argc >= 4)
    TheMarkerSize = atof(argv[3]);

  if(argc == 3)
    cerr << "NOTE: You need makersize to see 3d info!!!!" << endl;
  return true;
}

int
findParam(std::string param, int argc, char* argv[]) {
  for(int i = 0; i < argc; i++)
    if(string(argv[i]) == param)
      return i;

  return -1;
}
/************************************
 *
 *
 *
 *
 ************************************/
int
main(int argc, char** argv) {
  try {
    if(readArguments(argc, argv) == false) {
      return 0;
    }
    // parse arguments

    // read from camera or from  file
    if(TheInputVideo.find("live") != string::npos) {
      int vIdx = 0;
      // check if the :idx is here
      char cad[100];
      if(TheInputVideo.find(":") != string::npos) {
        std::replace(TheInputVideo.begin(), TheInputVideo.end(), ':', ' ');
        sscanf(TheInputVideo.c_str(), "%s %d", cad, &vIdx);
      }
      cout << "Opening camera index " << vIdx << endl;
      TheVideoCapturer.open(vIdx);
      waitTime = 10;
    } else
      TheVideoCapturer.open(TheInputVideo);
    // check video is open
    if(!TheVideoCapturer.isOpened()) {
      cerr << "Could not open video" << endl;
      return -1;
    }

    // read first image to get the dimensions
    TheVideoCapturer >> TheInputImage;

    // read camera parameters if passed
    if(TheIntrinsicFile != "") {
      TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
      TheCameraParameters.resize(TheInputImage.size());
    }
    // Configure other parameters
    if(ThePyrDownLevel > 0)
      MDetector.pyrDown(ThePyrDownLevel);

    // Create gui

    cv::namedWindow("thres", 1);
    cv::namedWindow("in", 1);
    MDetector.getThresholdParams(ThresParam1, ThresParam2);
    MDetector.setCornerRefinementMethod(MarkerDetector::SUBPIX);
    // james - make it go fast!
    MDetector.setDesiredSpeed(3);
    iThresParam1 = ThresParam1;
    iThresParam2 = ThresParam2;
    cv::createTrackbar("ThresParam1", "in", &iThresParam1, 13, cvTackBarEvents);
    cv::createTrackbar("ThresParam2", "in", &iThresParam2, 13, cvTackBarEvents);

    char key = 0;
    int index = 0;

    // this is the x y separation between two points on the image. These are 2.5cm away in real life.
    float separation = 0;
    // capture until press ESC or until the end of the video
    do {
      TheVideoCapturer.retrieve(TheInputImage);
      // copy image

      index++;                              // number of images captured
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
        // Each element of the marker array is a marker, and the first four elements of the marker give the corners as
        // xy coordinates.  xy 0 is at the top left of the screen.  Print each of the four corners.
        // cout<<endl<<TheMarkers[i];
        circle(TheInputImageCopy, TheMarkers[i][0], 10, Scalar(0, 255, 0));
        cv::putText(TheInputImageCopy, "0", TheMarkers[i][0], fontFace, 0.8, Scalar::all(255));

        circle(TheInputImageCopy, TheMarkers[i][1], 10, Scalar(0, 255, 0));
        cv::putText(TheInputImageCopy, "1", TheMarkers[i][1], fontFace, 0.8, Scalar::all(255));

        TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255), 1);
      }
      if(TheMarkers.size() != 0)
        cout << endl;

      if(TheMarkers.size() != 0) {
        // Then we have markers. Let's calculate their distance from the camera.
        cv::Point2f first, second, third, fourth, diffs, sep, centre;
        int i = 0;

        first = TheMarkers[i][0];
        second = TheMarkers[i][1];
        third = TheMarkers[i][2];
        fourth = TheMarkers[i][3];
        // Get center as average of top left and bottom right
        centre.x = (first.x + third.x) / 2.0;
        centre.y = (first.y + third.y) / 2.0;

        // Get differences top left minus bottom left
        diffs.x = first.x - fourth.x;
        diffs.y = first.y - fourth.x;

        // get rotation in degrees.
        //  var rotation = Math.atan(diffs[0]/diffs[1]) * 180 / Math.PI;
        float rotation = atan(diffs.x / diffs.y) * 180.0 / M_PI;

        if(diffs.y < 0) {
          rotation += 180;
        } else if(diffs.x < 0) {
          rotation += 360;
        }

        cout << " centre " << centre << " rotation " << rotation << endl;
        circle(TheInputImageCopy, centre, 10, Scalar(0, 255, 0));

        sep = first - second;
        cout << first << second << sep << endl;

        // marker is 160px wide when one handswidth or 6 inches or 15cm from screen.
        // F = (PxD)/2
        // focal length = (160 pixels x 15)/2.5
        // = 960.

        // and distance = D = (known width x focal length)/Pixels .

        separation = (2.5 * 960) / sep.x;
      }

      cv::putText(TheInputImageCopy, "test", Point(400, 400), fontFace, 1.0, Scalar::all(255));

      // print other rectangles that contains no valid markers
      /*
      for (unsigned int i=0;i<MDetector.getCandidates().size();i++) {
          aruco::Marker m( MDetector.getCandidates()[i],999);
          m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
      }
      */

      // draw a 3d cube in each marker if there is 3d info
      if(TheCameraParameters.isValid())
        for(unsigned int i = 0; i < TheMarkers.size(); i++) {
          CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
          CvDrawingUtils::draw3dAxis(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
        }
      // DONE! Easy, right?
      // show input with augmented information and  the thresholded image
      cv::imshow("in", TheInputImageCopy);
      cv::imshow("thres", MDetector.getThresholdedImage());

      key = cv::waitKey(waitTime); // wait for key to be pressed
    } while(key != 27 && TheVideoCapturer.grab());

  } catch(std::exception& ex)

  {
    cout << "Exception :" << ex.what() << endl;
  }
}
/************************************
 *
 *
 *
 *
 ************************************/

void
cvTackBarEvents(int pos, void*) {
  if(iThresParam1 < 3)
    iThresParam1 = 3;
  if(iThresParam1 % 2 != 1)
    iThresParam1++;
  if(ThresParam2 < 1)
    ThresParam2 = 1;
  ThresParam1 = iThresParam1;
  ThresParam2 = iThresParam2;
  MDetector.setThresholdParams(ThresParam1, ThresParam2);
  // recompute
  MDetector.detect(TheInputImage, TheMarkers, TheCameraParameters);
  TheInputImage.copyTo(TheInputImageCopy);
  for(unsigned int i = 0; i < TheMarkers.size(); i++) TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255), 1);
  // print other rectangles that contains no valid markers
  /*for (unsigned int i=0;i<MDetector.getCandidates().size();i++) {
      aruco::Marker m( MDetector.getCandidates()[i],999);
      m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
  }*/

  // draw a 3d cube in each marker if there is 3d info
  if(TheCameraParameters.isValid())
    for(unsigned int i = 0; i < TheMarkers.size(); i++) CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);

  cv::imshow("in", TheInputImageCopy);
  cv::imshow("thres", MDetector.getThresholdedImage());
}
