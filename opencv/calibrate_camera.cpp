/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include <ctime>

using namespace std;
// using namespace cv;

namespace {
const char* about = "Calibration using a ArUco Planar Grid board\n"
                    "  To capture a frame for calibration, press 'c',\n"
                    "  If input comes from video, press any key for next frame\n"
                    "  To finish capturing, press 'ESC' key and calibration starts.\n";
const char* keys = "{w        |       | Number of squares in X direction }"
                   "{h        |       | Number of squares in Y direction }"
                   "{l        |       | Marker side length (in meters) }"
                   "{s        |       | Separation between two consecutive markers in the grid (in meters) }"
                   "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
                   "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
                   "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
                   "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
                   "{@outfile |<none> | Output file with calibrated camera parameters }"
                   "{v        |       | Input from video file, if ommited, input comes from camera }"
                   "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
                   "{dp       |       | File of marker detector parameters }"
                   "{rs       | false | Apply refind strategy }"
                   "{zt       | false | Assume zero tangential distortion }"
                   "{a        |       | Fix aspect ratio (fx/fy) to this value }"
                   "{pc       | false | Fix the principal point at the center }";
} // namespace

/**
 */
static bool
readDetectorParameters(string filename, cv::Ptr<cv::aruco::DetectorParameters>& params) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if(!fs.isOpened())
    return false;
  fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
  fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
  fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
  fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
  fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
  fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
  fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
  fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
  fs["minDistanceToBorder"] >> params->minDistanceToBorder;
  fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
  fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
  fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
  fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
  fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
  fs["markerBorderBits"] >> params->markerBorderBits;
  fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
  fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
  fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
  fs["minOtsuStdDev"] >> params->minOtsuStdDev;
  fs["errorCorrectionRate"] >> params->errorCorrectionRate;
  return true;
}

/**
 */
static bool
saveCameraParams(const string& filename,
                 cv::Size imageSize,
                 float aspectRatio,
                 int flags,
                 const cv::Mat& cameraMatrix,
                 const cv::Mat& distCoeffs,
                 double totalAvgErr) {
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  if(!fs.isOpened())
    return false;

  time_t tt;
  time(&tt);
  struct tm* t2 = localtime(&tt);
  char buf[1024];
  strftime(buf, sizeof(buf) - 1, "%c", t2);

  fs << "calibration_time" << buf;

  fs << "image_width" << imageSize.width;
  fs << "image_height" << imageSize.height;

  if(flags & cv::CALIB_FIX_ASPECT_RATIO)
    fs << "aspectRatio" << aspectRatio;

  if(flags != 0) {
    sprintf(buf,
            "flags: %s%s%s%s",
            flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
            flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
            flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
            flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
  }

  fs << "flags" << flags;

  fs << "camera_matrix" << cameraMatrix;
  fs << "distortion_coefficients" << distCoeffs;

  fs << "avg_reprojection_error" << totalAvgErr;

  return true;
}

/**
 */
int
main(int argc, char* argv[]) {
  cv::CommandLineParser parser(argc, argv, keys);
  parser.about(about);

  if(argc < 6) {
    parser.printMessage();
    return 0;
  }

  int markersX = parser.get<int>("w");
  int markersY = parser.get<int>("h");
  float markerLength = parser.get<float>("l");
  float markerSeparation = parser.get<float>("s");
  int dictionaryId = parser.get<int>("d");
  string outputFile = parser.get<cv::String>(0);

  int calibrationFlags = 0;
  float aspectRatio = 1;
  if(parser.has("a")) {
    calibrationFlags |= cv::CALIB_FIX_ASPECT_RATIO;
    aspectRatio = parser.get<float>("a");
  }
  if(parser.get<bool>("zt"))
    calibrationFlags |= cv::CALIB_ZERO_TANGENT_DIST;
  if(parser.get<bool>("pc"))
    calibrationFlags |= cv::CALIB_FIX_PRINCIPAL_POINT;

  cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
  if(parser.has("dp")) {
    bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
    if(!readOk) {
      cerr << "Invalid detector parameters file" << endl;
      return 0;
    }
  }

  bool refindStrategy = parser.get<bool>("rs");
  int camId = parser.get<int>("ci");
  cv::String video;

  if(parser.has("v")) {
    video = parser.get<cv::String>("v");
  }

  if(!parser.check()) {
    parser.printErrors();
    return 0;
  }

  cv::VideoCapture inputVideo;
  int waitTime;
  if(!video.empty()) {
    inputVideo.open(video);
    waitTime = 0;
  } else {
    inputVideo.open(camId);
    waitTime = 10;
  }

  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

  // create board object
  cv::Ptr<cv::aruco::GridBoard> gridboard =
      cv::aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
  cv::Ptr<cv::aruco::Board> board = gridboard.staticCast<cv::aruco::Board>();

  // collected frames for calibration
  vector<vector<vector<cv::Point2f>>> allCorners;
  vector<vector<int>> allIds;
  cv::Size imgSize;

  while(inputVideo.grab()) {
    cv::Mat image, imageCopy;
    inputVideo.retrieve(image);

    vector<int> ids;
    vector<vector<cv::Point2f>> corners, rejected;

    // detect markers
    cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

    // refind strategy to detect more markers
    if(refindStrategy)
      cv::aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

    // draw results
    image.copyTo(imageCopy);
    if(ids.size() > 0)
      cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
    cv::putText(imageCopy,
                "Press 'c' to cv::add current frame. 'ESC' to finish and calibrate",
                cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(255, 0, 0),
                2);

    cv::imshow("out", imageCopy);
    char key = (char)cv::waitKey(waitTime);
    if(key == 27)
      break;
    if(key == 'c' && ids.size() > 0) {
      cout << "Frame captured" << endl;
      allCorners.push_back(corners);
      allIds.push_back(ids);
      imgSize = image.size();
    }
  }

  if(allIds.size() < 1) {
    cerr << "Not enough captures for calibration" << endl;
    return 0;
  }

  cv::Mat cameraMatrix, distCoeffs;
  vector<cv::Mat> rvecs, tvecs;
  double repError;

  if(calibrationFlags & cv::CALIB_FIX_ASPECT_RATIO) {
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = aspectRatio;
  }

  // prepare data for calibration
  vector<vector<cv::Point2f>> allCornersConcatenated;
  vector<int> allIdsConcatenated;
  vector<int> markerCounterPerFrame;
  markerCounterPerFrame.reserve(allCorners.size());
  for(unsigned int i = 0; i < allCorners.size(); i++) {
    markerCounterPerFrame.push_back((int)allCorners[i].size());
    for(unsigned int j = 0; j < allCorners[i].size(); j++) {
      allCornersConcatenated.push_back(allCorners[i][j]);
      allIdsConcatenated.push_back(allIds[i][j]);
    }
  }
  // calibrate camera
  repError = cv::aruco::calibrateCameraAruco(allCornersConcatenated,
                                             allIdsConcatenated,
                                             markerCounterPerFrame,
                                             board,
                                             imgSize,
                                             cameraMatrix,
                                             distCoeffs,
                                             rvecs,
                                             tvecs,
                                             calibrationFlags);

  bool saveOk = saveCameraParams(outputFile, imgSize, aspectRatio, calibrationFlags, cameraMatrix, distCoeffs, repError);

  if(!saveOk) {
    cerr << "Cannot save output file" << endl;
    return 0;
  }

  cout << "Rep cv::Error: " << repError << endl;
  cout << "Calibration saved to " << outputFile << endl;

  return 0;
}
