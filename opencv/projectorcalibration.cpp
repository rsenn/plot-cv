/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2015, OpenCV Foundation, all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#include <opencv2/highgui.hpp>
#include <vector>
#include <iostream>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
// using namespace cv;

static const char* keys = {"{@camSettingsPath | | Path of camera calibration file}"
                           "{@projSettingsPath | | Path of projector settings}"
                           "{@patternPath | | Path to checkerboard pattern}"
                           "{@outputName | | Base name for the calibration data}"};

static void
help() {
  cout << "\nThis example calibrates a camera and a projector" << endl;
  cout << "To call: ./example_structured_light_projectorcalibration <cam_settings_path> "
          " <proj_settings_path> <chessboard_path> <calibration_basename>"
          " cam settings are parameters about the chessboard that needs to be detected to"
          " calibrate the camera and proj setting are the same kind of parameters about the "
          "chessboard"
          " that needs to be detected to calibrate the projector"
       << endl;
}
enum calibrationPattern { CHESSBOARD, CIRCLES_GRID, ASYMETRIC_CIRCLES_GRID };

struct Settings {
  Settings();
  int patternType;
  cv::Size patternSize;
  cv::Size subpixelSize;
  cv::Size imageSize;
  float squareSize;
  int nbrOfFrames;
};

void loadSettings(cv::String path, Settings& sttngs);

void createObjectPoints(vector<cv::Point3f>& patternCorners, cv::Size patternSize, float squareSize, int patternType);

void createProjectorObjectPoints(vector<cv::Point2f>& patternCorners, cv::Size patternSize, float squareSize, int patternType);

double calibrate(vector<vector<cv::Point3f>> objPoints,
                 vector<vector<cv::Point2f>> imgPoints,
                 cv::Mat& cameraMatrix,
                 cv::Mat& distCoeffs,
                 vector<cv::Mat>& r,
                 vector<cv::Mat>& t,
                 cv::Size imgSize);

void fromCamToWorld(cv::Mat cameraMatrix,
                    vector<cv::Mat> rV,
                    vector<cv::Mat> tV,
                    vector<vector<cv::Point2f>> imgPoints,
                    vector<vector<cv::Point3f>>& worldPoints);

void saveCalibrationResults(
    cv::String path, cv::Mat camK, cv::Mat camDistCoeffs, cv::Mat projK, cv::Mat projDistCoeffs, cv::Mat fundamental);

void saveCalibrationData(cv::String path,
                         vector<cv::Mat> T1,
                         vector<cv::Mat> T2,
                         vector<cv::Mat> ptsProjCam,
                         vector<cv::Mat> ptsProjProj,
                         vector<cv::Mat> ptsProjCamN,
                         vector<cv::Mat> ptsProjProjN);

void normalize(const cv::Mat& pts, const int& dim, cv::Mat& normpts, cv::Mat& T);

void fromVectorToMat(vector<cv::Point2f> v, cv::Mat& pts);

void fromMatToVector(cv::Mat pts, vector<cv::Point2f>& v);

int
main(int argc, char** argv) {
  cv::VideoCapture cap(cv::CAP_PVAPI);
  cv::Mat frame;

  int nbrOfValidFrames = 0;

  vector<vector<cv::Point2f>> imagePointsCam, imagePointsProj, PointsInProj, imagePointsProjN, pointsInProjN;
  vector<vector<cv::Point3f>> objectPointsCam, worldPointsProj;
  vector<cv::Point3f> tempCam;
  vector<cv::Point2f> tempProj;
  vector<cv::Mat> T1, T2;
  vector<cv::Mat> projInProj, projInCam;
  vector<cv::Mat> projInProjN, projInCamN;

  vector<cv::Mat> rVecs, tVecs, projectorRVecs, projectorTVecs;
  cv::Mat cameraMatrix, distCoeffs, projectorMatrix, projectorDistCoeffs;
  cv::Mat pattern;
  vector<cv::Mat> images;

  Settings camSettings, projSettings;

  cv::CommandLineParser parser(argc, argv, keys);

  cv::String camSettingsPath = parser.get<cv::String>(0);
  cv::String projSettingsPath = parser.get<cv::String>(1);
  cv::String patternPath = parser.get<cv::String>(2);
  cv::String outputName = parser.get<cv::String>(3);

  if(camSettingsPath.empty() || projSettingsPath.empty() || patternPath.empty() || outputName.empty()) {
    help();
    return -1;
  }

  pattern = cv::imread(patternPath);

  loadSettings(camSettingsPath, camSettings);
  loadSettings(projSettingsPath, projSettings);

  projSettings.imageSize = cv::Size(pattern.rows, pattern.cols);

  createObjectPoints(tempCam, camSettings.patternSize, camSettings.squareSize, camSettings.patternType);
  createProjectorObjectPoints(tempProj, projSettings.patternSize, projSettings.squareSize, projSettings.patternType);

  if(!cap.isOpened()) {
    cout << "Camera could not be opened" << endl;
    return -1;
  }
  cap.set(cv::CAP_PROP_PVAPI_PIXELFORMAT, cv::CAP_PVAPI_PIXELFORMAT_BAYER8);

  cv::namedWindow("pattern", cv::WINDOW_NORMAL);
  cv::setWindowProperty("pattern", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

  cv::namedWindow("camera view", cv::WINDOW_NORMAL);

  cv::imshow("pattern", pattern);
  cout << "Press any key when ready" << endl;
  cv::waitKey(0);

  while(nbrOfValidFrames < camSettings.nbrOfFrames) {
    cap >> frame;
    if(frame.data) {
      cv::Mat color;
      cv::cvtColor(frame, color, cv::COLOR_BayerBG2BGR);
      if(camSettings.imageSize.height == 0 || camSettings.imageSize.width == 0) {
        camSettings.imageSize = cv::Size(frame.rows, frame.cols);
      }

      bool foundProj, foundCam;

      vector<cv::Point2f> projPointBuf;
      vector<cv::Point2f> camPointBuf;

      cv::imshow("camera view", color);
      if(camSettings.patternType == CHESSBOARD && projSettings.patternType == CHESSBOARD) {
        int calibFlags = cv::CALIB_CB_ADAPTIVE_THRESH;

        foundCam = cv::findChessboardCorners(color, camSettings.patternSize, camPointBuf, calibFlags);

        foundProj = cv::findChessboardCorners(color, projSettings.patternSize, projPointBuf, calibFlags);

        if(foundCam && foundProj) {
          cv::Mat gray;
          cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
          cout << "found pattern" << endl;
          cv::Mat projCorners, camCorners;
          cv::cornerSubPix(gray,
                           camPointBuf,
                           camSettings.subpixelSize,
                           cv::Size(-1, -1),
                           cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.1));

          cv::cornerSubPix(gray,
                           projPointBuf,
                           projSettings.subpixelSize,
                           cv::Size(-1, -1),
                           cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.1));

          cv::drawChessboardCorners(gray, camSettings.patternSize, camPointBuf, foundCam);
          cv::drawChessboardCorners(gray, projSettings.patternSize, projPointBuf, foundProj);

          cv::imshow("camera view", gray);
          char c = (char)cv::waitKey(0);
          if(c == 10) {
            cout << "saving pattern #" << nbrOfValidFrames << " for calibration" << endl;
            ostringstream name;
            name << nbrOfValidFrames;
            nbrOfValidFrames += 1;

            imagePointsCam.push_back(camPointBuf);
            imagePointsProj.push_back(projPointBuf);
            objectPointsCam.push_back(tempCam);
            PointsInProj.push_back(tempProj);
            images.push_back(frame);

            cv::Mat ptsProjProj, ptsProjCam;
            cv::Mat ptsProjProjN, ptsProjCamN;
            cv::Mat TProjProj, TProjCam;
            vector<cv::Point2f> ptsProjProjVec;
            vector<cv::Point2f> ptsProjCamVec;

            fromVectorToMat(tempProj, ptsProjProj);
            normalize(ptsProjProj, 2, ptsProjProjN, TProjProj);
            fromMatToVector(ptsProjProjN, ptsProjProjVec);
            pointsInProjN.push_back(ptsProjProjVec);
            T2.push_back(TProjProj);
            projInProj.push_back(ptsProjProj);
            projInProjN.push_back(ptsProjProjN);

            fromVectorToMat(projPointBuf, ptsProjCam);
            normalize(ptsProjCam, 2, ptsProjCamN, TProjCam);
            fromMatToVector(ptsProjCamN, ptsProjCamVec);
            imagePointsProjN.push_back(ptsProjCamVec);
            T1.push_back(TProjCam);
            projInCam.push_back(ptsProjCam);
            projInCamN.push_back(ptsProjCamN);

          } else if(c == 32) {
            cout << "capture discarded" << endl;
          } else if(c == 27) {
            cout << "closing program" << endl;
            return -1;
          }
        } else {
          cout << "no pattern found, move board and press any key" << endl;
          cv::imshow("camera view", frame);
          cv::waitKey(0);
        }
      }
    }
  }

  saveCalibrationData(outputName + "_points.yml", T1, T2, projInCam, projInProj, projInCamN, projInProjN);

  double rms = calibrate(objectPointsCam, imagePointsCam, cameraMatrix, distCoeffs, rVecs, tVecs, camSettings.imageSize);
  cout << "rms = " << rms << endl;
  cout << "camera matrix = \n" << cameraMatrix << endl;
  cout << "dist coeffs = \n" << distCoeffs << endl;

  fromCamToWorld(cameraMatrix, rVecs, tVecs, imagePointsProj, worldPointsProj);

  rms = calibrate(worldPointsProj,
                  PointsInProj,
                  projectorMatrix,
                  projectorDistCoeffs,
                  projectorRVecs,
                  projectorTVecs,
                  projSettings.imageSize);

  cout << "rms = " << rms << endl;
  cout << "projector matrix = \n" << projectorMatrix << endl;
  cout << "projector dist coeffs = \n" << distCoeffs << endl;

  cv::Mat stereoR, stereoT, essential, fundamental;
  cv::Mat RCam, RProj, PCam, PProj, Q;
  rms = cv::stereoCalibrate(worldPointsProj,
                            imagePointsProj,
                            PointsInProj,
                            cameraMatrix,
                            distCoeffs,
                            projectorMatrix,
                            projectorDistCoeffs,
                            camSettings.imageSize,
                            stereoR,
                            stereoT,
                            essential,
                            fundamental);

  cout << "stereo calibrate: \n" << fundamental << endl;

  saveCalibrationResults(outputName, cameraMatrix, distCoeffs, projectorMatrix, projectorDistCoeffs, fundamental);
  return 0;
}

Settings::Settings() {
  patternType = CHESSBOARD;
  patternSize = cv::Size(13, 9);
  subpixelSize = cv::Size(11, 11);
  squareSize = 50;
  nbrOfFrames = 25;
}

void
loadSettings(cv::String path, Settings& sttngs) {
  cv::FileStorage fsInput(path, cv::FileStorage::READ);

  fsInput["PatternWidth"] >> sttngs.patternSize.width;
  fsInput["PatternHeight"] >> sttngs.patternSize.height;
  fsInput["SubPixelWidth"] >> sttngs.subpixelSize.width;
  fsInput["SubPixelHeight"] >> sttngs.subpixelSize.height;
  fsInput["SquareSize"] >> sttngs.squareSize;
  fsInput["NbrOfFrames"] >> sttngs.nbrOfFrames;
  fsInput["PatternType"] >> sttngs.patternType;
  fsInput.release();
}

double
calibrate(vector<vector<cv::Point3f>> objPoints,
          vector<vector<cv::Point2f>> imgPoints,
          cv::Mat& cameraMatrix,
          cv::Mat& distCoeffs,
          vector<cv::Mat>& r,
          vector<cv::Mat>& t,
          cv::Size imgSize) {
  int calibFlags = 0;

  double rms = cv::calibrateCamera(objPoints, imgPoints, imgSize, cameraMatrix, distCoeffs, r, t, calibFlags);

  return rms;
}

void
createObjectPoints(vector<cv::Point3f>& patternCorners, cv::Size patternSize, float squareSize, int patternType) {
  switch(patternType) {
    case CHESSBOARD:
    case CIRCLES_GRID:
      for(int i = 0; i < patternSize.height; ++i) {
        for(int j = 0; j < patternSize.width; ++j) {
          patternCorners.push_back(cv::Point3f(float(i * squareSize), float(j * squareSize), 0));
        }
      }
      break;
    case ASYMETRIC_CIRCLES_GRID: break;
  }
}

void
createProjectorObjectPoints(vector<cv::Point2f>& patternCorners, cv::Size patternSize, float squareSize, int patternType) {
  switch(patternType) {
    case CHESSBOARD:
    case CIRCLES_GRID:
      for(int i = 1; i <= patternSize.height; ++i) {
        for(int j = 1; j <= patternSize.width; ++j) {
          patternCorners.push_back(cv::Point2f(float(j * squareSize), float(i * squareSize)));
        }
      }
      break;
    case ASYMETRIC_CIRCLES_GRID: break;
  }
}

void
fromCamToWorld(cv::Mat cameraMatrix,
               vector<cv::Mat> rV,
               vector<cv::Mat> tV,
               vector<vector<cv::Point2f>> imgPoints,
               vector<vector<cv::Point3f>>& worldPoints) {
  int s = (int)rV.size();
  cv::Mat invK64, invK;
  invK64 = cameraMatrix.inv();
  invK64.convertTo(invK, CV_32F);

  for(int i = 0; i < s; ++i) {
    cv::Mat r, t, rMat;
    rV[i].convertTo(r, CV_32F);
    tV[i].convertTo(t, CV_32F);

    cv::Rodrigues(r, rMat);
    cv::Mat transPlaneToCam = rMat.inv() * t;

    vector<cv::Point3f> wpTemp;
    int s2 = (int)imgPoints[i].size();
    for(int j = 0; j < s2; ++j) {
      cv::Mat coords(3, 1, CV_32F);
      coords.at<float>(0, 0) = imgPoints[i][j].x;
      coords.at<float>(1, 0) = imgPoints[i][j].y;
      coords.at<float>(2, 0) = 1.0f;

      cv::Mat worldPtCam = invK * coords;
      cv::Mat worldPtPlane = rMat.inv() * worldPtCam;

      float scale = transPlaneToCam.at<float>(2) / worldPtPlane.at<float>(2);
      cv::Mat worldPtPlaneReproject = scale * worldPtPlane - transPlaneToCam;

      cv::Point3f pt;
      pt.x = worldPtPlaneReproject.at<float>(0);
      pt.y = worldPtPlaneReproject.at<float>(1);
      pt.z = 0;
      wpTemp.push_back(pt);
    }
    worldPoints.push_back(wpTemp);
  }
}

void
saveCalibrationResults(
    cv::String path, cv::Mat camK, cv::Mat camDistCoeffs, cv::Mat projK, cv::Mat projDistCoeffs, cv::Mat fundamental) {
  cv::FileStorage fs(path + ".yml", cv::FileStorage::WRITE);
  fs << "camIntrinsics" << camK;
  fs << "camDistCoeffs" << camDistCoeffs;
  fs << "projIntrinsics" << projK;
  fs << "projDistCoeffs" << projDistCoeffs;
  fs << "fundamental" << fundamental;
  fs.release();
}

void
saveCalibrationData(cv::String path,
                    vector<cv::Mat> T1,
                    vector<cv::Mat> T2,
                    vector<cv::Mat> ptsProjCam,
                    vector<cv::Mat> ptsProjProj,
                    vector<cv::Mat> ptsProjCamN,
                    vector<cv::Mat> ptsProjProjN) {
  cv::FileStorage fs(path + ".yml", cv::FileStorage::WRITE);

  int size = (int)T1.size();
  fs << "size" << size;
  for(int i = 0; i < (int)T1.size(); ++i) {
    ostringstream nbr;
    nbr << i;
    fs << "TprojCam" + nbr.str() << T1[i];
    fs << "TProjProj" + nbr.str() << T2[i];
    fs << "ptsProjCam" + nbr.str() << ptsProjCam[i];
    fs << "ptsProjProj" + nbr.str() << ptsProjProj[i];
    fs << "ptsProjCamN" + nbr.str() << ptsProjCamN[i];
    fs << "ptsProjProjN" + nbr.str() << ptsProjProjN[i];
  }
  fs.release();
}

void
normalize(const cv::Mat& pts, const int& dim, cv::Mat& normpts, cv::Mat& T) {
  float averagedist = 0;
  float scale = 0;

  // centroid

  cv::Mat centroid(dim, 1, CV_32F);
  cv::Scalar tmp;

  if(normpts.empty()) {
    normpts = cv::Mat(pts.rows, pts.cols, CV_32F);
  }

  for(int i = 0; i < dim; ++i) {
    tmp = cv::mean(pts.row(i));
    centroid.at<float>(i, 0) = (float)tmp[0];
    cv::subtract(pts.row(i), centroid.at<float>(i, 0), normpts.row(i));
  }

  // average distance

  cv::Mat ptstmp;
  for(int i = 0; i < normpts.cols; ++i) {
    ptstmp = normpts.col(i);
    averagedist = averagedist + (float)cv::norm(ptstmp);
  }
  averagedist = averagedist / normpts.cols;
  scale = (float)(sqrt(static_cast<float>(dim)) / averagedist);

  normpts = normpts * scale;

  T = cv::Mat::eye(dim + 1, dim + 1, CV_32F);
  for(int i = 0; i < dim; ++i) {
    T.at<float>(i, i) = scale;
    T.at<float>(i, dim) = -scale * centroid.at<float>(i, 0);
  }
}

void
fromVectorToMat(vector<cv::Point2f> v, cv::Mat& pts) {
  int nbrOfPoints = (int)v.size();

  if(pts.empty())
    pts.create(2, nbrOfPoints, CV_32F);

  for(int i = 0; i < nbrOfPoints; ++i) {
    pts.at<float>(0, i) = v[i].x;
    pts.at<float>(1, i) = v[i].y;
  }
}

void
fromMatToVector(cv::Mat pts, vector<cv::Point2f>& v) {
  int nbrOfPoints = pts.cols;

  for(int i = 0; i < nbrOfPoints; ++i) {
    cv::Point2f temp;
    temp.x = pts.at<float>(0, i);
    temp.y = pts.at<float>(1, i);
    v.push_back(temp);
  }
}
