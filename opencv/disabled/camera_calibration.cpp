#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

// using namespace cv;
using namespace std;

static void
help() {
  cout << "This is a camera calibration sample." << endl
       << "Usage: calibration configurationFile" << endl
       << "Near the sample file you'll find the configuration file, which has detailed help of "
          "how to edit it.  It may be any OpenCV supported file cv::format XML/YAML."
       << endl;
}
class Settings {
public:
  Settings() : goodInput(false) {}
  enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
  enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };

  void
  cv::write(cv::FileStorage& fs) const { // Write serialization for this class
    fs << "{"
       << "BoardSize_Width" << boardSize.width << "BoardSize_Height" << boardSize.height << "Square_Size" << squareSize
       << "Calibrate_Pattern" << patternToUse << "Calibrate_NrOfFrameToUse" << nrFrames << "Calibrate_FixAspectRatio"
       << aspectRatio << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
       << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

       << "Write_DetectedFeaturePoints" << bwritePoints << "Write_extrinsicParameters" << bwriteExtrinsics
       << "Write_outputFileName" << outputFileName

       << "Show_UndistortedImage" << showUndistorsed

       << "Input_FlipAroundHorizontalAxis" << flipVertical << "Input_Delay" << delay << "Input" << input << "}";
  }
  void
  cv::read(const cv::FileNode& node) { // Read serialization for this class
    node["BoardSize_Width"] >> boardSize.width;
    node["BoardSize_Height"] >> boardSize.height;
    node["Calibrate_Pattern"] >> patternToUse;
    node["Square_Size"] >> squareSize;
    node["Calibrate_NrOfFrameToUse"] >> nrFrames;
    node["Calibrate_FixAspectRatio"] >> aspectRatio;
    node["Write_DetectedFeaturePoints"] >> bwritePoints;
    node["Write_extrinsicParameters"] >> bwriteExtrinsics;
    node["Write_outputFileName"] >> outputFileName;
    node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
    node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
    node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
    node["Show_UndistortedImage"] >> showUndistorsed;
    node["Input"] >> input;
    node["Input_Delay"] >> delay;
    interprate();
  }
  void
  interprate() {
    goodInput = true;
    if(boardSize.width <= 0 || boardSize.height <= 0) {
      cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
      goodInput = false;
    }
    if(squareSize <= 10e-6) {
      cerr << "Invalid square size " << squareSize << endl;
      goodInput = false;
    }
    if(nrFrames <= 0) {
      cerr << "Invalid number of frames " << nrFrames << endl;
      goodInput = false;
    }

    if(input.empty()) // Check for valid input
      inputType = INVALID;
    else {
      if(input[0] >= '0' && input[0] <= '9') {
        stringstream ss(input);
        ss >> cameraID;
        inputType = CAMERA;
      } else {
        if(readStringList(input, imageList)) {
          inputType = IMAGE_LIST;
          nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
        } else
          inputType = VIDEO_FILE;
      }
      if(inputType == CAMERA)
        inputCapture.open(cameraID);
      if(inputType == VIDEO_FILE)
        inputCapture.open(input);
      if(inputType != IMAGE_LIST && !inputCapture.isOpened())
        inputType = INVALID;
    }
    if(inputType == INVALID) {
      cerr << " Inexistent input: " << input;
      goodInput = false;
    }

    flag = 0;
    if(calibFixPrincipalPoint)
      flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
    if(calibZeroTangentDist)
      flag |= CV_CALIB_ZERO_TANGENT_DIST;
    if(aspectRatio)
      flag |= CV_CALIB_FIX_ASPECT_RATIO;

    calibrationPattern = NOT_EXISTING;
    if(!patternToUse.compare("CHESSBOARD"))
      calibrationPattern = CHESSBOARD;
    if(!patternToUse.compare("CIRCLES_GRID"))
      calibrationPattern = CIRCLES_GRID;
    if(!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID"))
      calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
    if(calibrationPattern == NOT_EXISTING) {
      cerr << " Inexistent camera calibration mode: " << patternToUse << endl;
      goodInput = false;
    }
    atImageList = 0;
  }
  Mat
  nextImage() {
    cv::Mat result;
    if(inputCapture.isOpened()) {
      cv::Mat view0;
      inputCapture >> view0;
      view0.copyTo(result);
    } else if(atImageList < (int)imageList.size())
      result = cv::imread(imageList[atImageList++], cv::LOAD_IMAGE_COLOR);

    return result;
  }

  static bool
  readStringList(const string& filename, vector<string>& l) {
    l.clear();
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
      return false;
    cv::FileNode n = fs.getFirstTopLevelNode();
    if(n.type() != cv::FileNode::SEQ)
      return false;
    cv::FileNodeIterator it = n.begin(), it_end = n.end();
    for(; it != it_end; ++it) l.push_back((string)*it);
    return true;
  }

public:
  cv::Size boardSize;          // The size of the board -> Number of items by width and height
  Pattern calibrationPattern;  // One of the Chessboard, circles, or asymmetric cv::circle pattern
  float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
  int nrFrames;                // The number of frames to use from the input for calibration
  float aspectRatio;           // The aspect ratio
  int delay;                   // In case of a video input
  bool bwritePoints;           //  Write detected feature points
  bool bwriteExtrinsics;       // Write extrinsic parameters
  bool calibZeroTangentDist;   // Assume zero tangential distortion
  bool calibFixPrincipalPoint; // Fix the principal point at the center
  bool flipVertical;           // Flip the captured images around the horizontal axis
  string outputFileName;       // The name of the file where to write
  bool showUndistorsed;        // Show undistorted images after calibration
  string input;                // The input ->

  int cameraID;
  vector<string> imageList;
  int atImageList;
  cv::VideoCapture inputCapture;
  InputType inputType;
  bool goodInput;
  int flag;

private:
  string patternToUse;
};

static void
cv::read(const cv::FileNode& node, Settings& x, const Settings& default_value = Settings()) {
  if(node.empty())
    x = default_value;
  else
    x.read(node);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(
    Settings& s, cv::Size imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, vector<vector<cv::Point2f>> imagePoints);

int
main(int argc, char* argv[]) {
  help();
  Settings s;
  const string inputSettingsFile = argc > 1 ? argv[1] : "default.xml";
  cv::FileStorage fs(inputSettingsFile, cv::FileStorage::READ); // Read the settings
  if(!fs.isOpened()) {
    cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
    return -1;
  }
  fs["Settings"] >> s;
  fs.release(); // close Settings file

  if(!s.goodInput) {
    cout << "Invalid input detected. Application stopping. " << endl;
    return -1;
  }

  vector<vector<cv::Point2f>> imagePoints;
  cv::Mat cameraMatrix, distCoeffs;
  cv::Size imageSize;
  int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
  clock_t prevTimestamp = 0;
  const cv::Scalar RED(0, 0, 255), GREEN(0, 255, 0);
  const char ESC_KEY = 27;

  for(int i = 0;; ++i) {
    cv::Mat view;
    bool blinkOutput = false;

    view = s.nextImage();

    //-----  If no more image, or got enough, then stop calibration and show result -------------
    if(mode == CAPTURING && imagePoints.size() >= (unsigned)s.nrFrames) {
      if(runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints))
        mode = CALIBRATED;
      else
        mode = DETECTION;
    }
    if(view.empty()) { // If no more images then run calibration, save and stop loop.
      if(imagePoints.size() > 0)
        runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints);
      break;
    }

    imageSize = view.size(); // Format input image.
    if(s.flipVertical)
      cv::flip(view, view, 0);

    vector<cv::Point2f> pointBuf;

    bool found;
    switch(s.calibrationPattern) { // Find feature points on the input format
      case Settings::CHESSBOARD:
        found = cv::findChessboardCorners(view,
                                          s.boardSize,
                                          pointBuf,
                                          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
        break;
      case Settings::CIRCLES_GRID: found = cv::findCirclesGrid(view, s.boardSize, pointBuf); break;
      case Settings::ASYMMETRIC_CIRCLES_GRID:
        found = cv::findCirclesGrid(view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID);
        break;
      default: found = false; break;
    }

    if(found) { // If done with success,
      // improve the found corners' coordinate accuracy for chessboard
      if(s.calibrationPattern == Settings::CHESSBOARD) {
        cv::Mat viewGray;
        cv::cvtColor(view, viewGray, COLOR_BGR2GRAY);
        cv::cornerSubPix(
            viewGray, pointBuf, cv::Size(11, 11), cv::Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
      }

      if(mode == CAPTURING && // For camera only take new cv::samples after delay time
         (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay * 1e-3 * CLOCKS_PER_SEC)) {
        imagePoints.push_back(pointBuf);
        prevTimestamp = clock();
        blinkOutput = s.inputCapture.isOpened();
      }

      // Draw the corners.
      cv::drawChessboardCorners(view, s.boardSize, cv::Mat(pointBuf), found);
    }

    //----------------------------- Output Text ------------------------------------------------
    string msg = (mode == CAPTURING) ? "100/100" : mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
    int baseLine = 0;
    cv::Size textSize = cv::getTextSize(msg, 1, 1, 1, &baseLine);
    cv::Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);

    if(mode == CAPTURING) {
      if(s.showUndistorsed)
        msg = cv::format("%d/%d Undist", (int)imagePoints.size(), s.nrFrames);
      else
        msg = cv::format("%d/%d", (int)imagePoints.size(), s.nrFrames);
    }

    cv::putText(view, msg, textOrigin, 1, 1, mode == CALIBRATED ? GREEN : RED);

    if(blinkOutput)
      cv::bitwise_not(view, view);

    //------------------------- Video capture  output  undistorted ------------------------------
    if(mode == CALIBRATED && s.showUndistorsed) {
      cv::Mat temp = view.clone();
      cv::undistort(temp, view, cameraMatrix, distCoeffs);
    }

    //------------------------------ Show image and check for input commands -------------------
    cv::imshow("Image View", view);
    char key = (char)cv::waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

    if(key == ESC_KEY)
      break;

    if(key == 'u' && mode == CALIBRATED)
      s.showUndistorsed = !s.showUndistorsed;

    if(s.inputCapture.isOpened() && key == 'g') {
      mode = CAPTURING;
      imagePoints.clear();
    }
  }

  // -----------------------Show the undistorted image for the image list ------------------------
  if(s.inputType == Settings::IMAGE_LIST && s.showUndistorsed) {
    cv::Mat view, rview, map1, map2;
    cv::initUndistortRectifyMap(cameraMatrix,
                                distCoeffs,
                                cv::Mat(),
                                cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                                imageSize,
                                CV_16SC2,
                                map1,
                                map2);

    for(int i = 0; i < (int)s.imageList.size(); i++) {
      view = cv::imread(s.imageList[i], 1);
      if(view.empty())
        continue;
      cv::remap(view, rview, map1, map2, INTER_LINEAR);
      cv::imshow("Image View", rview);
      char c = (char)cv::waitKey();
      if(c == ESC_KEY || c == 'q' || c == 'Q')
        break;
    }
  }

  return 0;
}

static double
computeReprojectionErrors(const vector<vector<Point3f>>& objectPoints,
                          const vector<vector<cv::Point2f>>& imagePoints,
                          const vector<cv::Mat>& rvecs,
                          const vector<cv::Mat>& tvecs,
                          const cv::Mat& cameraMatrix,
                          const cv::Mat& distCoeffs,
                          vector<float>& perViewErrors) {
  vector<cv::Point2f> imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  perViewErrors.resize(objectPoints.size());

  for(i = 0; i < (int)objectPoints.size(); ++i) {
    cv::projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
    err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), CV_L2);

    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float)std::sqrt(err * err / n);
    totalErr += err * err;
    totalPoints += n;
  }

  return std::sqrt(totalErr / totalPoints);
}

static void
calcBoardCornerPositions(cv::Size boardSize,
                         float squareSize,
                         vector<Point3f>& corners,
                         Settings::Pattern patternType /*= Settings::CHESSBOARD*/) {
  corners.clear();

  switch(patternType) {
    case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
      for(int i = 0; i < boardSize.height; ++i)
        for(int j = 0; j < boardSize.width; ++j) corners.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
      break;

    case Settings::ASYMMETRIC_CIRCLES_GRID:
      for(int i = 0; i < boardSize.height; i++)
        for(int j = 0; j < boardSize.width; j++)
          corners.push_back(Point3f(float((2 * j + i % 2) * squareSize), float(i * squareSize), 0));
      break;
    default: break;
  }
}

static bool
runCalibration(Settings& s,
               cv::Size& imageSize,
               cv::Mat& cameraMatrix,
               cv::Mat& distCoeffs,
               vector<vector<cv::Point2f>> imagePoints,
               vector<cv::Mat>& rvecs,
               vector<cv::Mat>& tvecs,
               vector<float>& reprojErrs,
               double& totalAvgErr) {

  cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  if(s.flag & CV_CALIB_FIX_ASPECT_RATIO)
    cameraMatrix.at<double>(0, 0) = 1.0;

  distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

  vector<vector<Point3f>> objectPoints(1);
  calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

  objectPoints.resize(imagePoints.size(), objectPoints[0]);

  // Find intrinsic and extrinsic camera parameters
  double rms = cv::calibrateCamera(
      objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, s.flag | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

  cout << "Re-projection cv::error reported by cv::calibrateCamera: " << rms << endl;

  bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);

  totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

  return ok;
}

// Print camera parameters to the output file
static void
saveCameraParams(Settings& s,
                 cv::Size& imageSize,
                 cv::Mat& cameraMatrix,
                 cv::Mat& distCoeffs,
                 const vector<cv::Mat>& rvecs,
                 const vector<cv::Mat>& tvecs,
                 const vector<float>& reprojErrs,
                 const vector<vector<cv::Point2f>>& imagePoints,
                 double totalAvgErr) {
  cv::FileStorage fs(s.outputFileName, cv::FileStorage::WRITE);

  time_t tm;
  time(&tm);
  struct tm* t2 = localtime(&tm);
  char buf[1024];
  strftime(buf, sizeof(buf) - 1, "%c", t2);

  fs << "calibration_Time" << buf;

  if(!rvecs.empty() || !reprojErrs.empty())
    fs << "nrOfFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
  fs << "image_Width" << imageSize.width;
  fs << "image_Height" << imageSize.height;
  fs << "board_Width" << s.boardSize.width;
  fs << "board_Height" << s.boardSize.height;
  fs << "square_Size" << s.squareSize;

  if(s.flag & CV_CALIB_FIX_ASPECT_RATIO)
    fs << "FixAspectRatio" << s.aspectRatio;

  if(s.flag) {
    sprintf(buf,
            "flags: %s%s%s%s",
            s.flag & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
            s.flag & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
            s.flag & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
            s.flag & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "");
    cvWriteComment(*fs, buf, 0);
  }

  fs << "flagValue" << s.flag;

  fs << "Camera_Matrix" << cameraMatrix;
  fs << "Distortion_Coefficients" << distCoeffs;

  fs << "Avg_Reprojection_Error" << totalAvgErr;
  if(!reprojErrs.empty())
    fs << "Per_View_Reprojection_Errors" << cv::Mat(reprojErrs);

  if(!rvecs.empty() && !tvecs.empty()) {
    CV_Assert(rvecs[0].type() == tvecs[0].type());
    cv::Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
    for(int i = 0; i < (int)rvecs.size(); i++) {
      cv::Mat r = bigmat(Range(i, i + 1), Range(0, 3));
      cv::Mat t = bigmat(Range(i, i + 1), Range(3, 6));

      CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
      CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
      //*.t() is MatExpr (not cv::Mat) so we can use assignment operator
      r = rvecs[i].t();
      t = tvecs[i].t();
    }
    cvWriteComment(*fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0);
    fs << "Extrinsic_Parameters" << bigmat;
  }

  if(!imagePoints.empty()) {
    cv::Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
    for(int i = 0; i < (int)imagePoints.size(); i++) {
      cv::Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
      cv::Mat imgpti(imagePoints[i]);
      imgpti.copyTo(r);
    }
    fs << "Image_points" << imagePtMat;
  }
}

bool
runCalibrationAndSave(
    Settings& s, cv::Size imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, vector<vector<cv::Point2f>> imagePoints) {
  vector<cv::Mat> rvecs, tvecs;
  vector<float> reprojErrs;
  double totalAvgErr = 0;

  bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs, totalAvgErr);
  cout << (ok ? "Calibration succeeded" : "Calibration failed") << ". avg re projection cv::error = " << totalAvgErr;

  if(ok)
    saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints, totalAvgErr);
  return ok;
}
