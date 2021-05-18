#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core_c.h>

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>

//using namespace cv;
using namespace std;

const char* usage = " \nexample command cv::line for calibration from a live feed.\n"
                    "   calibration  -w 4 -h 5 -s 0.025 -o camera.yml -op -oe\n"
                    " \n"
                    " example command cv::line for calibration from a list of stored images:\n"
                    "   imagelist_creator image_list.xml *.png\n"
                    "   calibration -w 4 -h 5 -s 0.025 -o camera.yml -op -oe image_list.xml\n"
                    " where image_list.xml is the standard OpenCV XML/YAML\n"
                    " use imagelist_creator to create the xml or yaml list\n"
                    " file consisting of the list of strings, e.g.:\n"
                    " \n"
                    "<?xml version=\"1.0\"?>\n"
                    "<opencv_storage>\n"
                    "<images>\n"
                    "view000.png\n"
                    "view001.png\n"
                    "<!-- view002.png -->\n"
                    "view003.png\n"
                    "view010.png\n"
                    "one_extra_view.jpg\n"
                    "</images>\n"
                    "</opencv_storage>\n";

const char* liveCaptureHelp = "When the live video from camera is used as input, the following hot-keys may be used:\n"
                              "  <ESC>, 'q' - quit the program\n"
                              "  'g' - start capturing images\n"
                              "  'u' - switch undistortion on/off\n";

static void
help() {
  printf("This is a camera calibration sample.\n"
         "Usage: calibration\n"
         "     -w <board_width>         # the number of inner corners per one of board dimension\n"
         "     -h <board_height>        # the number of inner corners per another board dimension\n"
         "     [-pt <pattern>]          # the type of pattern: chessboard or circles' grid\n"
         "     [-n <number_of_frames>]  # the number of frames to use for calibration\n"
         "                              # (if not specified, it will be set to the number\n"
         "                              #  of board views actually available)\n"
         "     [-d <delay>]             # a minimum delay in ms between subsequent attempts to "
         "capture a next view\n"
         "                              # (used only for video capturing)\n"
         "     [-s <squareSize>]       # square size in some user-defined units (1 by default)\n"
         "     [-o <out_camera_params>] # the output filename for intrinsic [and extrinsic] "
         "parameters\n"
         "     [-op]                    # cv::write detected feature points\n"
         "     [-oe]                    # cv::write extrinsic parameters\n"
         "     [-zt]                    # assume zero tangential distortion\n"
         "     [-a <aspectRatio>]      # fix aspect ratio (fx/fy)\n"
         "     [-p]                     # fix the principal point at the center\n"
         "     [-v]                     # cv::flip the captured images around the horizontal axis\n"
         "     [-V]                     # use a video file, and not an image list, uses\n"
         "                              # [input_data] string for the video file name\n"
         "     [-su]                    # show undistorted images after calibration\n"
         "     [input_data]             # input data, one of the following:\n"
         "                              #  - text file with a list of the images of the board\n"
         "                              #    the text file can be generated with imagelist_creator\n"
         "                              #  - name of video file with a video of the board\n"
         "                              # if input_data not specified, a live view from the camera is "
         "used\n"
         "\n");
  printf("\n%s", usage);
  printf("\n%s", liveCaptureHelp);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

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
  perViewErrors.cv::resize(objectPoints.size());

  for(i = 0; i < (int)objectPoints.size(); i++) {
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
calcChessboardCorners(cv::Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD) {
  corners.cv::resize(0);

  switch(patternType) {
    case CHESSBOARD:
    case CIRCLES_GRID:
      for(int i = 0; i < boardSize.height; i++)
        for(int j = 0; j < boardSize.width; j++) corners.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
      break;

    case ASYMMETRIC_CIRCLES_GRID:
      for(int i = 0; i < boardSize.height; i++)
        for(int j = 0; j < boardSize.width; j++)
          corners.push_back(Point3f(float((2 * j + i % 2) * squareSize), float(i * squareSize), 0));
      break;

    default: CV_Error(CV_StsBadArg, "Unknown pattern type\n");
  }
}

static bool
runCalibration(vector<vector<cv::Point2f>> imagePoints,
               cv::Size imageSize,
               cv::Size boardSize,
               Pattern patternType,
               float squareSize,
               float aspectRatio,
               int flags,
               cv::Mat& cameraMatrix,
               cv::Mat& distCoeffs,
               vector<cv::Mat>& rvecs,
               vector<cv::Mat>& tvecs,
               vector<float>& reprojErrs,
               double& totalAvgErr) {
  cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  if(flags & CV_CALIB_FIX_ASPECT_RATIO)
    cameraMatrix.at<double>(0, 0) = aspectRatio;

  distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

  vector<vector<Point3f>> objectPoints(1);
  calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);

  objectPoints.cv::resize(imagePoints.size(), objectPoints[0]);

  double rms = cv::calibrateCamera(
      objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flags | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);
  ///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
  printf("RMS cv::error reported by cv::calibrateCamera: %g\n", rms);

  bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);

  totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

  return ok;
}

static void
saveCameraParams(const string& filename,
                 cv::Size imageSize,
                 cv::Size boardSize,
                 float squareSize,
                 float aspectRatio,
                 int flags,
                 const cv::Mat& cameraMatrix,
                 const cv::Mat& distCoeffs,
                 const vector<cv::Mat>& rvecs,
                 const vector<cv::Mat>& tvecs,
                 const vector<float>& reprojErrs,
                 const vector<vector<cv::Point2f>>& imagePoints,
                 double totalAvgErr) {
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);

  time_t tt;
  time(&tt);
  struct tm* t2 = localtime(&tt);
  char buf[1024];
  strftime(buf, sizeof(buf) - 1, "%c", t2);

  fs << "calibration_time" << buf;

  if(!rvecs.empty() || !reprojErrs.empty())
    fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
  fs << "image_width" << imageSize.width;
  fs << "image_height" << imageSize.height;
  fs << "board_width" << boardSize.width;
  fs << "board_height" << boardSize.height;
  fs << "square_size" << squareSize;

  if(flags & CV_CALIB_FIX_ASPECT_RATIO)
    fs << "aspectRatio" << aspectRatio;

  if(flags != 0) {
    sprintf(buf,
            "flags: %s%s%s%s",
            flags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
            flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
            flags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
            flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    // cvWriteComment(*fs, buf, 0);
  }

  fs << "flags" << flags;

  fs << "camera_matrix" << cameraMatrix;
  fs << "distortion_coefficients" << distCoeffs;

  fs << "avg_reprojection_error" << totalAvgErr;
  if(!reprojErrs.empty())
    fs << "per_view_reprojection_errors" << cv::Mat(reprojErrs);

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
    // cvWriteComment(*fs, "a set of 6-tuples (rotation vector + translation vector) for each view",
    // 0);
    fs << "extrinsic_parameters" << bigmat;
  }

  if(!imagePoints.empty()) {
    cv::Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
    for(int i = 0; i < (int)imagePoints.size(); i++) {
      cv::Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
      cv::Mat imgpti(imagePoints[i]);
      imgpti.copyTo(r);
    }
    fs << "image_points" << imagePtMat;
  }
}

static bool
readStringList(const string& filename, vector<string>& l) {
  l.cv::resize(0);
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

static bool
runAndSave(const string& outputFilename,
           const vector<vector<cv::Point2f>>& imagePoints,
           cv::Size imageSize,
           cv::Size boardSize,
           Pattern patternType,
           float squareSize,
           float aspectRatio,
           int flags,
           cv::Mat& cameraMatrix,
           cv::Mat& distCoeffs,
           bool writeExtrinsics,
           bool writePoints) {
  vector<cv::Mat> rvecs, tvecs;
  vector<float> reprojErrs;
  double totalAvgErr = 0;

  bool ok = runCalibration(imagePoints,
                           imageSize,
                           boardSize,
                           patternType,
                           squareSize,
                           aspectRatio,
                           flags,
                           cameraMatrix,
                           distCoeffs,
                           rvecs,
                           tvecs,
                           reprojErrs,
                           totalAvgErr);
  printf("%s. avg reprojection cv::error = %.2f\n", ok ? "Calibration succeeded" : "Calibration failed", totalAvgErr);

  if(ok)
    saveCameraParams(outputFilename,
                     imageSize,
                     boardSize,
                     squareSize,
                     aspectRatio,
                     flags,
                     cameraMatrix,
                     distCoeffs,
                     writeExtrinsics ? rvecs : vector<cv::Mat>(),
                     writeExtrinsics ? tvecs : vector<cv::Mat>(),
                     writeExtrinsics ? reprojErrs : vector<float>(),
                     writePoints ? imagePoints : vector<vector<cv::Point2f>>(),
                     totalAvgErr);
  return ok;
}

int
main(int argc, char** argv) {
  cv::Size boardSize, imageSize;
  float squareSize = 1.f, aspectRatio = 1.f;
  cv::Mat cameraMatrix, distCoeffs;
  const char* outputFilename = "out_camera_data.yml";
  const char* inputFilename = 0;

  int i, nframes = 10;
  bool writeExtrinsics = false, writePoints = false;
  bool undistortImage = false;
  int flags = 0;
  cv::VideoCapture capture;
  bool flipVertical = false;
  bool showUndistorted = false;
  bool videofile = false;
  int delay = 1000;
  clock_t prevTimestamp = 0;
  int mode = DETECTION;
  int cameraId = 0;
  vector<vector<cv::Point2f>> imagePoints;
  vector<string> imageList;
  Pattern pattern = CHESSBOARD;

  if(argc < 2) {
    help();
    return 0;
  }

  for(i = 1; i < argc; i++) {
    const char* s = argv[i];
    if(strcmp(s, "-w") == 0) {
      if(sscanf(argv[++i], "%u", &boardSize.width) != 1 || boardSize.width <= 0)
        return fprintf(stderr, "Invalid board width\n"), -1;
    } else if(strcmp(s, "-h") == 0) {
      if(sscanf(argv[++i], "%u", &boardSize.height) != 1 || boardSize.height <= 0)
        return fprintf(stderr, "Invalid board height\n"), -1;
    } else if(strcmp(s, "-pt") == 0) {
      i++;
      if(!strcmp(argv[i], "circles"))
        pattern = CIRCLES_GRID;
      else if(!strcmp(argv[i], "acircles"))
        pattern = ASYMMETRIC_CIRCLES_GRID;
      else if(!strcmp(argv[i], "chessboard"))
        pattern = CHESSBOARD;
      else
        return fprintf(stderr, "Invalid pattern type: must be chessboard or circles\n"), -1;
    } else if(strcmp(s, "-s") == 0) {
      if(sscanf(argv[++i], "%f", &squareSize) != 1 || squareSize <= 0)
        return fprintf(stderr, "Invalid board square width\n"), -1;
    } else if(strcmp(s, "-n") == 0) {
      if(sscanf(argv[++i], "%u", &nframes) != 1 || nframes <= 3)
        return printf("Invalid number of images\n"), -1;
    } else if(strcmp(s, "-a") == 0) {
      if(sscanf(argv[++i], "%f", &aspectRatio) != 1 || aspectRatio <= 0)
        return printf("Invalid aspect ratio\n"), -1;
      flags |= CV_CALIB_FIX_ASPECT_RATIO;
    } else if(strcmp(s, "-d") == 0) {
      if(sscanf(argv[++i], "%u", &delay) != 1 || delay <= 0)
        return printf("Invalid delay\n"), -1;
    } else if(strcmp(s, "-op") == 0) {
      writePoints = true;
    } else if(strcmp(s, "-oe") == 0) {
      writeExtrinsics = true;
    } else if(strcmp(s, "-zt") == 0) {
      flags |= CV_CALIB_ZERO_TANGENT_DIST;
    } else if(strcmp(s, "-p") == 0) {
      flags |= CV_CALIB_FIX_PRINCIPAL_POINT;
    } else if(strcmp(s, "-v") == 0) {
      flipVertical = true;
    } else if(strcmp(s, "-V") == 0) {
      videofile = true;
    } else if(strcmp(s, "-o") == 0) {
      outputFilename = argv[++i];
    } else if(strcmp(s, "-su") == 0) {
      showUndistorted = true;
    } else if(s[0] != '-') {
      if(isdigit(s[0]))
        sscanf(s, "%d", &cameraId);
      else
        inputFilename = s;
    } else
      return fprintf(stderr, "Unknown option %s", s), -1;
  }

  if(inputFilename) {
    if(!videofile && readStringList(inputFilename, imageList))
      mode = CAPTURING;
    else
      capture.open(inputFilename);
  } else
    capture.open(cameraId);

  if(!capture.isOpened() && imageList.empty())
    return fprintf(stderr, "Could not initialize video (%d) capture\n", cameraId), -2;

  if(!imageList.empty())
    nframes = (int)imageList.size();

  if(capture.isOpened())
    printf("%s", liveCaptureHelp);

  cv::namedWindow("Image View", 1);

  for(i = 0;; i++) {
    cv::Mat view, viewGray;
    bool blink = false;

    if(capture.isOpened()) {
      cv::Mat view0;
      capture >> view0;
      view0.copyTo(view);
    } else if(i < (int)imageList.size())
      view = cv::imread(imageList[i], 1);

    if(!view.data) {
      if(imagePoints.size() > 0)
        runAndSave(outputFilename,
                   imagePoints,
                   imageSize,
                   boardSize,
                   pattern,
                   squareSize,
                   aspectRatio,
                   flags,
                   cameraMatrix,
                   distCoeffs,
                   writeExtrinsics,
                   writePoints);
      break;
    }

    imageSize = view.size();

    if(flipVertical)
      cv::flip(view, view, 0);

    vector<cv::Point2f> pointbuf;
    cv::cvtColor(view, viewGray, COLOR_BGR2GRAY);

    bool found;
    switch(pattern) {
      case CHESSBOARD:
        found = cv::findChessboardCorners(view,
                                      boardSize,
                                      pointbuf,
                                      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
        break;
      case CIRCLES_GRID: found = cv::findCirclesGrid(view, boardSize, pointbuf); break;
      case ASYMMETRIC_CIRCLES_GRID: found = cv::findCirclesGrid(view, boardSize, pointbuf, CALIB_CB_ASYMMETRIC_GRID); break;
      default: return fprintf(stderr, "Unknown pattern type\n"), -1;
    }

    // improve the found corners' coordinate accuracy
    if(pattern == CHESSBOARD && found)
      cv::cornerSubPix(viewGray, pointbuf, cv::Size(11, 11), cv::Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    if(mode == CAPTURING && found && (!capture.isOpened() || clock() - prevTimestamp > delay * 1e-3 * CLOCKS_PER_SEC)) {
      imagePoints.push_back(pointbuf);
      prevTimestamp = clock();
      blink = capture.isOpened();
    }

    if(found)
      cv::drawChessboardCorners(view, boardSize, cv::Mat(pointbuf), found);

    string msg = mode == CAPTURING ? "100/100" : mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
    int baseLine = 0;
    cv::Size textSize = cv::getTextSize(msg, 1, 1, 1, &baseLine);
    cv::Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);

    if(mode == CAPTURING) {
      if(undistortImage)
        msg = cv::format("%d/%d Undist", (int)imagePoints.size(), nframes);
      else
        msg = cv::format("%d/%d", (int)imagePoints.size(), nframes);
    }

    cv::putText(view, msg, textOrigin, 1, 1, mode != CALIBRATED ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0));

    if(blink)
      cv::bitwise_not(view, view);

    if(mode == CALIBRATED && undistortImage) {
      cv::Mat temp = view.clone();
      cv::undistort(temp, view, cameraMatrix, distCoeffs);
    }

    cv::imshow("Image View", view);
    int key = 0xff & cv::waitKey(capture.isOpened() ? 50 : 500);

    if((key & 255) == 27)
      break;

    if(key == 'u' && mode == CALIBRATED)
      undistortImage = !undistortImage;

    if(capture.isOpened() && key == 'g') {
      mode = CAPTURING;
      imagePoints.clear();
    }

    if(mode == CAPTURING && imagePoints.size() >= (unsigned)nframes) {
      if(runAndSave(outputFilename,
                    imagePoints,
                    imageSize,
                    boardSize,
                    pattern,
                    squareSize,
                    aspectRatio,
                    flags,
                    cameraMatrix,
                    distCoeffs,
                    writeExtrinsics,
                    writePoints))
        mode = CALIBRATED;
      else
        mode = DETECTION;
      if(!capture.isOpened())
        break;
    }
  }

  if(!capture.isOpened() && showUndistorted) {
    cv::Mat view, rview, map1, map2;
    cv::initUndistortRectifyMap(cameraMatrix,
                            distCoeffs,
                            cv::Mat(),
                            cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                            imageSize,
                            CV_16SC2,
                            map1,
                            map2);

    for(i = 0; i < (int)imageList.size(); i++) {
      view = cv::imread(imageList[i], 1);
      if(!view.data)
        continue;
      // cv::undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
      cv::remap(view, rview, map1, map2, INTER_LINEAR);
      cv::imshow("Image View", rview);
      int c = 0xff & cv::waitKey();
      if((c & 255) == 27 || c == 'q' || c == 'Q')
        break;
    }
  }

  return 0;
}
