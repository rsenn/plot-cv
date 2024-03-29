#include "pibilight.h"
//#include "yaml-cpp/yaml.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <libyuv.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>

using namespace std;
// using namespace cv;

string logFilePrefix = "/home/pi/pibilight/cv::log/";
string logFileName;
bool logCreated = false;
int cameraFD;
int outputFD;
int width = 720;
int height = 480;
int outWidth = 640;
int outHeight = 360;
size_t bufferSize = outWidth * outHeight * 2;

#define CONFIG_FILE "/home/pi/pibilight/config.yml"

string loopbackDevice = "/dev/video0";

unsigned char* outBuffer;

cv::VideoCapture cap;

cv::Mat currentImage;
cv::Mat processedImage;
cv::Mat undistortedTemp;
cv::Mat bgraImg;
cv::Mat yuyvImg;

cv::Mat cameraMatrix, distortionCoefficients;

cv::Point2f sourcePoints[4];
cv::Point2f destinationPoints[4];

cv::Mat perspTransform;

bool
createLogFile() {
  time_t currentTime = time(nullptr);
  char timeStr[100];
  strftime(timeStr, sizeof(timeStr), "%Y-%m-%d_%H-%M", localtime(&currentTime));
  logFileName = timeStr;
  logFileName.append(".cv::log");

  ofstream outfile(logFilePrefix + logFileName);
  outfile << "Log created on " << string(timeStr) << endl << flush;
  outfile.close();

  return true;
}

void
logLine(string cv::line) {
  if(!logCreated) {
    logCreated = createLogFile();
  }

  ofstream file;
  file.open(logFilePrefix + logFileName, ios_base::app);
  file << cv::line << endl;
  file.close();

  cout << cv::line << endl;
}

void
loadConfig() {
  cv::FileStorage configFile(CONFIG_FILE, cv::FileStorage::READ);

  if(!configFile["width"].empty()) {
    width = (int)configFile["width"];
  }

  if(!configFile["height"].empty()) {
    height = (int)configFile["height"];
  }

  if(!configFile["outWidth"].empty()) {
    outWidth = (int)configFile["outWidth"];
  }

  if(!configFile["outHeight"].empty()) {
    outHeight = (int)configFile["outHeight"];
  }

  bufferSize = outWidth * outHeight * 2;

  destinationPoints[0] = cv::Point2f{0, 0};
  destinationPoints[1] = cv::Point2f{(float)outWidth - 1, 0};
  destinationPoints[2] = cv::Point2f{0, (float)outHeight - 1};
  destinationPoints[3] = cv::Point2f{(float)outWidth - 1, (float)outHeight - 1};

  configFile["camera_matrix"] >> cameraMatrix;
  configFile["distortion_coefficients"] >> distortionCoefficients;

  cv::Mat tempSourcePoints;
  configFile["corner_points"] >> tempSourcePoints;
  sourcePoints[0] = cv::Point2f{tempSourcePoints.at<float>(0, 0), tempSourcePoints.at<float>(0, 1)};
  sourcePoints[1] = cv::Point2f{tempSourcePoints.at<float>(1, 0), tempSourcePoints.at<float>(1, 1)};
  sourcePoints[2] = cv::Point2f{tempSourcePoints.at<float>(2, 0), tempSourcePoints.at<float>(2, 1)};
  sourcePoints[3] = cv::Point2f{tempSourcePoints.at<float>(3, 0), tempSourcePoints.at<float>(3, 1)};

  // cout << "0" << sourcePoints[0] << endl;
  // cout << "1" << sourcePoints[1] << endl;
  // cout << "2" << sourcePoints[2] << endl;
  // cout << "3" << sourcePoints[3] << endl;

  perspTransform = cv::getPerspectiveTransform(sourcePoints, destinationPoints);

  // cout << "Dist:" << distortionCoefficients << endl;
  // cout << "CMat:" << cameraMatrix << endl;
  // cout << "Points:" << tempSourcePoints << endl;
}

void
openCamera() {
  int deviceID = 1; // 0 = open default camera
  int apiID = 0;    // 0 = autodetect default API
  // open selected camera using selected API
  cap.open(deviceID + apiID);

  // check if we succeeded
  if(!cap.isOpened()) {
    throw("ERROR! Unable to open camera");
    return;
  }
}

void
captureImage() {
  // wait for a new frame from camera and store it into 'frame'
  cap.read(currentImage);
  // check if we succeeded
  if(currentImage.empty()) {
    throw("ERROR! blank frame grabbed");
  }

  if(processedImage.empty()) {
    currentImage.copyTo(processedImage);
  }

  return;
}

void
initOutput() {
  outputFD = open(loopbackDevice.c_str(), O_WRONLY);
  if(outputFD == -1) {
    throw("cv::Error Opening Output device");
    return;
  }

  struct v4l2_capability vid_caps = {0};

  if(-1 == ioctl(outputFD, VIDIOC_QUERYCAP, &vid_caps)) {
    throw("Query capture capabilites");
    return;
  }

  struct v4l2_format fmt = {0};
  fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

  if(-1 == ioctl(outputFD, VIDIOC_G_FMT, &fmt)) {
    throw(std::string("Getting Pixel Format Output: ").append(strerror(errno)));
    return;
  }

  fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  fmt.fmt.pix.width = outWidth;
  fmt.fmt.pix.height = outHeight;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.sizeimage = bufferSize;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  fmt.fmt.pix.bytesperline = outWidth * 2;
  fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;

  if(-1 == ioctl(outputFD, VIDIOC_S_FMT, &fmt)) {
    throw("Setting Pixel Format Output");
    return;
  }

  outBuffer = (unsigned char*)malloc(sizeof(unsigned char) * bufferSize);
  memset(outBuffer, 0, bufferSize);
}

void
outputImage() {
  // Convert to YUYV
  cv::cvtColor(processedImage, bgraImg, CV_BGR2BGRA);

  if(yuyvImg.empty()) {
    yuyvImg.create(bgraImg.rows, bgraImg.cols, CV_8UC2);
  }

  int res = libyuv::ARGBToYUY2(bgraImg.data, bgraImg.cols * 4, yuyvImg.data, outWidth * 2, bgraImg.cols, bgraImg.rows);

  if(cv::write(outputFD, yuyvImg.ptr(), bufferSize) == -1) {
    throw("Write to output");
    return;
  }
}

void
processImage() {
  // Undistort the image by distortion coefficients
  // if(!cameraMatrix.empty())
  // {
  //   cv::undistort(currentImage, undistortedTemp, cameraMatrix, distortionCoefficients);
  // }
  // else
  // {
  // currentImage.copyTo(undistortedTemp);
  // }

  // Perspective Transformation
  if(!perspTransform.empty()) {
    // cout << perspTransform << endl;
    // cv::warpPerspective(undistortedTemp, processedImage, perspTransform, cv::Size{width, height});
    cv::warpPerspective(currentImage, processedImage, perspTransform, cv::Size{outWidth, outHeight});
  } else {
    processedImage = undistortedTemp;
  }
}

int
main(int argc, char** argv) {
  // cout << "OpenCV version: " << CV_VERSION << endl;
  try {
    // Load config file
    loadConfig();

    // Open V4L2-capture device
    openCamera();

    // Open output V4L2-Device
    initOutput();

    // Perform the operation
    while(true) {
      captureImage();
      // cv::imwrite("image.png", currentImage);

      processImage();
      // cv::imwrite("processed.png", processedImage);

      // currentImage.copyTo(processedImage);
      outputImage();

      // sleep(3000);
      sleep(50);
    }
  } catch(std::exception const& e) { logLine(e.what()); } catch(std::string& e) {
    logLine(e);
  } catch(const char* e) { logLine(std::string(e)); } catch(...) {
    logLine("Exception occurred");
  }
  return 0;
}
