#include <cameraimages.h>
#include <opencv2/opencv.hpp>

CameraImages::CameraImages(std::string aPath) {
  mImagePath = new std::string(aPath);
  mImageCapture = NULL;
}

CameraImages::~CameraImages() {
  if(mImageCapture != NULL) {
    delete mImageCapture;
  }
  delete mImagePath;
}

cv::VideoCapture*
CameraImages::GetCamera() {
  return mImageCapture;
}

void
CameraImages::Initialize() {
  mImageCapture = new cv::VideoCapture(*mImagePath);
}
