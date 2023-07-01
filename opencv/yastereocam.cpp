#include <yastereocam.h>
#include <QDebug>
#include <QImage>
#include <QCoreApplication>
#include <QProcess>

#include <opencv2/opencv.hpp>

YaStereoCam::YaStereoCam(QObject* parent) : QObject(parent) {
  qInfo() << __PRETTY_FUNCTION__;
  _capL = new cv::VideoCapture(0);

  _capL->set(cv::CAP_PROP_FRAME_WIDTH, 640);
  _capL->set(cv::CAP_PROP_FRAME_HEIGHT, 480);

  if(!_capL->isOpened()) {
    qErrnoWarning("CV Camera/Left capture cv::error");
    QCoreApplication::exit(-1);
  };
#ifdef DEBUG_PC
  qInfo() << "use single cam as a right source too";
#else  // DEBUG_PC
  _capR = new cv::VideoCapture(1);
  _capR->set(cv::CAP_PROP_FRAME_WIDTH, 640);
  _capR->set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  if(!_capR->isOpened()) {
    qErrnoWarning("CV Camera/Right capture cv::error");
    QCoreApplication::exit(-1);
  }
#endif // DEBUG_PC
  getCamProps();
  setCamProps();
  _imgInL = new cv::Mat(640, 480, CV_8UC3);
  _imgInR = new cv::Mat(640, 480, CV_8UC3);
  count = 0;
}

YaStereoCam::~YaStereoCam() {
  qInfo() << __PRETTY_FUNCTION__;
  qInfo() << "cv::total # of captured frames" << count;
}

void
YaStereoCam::getCamProps() {
  _camLWidth = _capL->get(cv::CAP_PROP_FRAME_WIDTH);
  _camLHeight = _capL->get(cv::CAP_PROP_FRAME_HEIGHT);
  _camLFps = _capL->get(cv::CAP_PROP_FPS);
#ifdef DEBUG_PC
  _camRFps = _camLFps;
  _camRWidth = _camLWidth;
  _camRHeight = _camLHeight;
#else  // DEBUG_PC
  _camRWidth = _capR->get(cv::CAP_PROP_FRAME_WIDTH);
  _camRHeight = _capR->get(cv::CAP_PROP_FRAME_HEIGHT);
  _camRFps = _capR->get(cv::CAP_PROP_FPS);
#endif // DEBUG_PC
  qInfo() << " camL" << _camLWidth << "x" << _camLHeight << "@" << _camLFps << "\n"
          << "camR" << _camRWidth << "x" << _camRHeight << "@" << _camRFps;
}

void
YaStereoCam::setCamProps() {
  qInfo() << __PRETTY_FUNCTION__;
  // https://www.raspberrypi.org/forums/viewtopic.php?t=62364
  QProcess pr;
  int res = QProcess::execute("v4l2-ctl", QStringList() << "--list-formats");
  switch(res) {
    case -1: {
      qWarning() << "process crashed";
      break;
    }
    case -2: {
      qWarning() << "process cannot started";
      break;
    }
    default: {
      qInfo() << "process OK";
      break;
    }
  }
}

void
YaStereoCam::capImages() {
  //    qInfo() << __PRETTY_FUNCTION__;
  count++;
  qInfo() << "cv::cap frame #" << count;
  *_capL >> *_imgInL;
  if(_imgInL->empty()) {
    qWarning() << "\tImage/Left is empty #" << count;
    QCoreApplication::exit(-1);
  }

#ifdef DEBUG_PC
  *_imgInR = *_imgInL;
#else
  *_capR >> *_imgInR;
  if(_imgInR->empty()) {
    qWarning() << "\tImage/Right is empty #" << count;
    QCoreApplication::exit(-2);
  }
#endif

  // cv::imwrite("outCvImgInL.jpg",*_imgInL); //test write
  emit imageLReady();
  emit imageRReady();
}

void
YaStereoCam::getImageL(QImage& cv::img, bool swapRnB) {
  //    qInfo() << __PRETTY_FUNCTION__;
  if(swapRnB) {
    cv::cvtColor(*_imgInL, *_imgInL, cv::COLOR_BGR2RGB);
  }
  QImage qimg(_imgInL->ptr(), _imgInL->cols, _imgInL->rows, _imgInL->step, QImage::Format_RGB888);
  cv::img = qimg;
}

void
YaStereoCam::getImageR(QImage& cv::img, bool swapRnB) {
//    qInfo() << __PRETTY_FUNCTION__;
#ifndef DEBUG_PC
  if(swapRnB) {
    cv::cvtColor(*_imgInR, *_imgInR, cv::COLOR_BGR2RGB);
  }
#endif // DEBUG_PC
  QImage qimg(_imgInR->ptr(), _imgInR->cols, _imgInR->rows, _imgInR->step, QImage::Format_RGB888);
  cv::img = qimg;
}
void
YaStereoCam::getImageL(cv::Mat& cv::img) {
  //    qInfo() << __PRETTY_FUNCTION__;
  cv::img = *_imgInL;
}
void
YaStereoCam::getImageR(cv::Mat& cv::img) {
  //    qInfo() << __PRETTY_FUNCTION__;
  cv::img = *_imgInR;
}
