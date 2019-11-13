#include "analysis_algorithms/ObjectCouting.h"
#include "backgroundsub_algorithms/PBAS/PixelBasedAdaptiveSegmenter.h"
#include "tracking_algorithms/BlobTracking.h"
#include "communication_api/serverapi.h"
#include <iostream>
#include <memory>
#include <opencv2/bgsegm.hpp>
#include <QCoreApplication>
using cv::BackgroundSubtractor;
using cv::namedWindow;
using cv::Ptr;
using cv::VideoCapture;
using cv::bgsegm::BackgroundSubtractorMOG;
using cv::bgsegm::createBackgroundSubtractorMOG;
using std::cerr;
using std::make_unique;
using std::unique_ptr;
int
main(int argc, char* argv[]) {
  QCoreApplication app(argc, argv);
  for(int i{}; i < argc; i++) cout << argv[i] << std::endl;
  cout << "OpenCV version: " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << "." << CV_SUBMINOR_VERSION << std::endl;
  cout << "Qt version: " << QT_VERSION_STR << endl;

  //
  if(argc < 4) {
    cout << "Input Error" << endl;
    return 1;
  }
  /* Open video file */
  VideoCapture systemCapture;
  QString captureSource{argv[1]};
  if(captureSource == "video")
    systemCapture.open(argv[2]);
  else if(captureSource == "camera") {
    systemCapture.open(QString(argv[2]).toInt());
    systemCapture.set(cv::CAP_PROP_FRAME_WIDTH, 640.0);
    systemCapture.set(cv::CAP_PROP_FRAME_HEIGHT, 480.0);
  }
  if(!systemCapture.isOpened()) {
    cerr << "Cannot open video or camera ";
    return 1;
  }
  /* Background Subtraction Algorithm */
  // unique_ptr<IBGS> bgs{ make_unique<PixelBasedAdaptiveSegmenter>() };
  /* Blob Tracking Algorithm */
  cv::Mat img_blob;
  unique_ptr<BlobTracking> blobTracking{make_unique<BlobTracking>()};

  /* Vehicle Counting Algorithm */
  unique_ptr<ObjectCouting> objectCouting{make_unique<ObjectCouting>(QString(argv[3]))};

  std::cout << "Press 'q' to quit..." << std::endl;

  Ptr<BackgroundSubtractorMOG> backgroundSuctractor{createBackgroundSubtractorMOG(200, 5, 0.7, 15)};
  Mat img_input{};
  // namedWindow("BGS", cv::WINDOW_NORMAL);
  int captureFPS{static_cast<int>(systemCapture.get(cv::CAP_PROP_FPS))};
  while(1) {
    systemCapture >> img_input;
    if(img_input.empty())
      break;

    // cv::imshow("Input", img_input);
    // bgs->process(...) internally process and show the foreground mask image
    // bgs->process(img_input, img_mask);
    cv::Mat img_output;
    backgroundSuctractor->apply(img_input, img_output);
    // cv::imshow("BGS", img_output);
    if(!img_output.empty()) {
      // Perform blob tracking
      blobTracking->process(img_input, img_output, img_blob);

      // Perform  counting
      objectCouting->setInput(img_blob);
      objectCouting->setTracks(blobTracking->getTracks());
      objectCouting->process();
    }
    // Press q to quit the program
    if(cv::waitKey(captureFPS) == 'q')
      break;
  }
  cvDestroyAllWindows();
  // cvReleaseCapture(&capture);
}
