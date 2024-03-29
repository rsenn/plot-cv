#if defined(__linux__) || defined(LINUX) || defined(__APPLE__) || defined(ANDROID) || (defined(_MSC_VER) && _MSC_VER >= 1800)

#include <opencv2/imgproc.hpp> // Gaussian Blur
#include <opencv2/core.hpp>    // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp> // OpenCV window I/O
#include <opencv2/features2d.hpp>
#include <opencv2/objdetect.hpp>

#include <stdio.h>
#include <string>
#include <vector>

using namespace std;
// using namespace cv;

const string WindowName = "Face Detection example";

class CascadeDetectorAdapter : public cv::DetectionBasedTracker::IDetector {
public:
  CascadeDetectorAdapter(cv::Ptr<cv::CascadeClassifier> detector) : IDetector(), Detector(detector) { CV_Assert(detector); }

  void
  detect(const cv::Mat& Image, std::vector<cv::Rect>& objects) {
    Detector->detectMultiScale(Image, objects, scaleFactor, minNeighbours, 0, minObjSize, maxObjSize);
  }

  virtual ~CascadeDetectorAdapter() {}

private:
  CascadeDetectorAdapter();
  cv::Ptr<cv::CascadeClassifier> Detector;
};

int
main(int, char**) {
  cv::namedWindow(WindowName);

  cv::VideoCapture VideoStream(0);

  if(!VideoStream.isOpened()) {
    printf("cv::Error: Cannot open video stream from camera\n");
    return 1;
  }

  std::string cascadeFrontalfilename = "../../data/lbpcascades/lbpcascade_frontalface.xml";
  cv::Ptr<cv::CascadeClassifier> cascade = cv::makePtr<cv::CascadeClassifier>(cascadeFrontalfilename);
  cv::Ptr<cv::DetectionBasedTracker::IDetector> MainDetector = cv::makePtr<CascadeDetectorAdapter>(cascade);
  if(cascade->empty()) {
    printf("cv::Error: Cannot load %s\n", cascadeFrontalfilename.c_str());
    return 2;
  }

  cascade = cv::makePtr<cv::CascadeClassifier>(cascadeFrontalfilename);
  cv::Ptr<cv::DetectionBasedTracker::IDetector> TrackingDetector = cv::makePtr<CascadeDetectorAdapter>(cascade);
  if(cascade->empty()) {
    printf("cv::Error: Cannot load %s\n", cascadeFrontalfilename.c_str());
    return 2;
  }

  cv::DetectionBasedTracker::Parameters params;
  cv::DetectionBasedTracker Detector(MainDetector, TrackingDetector, params);

  if(!Detector.run()) {
    printf("cv::Error: Detector initialization failed\n");
    return 2;
  }

  cv::Mat ReferenceFrame;
  cv::Mat GrayFrame;
  vector<cv::Rect> Faces;

  do {
    VideoStream >> ReferenceFrame;
    cv::cvtColor(ReferenceFrame, GrayFrame, cv::COLOR_RGB2GRAY);
    Detector.process(GrayFrame);
    Detector.getObjects(Faces);

    for(size_t i = 0; i < Faces.size(); i++) { cv::rectangle(ReferenceFrame, Faces[i], cv::Scalar(0, 255, 0)); }

    cv::imshow(WindowName, ReferenceFrame);
  } while(cv::waitKey(30) < 0);

  Detector.stop();

  return 0;
}

#else

#include <stdio.h>
int
main() {
  printf("This sample works for UNIX or ANDROID or Visual Studio 2013+ only\n");
  return 0;
}

#endif
