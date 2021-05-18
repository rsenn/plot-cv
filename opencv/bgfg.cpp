#include <opencv2/bgsegm.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <iostream>

//using namespace cv;
//using namespace cv::bgsegm;

const cv::String about = "\nA program demonstrating the use and capabilities of different background "
                     "subtraction algrorithms\n"
                     "Using OpenCV version " +
                     cv::String(CV_VERSION) + "\nPress q or ESC to exit\n";

const cv::String keys = "{help h usage ? |      | print this message   }"
                    "{vid            |      | path to a video file }"
                    "{algo           | GMG  | name of the algorithm (GMG, CNT, KNN, MOG, MOG2) }";

static cv::Ptr<cv::BackgroundSubtractor>
createBGSubtractorByName(const cv::String& algoName) {
  cv::Ptr<cv::BackgroundSubtractor> algo;
  if(algoName == cv::String("GMG"))
    algo = createBackgroundSubtractorGMG(20, 0.7);
  else if(algoName == cv::String("CNT"))
    algo = createBackgroundSubtractorCNT();
  else if(algoName == cv::String("KNN"))
    algo = cv::createBackgroundSubtractorKNN();
  else if(algoName == cv::String("MOG"))
    algo = createBackgroundSubtractorMOG();
  else if(algoName == cv::String("MOG2"))
    algo = cv::createBackgroundSubtractorMOG2();

  return algo;
}

int
main(int argc, char** argv) {
  cv::CommandLineParser parser(argc, argv, keys);
  parser.about(about);
  parser.printMessage();
  if(parser.has("help")) {
    parser.printMessage();
    return 0;
  }

  cv::String videoPath = parser.get<cv::String>("vid");
  cv::String algoName = parser.get<cv::String>("algo");

  if(!parser.check()) {
    parser.printErrors();
    return 0;
  }

  cv::Ptr<cv::BackgroundSubtractor> bgfs = createBGSubtractorByName(algoName);
  if(!bgfs) {
    std::cerr << "Failed to create " << algoName << " background subtractor" << std::endl;
    return -1;
  }

  cv::VideoCapture cap;
  if(argc > 1)
    cap.open(videoPath);
  else
    cap.open(0);

  if(!cap.isOpened()) {
    std::cerr << "Cannot cv::read video. Try moving video file to sample directory." << std::endl;
    return -1;
  }

  cv::Mat frame, fgmask, segm;

  cv::namedWindow("FG Segmentation", cv::WINDOW_NORMAL);

  for(;;) {
    cap >> frame;

    if(frame.empty())
      break;

    bgfs->apply(frame, fgmask);

    frame.convertTo(segm, CV_8U, 0.5);
    cv::add(frame, cv::Scalar(100, 100, 0), segm, fgmask);

    cv::imshow("FG Segmentation", segm);

    int c = cv::waitKey(30);
    if(c == 'q' || c == 'Q' || (c & 255) == 27)
      break;
  }

  return 0;
}
