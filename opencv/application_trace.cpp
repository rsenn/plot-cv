/* OpenCV Application Tracing support demo. */
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utils/trace.hpp>

//using namespace cv;
using namespace std;

static void
process_frame(const cv::UMat& frame) {
  CV_TRACE_FUNCTION(); // OpenCV Trace macro for function

  cv::imshow("Live", frame);

  cv::UMat gray, processed;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  cv::Canny(gray, processed, 32, 64, 3);
  cv::imshow("Processed", processed);
}

int
main(int argc, char** argv) {
  CV_TRACE_FUNCTION();

  cv::CommandLineParser parser(argc,
                               argv,
                               "{help h ? |     | help message}"
                               "{n        | 100 | number of frames to process }"
                               "{@video   | 0   | video filename or cameraID }");
  if(parser.has("help")) {
    parser.printMessage();
    return 0;
  }

  cv::VideoCapture capture;
  std::string video = parser.get<string>("@video");
  if(video.size() == 1 && isdigit(video[0]))
    capture.open(parser.get<int>("@video"));
  else
    capture.open(cv::samples::findFileOrKeep(video)); // keep GStreamer pipelines
  int nframes = 0;
  if(capture.isOpened()) {
    nframes = (int)capture.get(cv::CAP_PROP_FRAME_COUNT);
    cout << "Video " << video << ": width=" << capture.get(cv::CAP_PROP_FRAME_WIDTH)
         << ", height=" << capture.get(cv::CAP_PROP_FRAME_HEIGHT) << ", nframes=" << nframes << endl;
  } else {
    cout << "Could not initialize video capturing...\n";
    return -1;
  }

  int N = parser.get<int>("n");
  if(nframes > 0 && N > nframes)
    N = nframes;

  cout << "Start processing..." << endl << "Press ESC key to terminate" << endl;

  cv::UMat frame;
  for(int i = 0; N > 0 ? (i < N) : true; i++) {
    CV_TRACE_REGION("FRAME"); // OpenCV Trace macro for named "scope" region
    {
      CV_TRACE_REGION("cv::read");
      capture.read(frame);

      if(frame.empty()) {
        cerr << "Can't capture frame: " << i << std::endl;
        break;
      }

      // OpenCV Trace macro for NEXT named region in the same C++ scope
      // Previous "cv::read" region will be marked complete on this cv::line.
      // Use this to eliminate unnecessary curly braces.
      CV_TRACE_REGION_NEXT("process");
      process_frame(frame);

      CV_TRACE_REGION_NEXT("delay");
      if(cv::waitKey(1) == 27 /*ESC*/)
        break;
    }
  }

  return 0;
}
