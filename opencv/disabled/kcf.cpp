/*----------------------------------------------
 * Usage:
 * example_tracking_kcf <video_name>
 *
 * example:
 * example_tracking_kcf Bolt/img/%04.jpg
 * example_tracking_kcf faceocc2.webm
 *--------------------------------------------------*/

#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>
#include "samples_utility.hpp"

using namespace std;
//using namespace cv;

int
main(int argc, char** argv) {
  // show help
  if(argc < 2) {
    cout << " Usage: example_tracking_kcf <video_name>\n"
            " examples:\n"
            " example_tracking_kcf Bolt/img/%04.jpg\n"
            " example_tracking_kcf faceocc2.webm\n"
         << endl;
    return 0;
  }

  // create the tracker
  Ptr<cv::Tracker> tracker = cv::TrackerKCF::create();

  // set input video
  std::string video = argv[1];
  cv::VideoCapture cap(video);

  cv::Mat frame;

  // get bounding box
  cap >> frame;
  Rect2d roi = cv::selectROI("tracker", frame, true, false);

  // quit if ROI was not selected
  if(roi.width == 0 || roi.height == 0)
    return 0;

  // initialize the tracker
  tracker->init(frame, roi);

  // do the tracking
  printf("Start the tracking process, press ESC to quit.\n");
  for(;;) {
    // get frame from the video
    cap >> frame;

    // stop the program if no more images
    if(frame.rows == 0 || frame.cols == 0)
      break;

    // update the tracking result
    bool isfound = tracker->update(frame, roi);
    if(!isfound) {
      cout << "The target has been lost...\n";
      cv::waitKey(0);
      return 0;
    }

    // draw the tracked object
    cv::rectangle(frame, roi, cv::Scalar(255, 0, 0), 2, 1);

    // show image with the tracked object
    cv::imshow("tracker", frame);

    // quit on ESC button
    if(cv::waitKey(1) == 27)
      break;
  }
}
