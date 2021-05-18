/*----------------------------------------------
 * Usage:
 * example_tracking_multitracker <video_name> [algorithm]
 *
 * example:
 * example_tracking_multitracker Bolt/img/%04d.jpg
 * example_tracking_multitracker faceocc2.webm KCF
 *--------------------------------------------------*/

#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>
#include <ctime>
#include <samples_utility.hpp>

using namespace std;
//using namespace cv;

int
main(int argc, char** argv) {
  // show help
  if(argc < 2) {
    cout << " Usage: example_tracking_multitracker <video_name> [algorithm]\n"
            " examples:\n"
            " example_tracking_multitracker Bolt/img/%04d.jpg\n"
            " example_tracking_multitracker faceocc2.webm MEDIANFLOW\n"
         << endl;
    return 0;
  }

  // set the default tracking algorithm
  std::string trackingAlg = "KCF";

  // set the tracking algorithm from parameter
  if(argc > 2)
    trackingAlg = argv[2];

  // create the tracker
  //! [create]
  cv::MultiTracker trackers;
  //! [create]

  // container of the tracked objects
  //! [roi]
  vector<cv::Rect2d> objects;
  //! [roi]

  // set input video
  std::string video = argv[1];
  cv::VideoCapture cap(video);

  cv::Mat frame;

  // get bounding box
  cap >> frame;
  //! [selectmulti]
  vector<cv::Rect> ROIs;
  cv::selectROIs("tracker", frame, ROIs);
  //! [selectmulti]

  // quit when the tracked object(s) is not provided
  if(ROIs.size() < 1)
    return 0;

  // initialize the tracker
  //! [init]
  std::vector<cv::Ptr<cv::Tracker>> algorithms;
  for(size_t i = 0; i < ROIs.size(); i++) {
    algorithms.push_back(createTrackerByName(trackingAlg));
    objects.push_back(ROIs[i]);
  }

  trackers.add(algorithms, frame, objects);
  //! [init]

  // do the tracking
  printf("Start the tracking process, press ESC to quit.\n");
  for(;;) {
    // get frame from the video
    cap >> frame;

    // stop the program if no more images
    if(frame.rows == 0 || frame.cols == 0)
      break;

    // update the tracking result
    //! [update]
    trackers.update(frame);
    //! [update]

    //! [result]
    // draw the tracked object
    for(unsigned i = 0; i < trackers.getObjects().size(); i++)
      cv::rectangle(frame, trackers.getObjects()[i], cv::Scalar(255, 0, 0), 2, 1);
    //! [result]

    // show image with the tracked object
    cv::imshow("tracker", frame);

    // quit on ESC button
    if(cv::waitKey(1) == 27)
      break;
  }
}
