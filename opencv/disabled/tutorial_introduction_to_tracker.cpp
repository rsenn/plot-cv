#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>

using namespace std;
// using namespace cv;

int
main(int argc, char** argv) {
  // show help
  //! [help]
  if(argc < 2) {
    cout << " Usage: tracker <video_name>\n"
            " examples:\n"
            " example_tracking_kcf Bolt/img/%04d.jpg\n"
            " example_tracking_kcf faceocc2.webm\n"
         << endl;
    return 0;
  }
  //! [help]

  // declares all required variables
  //! [vars]
  Rect2d roi;
  cv::Mat frame;
  //! [vars]

  // create a tracker object
  //! [create]
  Ptr<cv::Tracker> tracker = cv::TrackerKCF::create();
  //! [create]

  // set input video
  //! [setvideo]
  std::string video = argv[1];
  cv::VideoCapture cap(video);
  //! [setvideo]

  // get bounding box
  //! [getframe]
  cap >> frame;
  //! [getframe]
  //! [selectroi]
  roi = cv::selectROI("tracker", frame);
  //! [selectroi]

  // quit if ROI was not selected
  if(roi.width == 0 || roi.height == 0)
    return 0;

  // initialize the tracker
  //! [init]
  tracker->init(frame, roi);
  //! [init]

  // perform the tracking process
  printf("Start the tracking process, press ESC to quit.\n");
  for(;;) {
    // get frame from the video
    cap >> frame;

    // stop the program if no more images
    if(frame.rows == 0 || frame.cols == 0)
      break;

    // update the tracking result
    //! [update]
    tracker->update(frame, roi);
    //! [update]

    //! [visualization]
    // draw the tracked object
    cv::rectangle(frame, roi, cv::Scalar(255, 0, 0), 2, 1);

    // show image with the tracked object
    cv::imshow("tracker", frame);
    //! [visualization]

    // quit on ESC button
    if(cv::waitKey(1) == 27)
      break;
  }

  return 0;
}
