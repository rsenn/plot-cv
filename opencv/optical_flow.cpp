#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

//using namespace cv;
using namespace std;

int
main(int argc, char** argv) {
  const string about = "This sample demonstrates Lucas-Kanade Optical Flow calculation.\n"
                       "The example file can be downloaded from:\n"
                       "  "
                       "https://www.bogotobogo.com/python/OpenCV_Python/images/mean_shift_tracking/"
                       "slow_traffic_small.mp4";
  const string keys = "{ h help |      | print this help message }"
                      "{ @image |<none>| path to image file }";
  cv::CommandLineParser parser(argc, argv, keys);
  parser.about(about);
  if(parser.has("help")) {
    parser.printMessage();
    return 0;
  }
  string filename = parser.get<string>("@image");
  if(!parser.check()) {
    parser.printErrors();
    return 0;
  }

  cv::VideoCapture capture(filename);
  if(!capture.isOpened()) {
    // cv::error in opening the video input
    cerr << "Unable to open file!" << endl;
    return 0;
  }

  // Create some random colors
  vector<cv::Scalar> colors;
  RNG rng;
  for(int i = 0; i < 100; i++) {
    int r = rng.uniform(0, 256);
    int g = rng.uniform(0, 256);
    int b = rng.uniform(0, 256);
    colors.push_back(cv::Scalar(r, g, b));
  }

  cv::Mat old_frame, old_gray;
  vector<cv::Point2f> p0, p1;

  // Take first frame and find corners in it
  capture >> old_frame;
  cv::cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
  cv::goodFeaturesToTrack(old_gray, p0, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);

  // Create a mask image for drawing purposes
  cv::Mat mask = cv::Mat::zeros(old_frame.size(), old_frame.type());

  while(true) {
    cv::Mat frame, frame_gray;

    capture >> frame;
    if(frame.empty())
      break;
    cv::cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

    // calculate optical flow
    vector<uchar> status;
    vector<float> err;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
    cv::calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, cv::Size(15, 15), 2, criteria);

    vector<cv::Point2f> good_new;
    for(uint i = 0; i < p0.size(); i++) {
      // Select good points
      if(status[i] == 1) {
        good_new.push_back(p1[i]);
        // draw the tracks
        cv::line(mask, p1[i], p0[i], colors[i], 2);
        cv::circle(frame, p1[i], 5, colors[i], -1);
      }
    }
    cv::Mat img;
    cv::add(frame, mask, img);

    cv::imshow("Frame", img);

    int keyboard = cv::waitKey(30);
    if(keyboard == 'q' || keyboard == 27)
      break;

    // Now update the previous frame and previous points
    old_gray = frame_gray.clone();
    p0 = good_new;
  }
}
