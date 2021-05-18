#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <ctype.h>

//using namespace cv;
using namespace std;

static void
help() {
  // print a welcome message, and the OpenCV version
  cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
          "Using OpenCV version "
       << CV_VERSION << endl;
  cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
  cout << "\nHot keys: \n"
          "\tESC - quit the program\n"
          "\tr - auto-initialize tracking\n"
          "\tc - delete all the points\n"
          "\tn - switch the \"night\" mode on/off\n"
          "To cv::add/remove a feature point click it\n"
       << endl;
}

cv::Point2f point;
bool addRemovePt = false;

static void
onMouse(int event, int x, int y, int /*flags*/, void* /*param*/) {
  if(event == EVENT_LBUTTONDOWN) {
    point = cv::Point2f((float)x, (float)y);
    addRemovePt = true;
  }
}

int
main(int argc, char** argv) {
  cv::VideoCapture cap;
  TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
  cv::Size subPixWinSize(10, 10), winSize(31, 31);

  const int MAX_COUNT = 500;
  bool needToInit = false;
  bool nightMode = false;

  help();
  cv::CommandLineParser parser(argc, argv, "{@input|0|}");
  string input = parser.get<string>("@input");

  if(input.size() == 1 && isdigit(input[0]))
    cap.open(input[0] - '0');
  else
    cap.open(input);

  if(!cap.isOpened()) {
    cout << "Could not initialize capturing...\n";
    return 0;
  }

  cv::namedWindow("LK Demo", 1);
  cv::setMouseCallback("LK Demo", onMouse, 0);

  cv::Mat gray, prevGray, image, frame;
  vector<cv::Point2f> points[2];

  for(;;) {
    cap >> frame;
    if(frame.empty())
      break;

    frame.copyTo(image);
    cv::cvtColor(image, gray, COLOR_BGR2GRAY);

    if(nightMode)
      image = cv::Scalar::all(0);

    if(needToInit) {
      // automatic initialization
      cv::goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
      cv::cornerSubPix(gray, points[1], subPixWinSize, cv::Size(-1, -1), termcrit);
      addRemovePt = false;
    } else if(!points[0].empty()) {
      vector<uchar> status;
      vector<float> err;
      if(prevGray.empty())
        gray.copyTo(prevGray);
      cv::calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize, 3, termcrit, 0, 0.001);
      size_t i, k;
      for(i = k = 0; i < points[1].size(); i++) {
        if(addRemovePt) {
          if(cv::norm(point - points[1][i]) <= 5) {
            addRemovePt = false;
            continue;
          }
        }

        if(!status[i])
          continue;

        points[1][k++] = points[1][i];
        cv::circle(image, points[1][i], 3, cv::Scalar(0, 255, 0), -1, 8);
      }
      points[1].cv::resize(k);
    }

    if(addRemovePt && points[1].size() < (size_t)MAX_COUNT) {
      vector<cv::Point2f> tmp;
      tmp.push_back(point);
      cv::cornerSubPix(gray, tmp, winSize, cv::Size(-1, -1), termcrit);
      points[1].push_back(tmp[0]);
      addRemovePt = false;
    }

    needToInit = false;
    cv::imshow("LK Demo", image);

    char c = (char)cv::waitKey(10);
    if(c == 27)
      break;
    switch(c) {
      case 'r': needToInit = true; break;
      case 'c':
        points[0].clear();
        points[1].clear();
        break;
      case 'n': nightMode = !nightMode; break;
    }

    std::cv::swap(points[1], points[0]);
    cv::swap(prevGray, gray);
  }

  return 0;
}
