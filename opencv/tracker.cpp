#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>
#include <samples_utility.hpp>

using namespace std;
// using namespace cv;

static const char* keys = {"{@tracker_algorithm | | cv::Tracker algorithm }"
                           "{@video_name      | | video name        }"
                           "{@start_frame     |0| Start frame       }"
                           "{@bounding_frame  |0,0,0,0| Initial bounding frame}"};

static void
help() {
  cout << "\nThis example shows the functionality of \"Long-term optical tracking API\""
          "-- pause video [p] and draw a bounding box around the target to start the tracker\n"
          "Example of <video_name> is in opencv_extra/testdata/cv/tracking/\n"
          "Call:\n"
          "./tracker <tracker_algorithm> <video_name> <start_frame> [<bounding_frame>]\n"
          "tracker_algorithm can be: MIL, BOOSTING, MEDIANFLOW, TLD, KCF, GOTURN, MOSSE.\n"
       << endl;

  cout << "\n\nHot keys: \n"
          "\tq - quit the program\n"
          "\tp - pause video\n";
}

int
main(int argc, char** argv) {
  cv::CommandLineParser parser(argc, argv, keys);

  cv::String tracker_algorithm = parser.get<cv::String>(0);
  cv::String video_name = parser.get<cv::String>(1);
  int start_frame = parser.get<int>(2);

  if(tracker_algorithm.empty() || video_name.empty()) {
    help();
    return -1;
  }

  int coords[4] = {0, 0, 0, 0};
  bool initBoxWasGivenInCommandLine = false;
  {
    cv::String initBoundingBox = parser.get<cv::String>(3);
    for(size_t npos = 0, pos = 0, ctr = 0; ctr < 4; ctr++) {
      npos = initBoundingBox.find_first_of(',', pos);
      if(npos == string::npos && ctr < 3) {
        printf("bounding box should be given in cv::format \"x1,y1,x2,y2\",where x's and y's are "
               "integer coordinates of "
               "opposed corners of bdd box\n");
        printf("got: %s\n", initBoundingBox.substr(pos, string::npos).c_str());
        printf("manual selection of bounding box will be employed\n");
        break;
      }
      int num = atoi(initBoundingBox.substr(pos, (ctr == 3) ? (string::npos) : (npos - pos)).c_str());
      if(num <= 0) {
        printf("bounding box should be given in cv::format \"x1,y1,x2,y2\",where x's and y's are "
               "integer coordinates of "
               "opposed corners of bdd box\n");
        printf("got: %s\n", initBoundingBox.substr(pos, npos - pos).c_str());
        printf("manual selection of bounding box will be employed\n");
        break;
      }
      coords[ctr] = num;
      pos = npos + 1;
    }
    if(coords[0] > 0 && coords[1] > 0 && coords[2] > 0 && coords[3] > 0) {
      initBoxWasGivenInCommandLine = true;
    }
  }

  // open the capture
  cv::VideoCapture cap;
  cap.open(video_name);
  cap.set(cv::CAP_PROP_POS_FRAMES, start_frame);

  if(!cap.isOpened()) {
    help();
    cout << "***Could not initialize capturing...***\n";
    cout << "Current parameter's value: \n";
    parser.printMessage();
    return -1;
  }

  cv::Mat frame;
  cv::namedWindow("Tracking API", 1);

  cv::Mat image;
  cv::Rect2d boundingBox;
  bool paused = false;

  // instantiates the specific Tracker
  cv::Ptr<cv::Tracker> tracker = createTrackerByName(tracker_algorithm);
  if(!tracker) {
    cout << "***cv::Error in the instantiation of the tracker...***\n";
    return -1;
  }

  // get the first frame
  cap >> frame;
  frame.copyTo(image);
  if(initBoxWasGivenInCommandLine) {
    boundingBox.x = coords[0];
    boundingBox.y = coords[1];
    boundingBox.width = std::abs(coords[2] - coords[0]);
    boundingBox.height = std::abs(coords[3] - coords[1]);
    printf("bounding box with vertices (%d,%d) and (%d,%d) was given in command cv::line\n",
           coords[0],
           coords[1],
           coords[2],
           coords[3]);
    cv::rectangle(image, boundingBox, cv::Scalar(255, 0, 0), 2, 1);
  } else
    boundingBox = cv::selectROI("Tracking API", image);

  cv::imshow("Tracking API", image);

  bool initialized = false;
  int frameCounter = 0;
  int64 timeTotal = 0;

  for(;;) {
    if(!paused) {
      if(initialized) {
        cap >> frame;
        if(frame.empty()) {
          break;
        }
        frame.copyTo(image);
      }

      if(!initialized) {
        // initializes the tracker
        if(!tracker->init(frame, boundingBox)) {
          cout << "***Could not initialize tracker...***\n";
          return -1;
        }
        initialized = true;
      } else if(initialized) {
        int64 frameTime = cv::getTickCount();
        // updates the tracker
        if(tracker->update(frame, boundingBox)) {
          cv::rectangle(image, boundingBox, cv::Scalar(255, 0, 0), 2, 1);
        }
        frameTime = cv::getTickCount() - frameTime;
        timeTotal += frameTime;
      }
      cv::imshow("Tracking API", image);
      frameCounter++;
    }

    char c = (char)cv::waitKey(2);
    if(c == 'q')
      break;
    if(c == 'p')
      paused = !paused;
  }

  double s = frameCounter / (timeTotal / cv::getTickFrequency());
  printf("FPS: %f\n", s);

  return 0;
}
