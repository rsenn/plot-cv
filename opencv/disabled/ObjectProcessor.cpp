// ObjectProcessor.cpp : Defines the entry point for the console application.
// Written by Faris Parkar
// Program runs with OPENCV 3.3.0 library!!!  not tested for versions greater or lower
// Targeted for Windows platform

// PURSUIT OF COLOR SILHOUETTE is a C++ program that allows user to play around with options that
// include object tracking and identification,  contouring, all with their own windows computer
// webcam

// including necessary libraries to access different algorithms in order to create our own code
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utils/trace.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv/cv.h>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ctype.h>
#include <cstdlib>
#include <vector>
#include <math.h>

// using namespace cv;
// using namespace cv::dnn; // using opencv namespace and cv::dnn module namespace to code the
// identification portion
using namespace std;

// declaring variables for track and identify and trackbars
cv::Mat image; // Declaring the matrix image
// Declaring of variables used for the tracker
bool backprojMode = false;
bool objectSelection = false;
int inputWindowSize = 0;
bool showHist = true;
cv::Point origin;
cv::Rect boundingbox;
int vmin = 0, vmax = 256, smin = 10, smax = 256, hueMin = 0, hueMax = 256;

const string trackbarWindow = "Trackbars";

void
is_trackbar(int, void*) {
  // whenever trackbar position is changed, call this function
}
void
createTrackbars() {
  // create window for trackbars

  cv::namedWindow(trackbarWindow, 0);
  // create memory to store trackbar name on window
  char TrackbarName[100];
  /*sprintf(TrackbarName, "S_MIN", smin);
  sprintf(TrackbarName, "V_MIN", vmin);
  sprintf(TrackbarName, "V_MAX", vmax);*/

  // create trackbars and insert them into window
  cv::createTrackbar("S_MIN", trackbarWindow, &smin, 256, is_trackbar);
  cv::createTrackbar("V_MIN", trackbarWindow, &vmin, 256, is_trackbar);
  cv::createTrackbar("V_MAX", trackbarWindow, &vmin, 256, is_trackbar);
}
void
createTrackbarsBG() {

  cv::namedWindow(trackbarWindow, 0);
  // create memory to store trackbar name on window
  char TrackbarName[50];
  /*sprintf_s(TrackbarName, "H_MIN", 0);
  sprintf_s(TrackbarName, "H_MAX", 256);
  sprintf_s(TrackbarName, "S_MIN", smin);
  sprintf_s(TrackbarName, "S_MAX", 256); // cv::threshold values during program options
  sprintf_s(TrackbarName, "V_MIN", vmin);
  sprintf_s(TrackbarName, "V_MAX", vmax);*/
  // create trackbars and insert them into window

  cv::createTrackbar("H_MIN", trackbarWindow, &hueMin, hueMax, is_trackbar);
  cv::createTrackbar("H_MAX", trackbarWindow, &hueMax, hueMax, is_trackbar);
  cv::createTrackbar("S_MIN", trackbarWindow, &smin, smax, is_trackbar);
  cv::createTrackbar("S_MAX", trackbarWindow, &smax, smax, is_trackbar);
  cv::createTrackbar("V_MIN", trackbarWindow, &vmin, vmax, is_trackbar);
  cv::createTrackbar("V_MAX", trackbarWindow, &vmax, vmax, is_trackbar);
}
// User draws box around object to track. This triggers CAMShift to start tracking
void
userMouse(int click, int x, int y, int, void*) {
  if(objectSelection) {
    boundingbox.x = MIN(x, origin.x);
    boundingbox.y = MIN(y, origin.y);
    boundingbox.width = abs(x - origin.x);
    boundingbox.height = abs(y - origin.y);

    boundingbox &= cv::Rect(0, 0, image.cols, image.rows); // mouse function learned and modified from OpenCV book
  }

  switch(click) {
    case EVENT_LBUTTONDOWN:
      origin = cv::Point(x, y);
      boundingbox = cv::Rect(x, y, 0, 0);
      objectSelection = true;
      break;
    case EVENT_LBUTTONUP:
      objectSelection = false;
      if(boundingbox.width > 0 && boundingbox.height > 0)
        inputWindowSize = -1; // Set up CAMShift properties in track_and_identify() loop
      break;
  }
}

void centerstring(char* a); // title centering

void
centerstring(char* a) {
  int l = strlen(a);
  int position = int((80 - 1) / 2); // Writes OBJECTPROCESSOR in the center
  for(int i = 0; i < position; i++) { cout << " "; }
  cout << a;
}

/* Find best class for the blob (i. e. class with maximal probability) */
static void
getMaxClass(const cv::Mat& probBlob, int* classId, double* classProb) {
  cv::Mat probcv::Mat = probBlob.reshape(1, 1); // reshape the blob to 1x1000 matrix
  cv::Point classNumber;
  cv::minMaxLoc(probcv::Mat, NULL, classProb, NULL, &classNumber);
  *classId = classNumber.x;
}

int track_and_identify(int argc, char** argv);
int bg_sub_contour(int argc, char** argv);
void contour_figure(int, void*); // prototype to contour function
// void drawObject();

// readClassNames() is from cv::dnn module in OpenCV library
static std::vector<cv::String>
readClassNames(const char* filename = "synset_words.txt") {
  std::vector<cv::String> classNames;
  std::ifstream fp(filename);
  if(!fp.is_open()) {
    std::cerr << "File with classes labels not found: " << filename << std::endl;
    exit(-1);
  }
  std::string name;
  while(!fp.eof()) {
    std::getline(fp, name);
    if(name.length())
      classNames.push_back(name.substr(name.find(' ') + 1));
  }
  fp.close();
  return classNames;
}

// Convert to string
#define SSTR(x) static_cast<std::ostringstream&>((std::ostringstream() << std::dec << x)).str()

// cv::Tracker is based on extracting colours from a selected objected in an image
// updating its values on a histogram
int
track_and_identify(int argc, char** argv) {
  // instructions
  cout << "You will see three screen appear:" << endl;
  cout << "1.The Object cv::Tracker with your current display" << endl;
  cout << "2.The Trackbars where you can adjust the image saturation and lightness of color once "
          "selected"
       << endl;
  cout << "3.The histrogram that appears black until an image is selected" << endl << endl;
  cout << "First, adjust your object then pause your screen by pressing 'p'." << endl << endl;
  cout << "Select your object by click and dragging to make a cv::rectangle" << endl << endl;
  cout << "Once, the object begins tracking, user can adjust saturation and lightness of color on "
          "Trackbars"
       << endl;
  cout << "Select image if necessary" << endl;
  cout << "To exit tracking, click on stream feed and hit 'ESC'" << endl << endl;
  cout << "Caffe Model begins to identify object" << endl;

  cv::VideoCapture cap(0); // Camera capture
  cv::Rect trackingBox;    // Delcaing the rectangular box of type
  int hue_size = 32;       // Hue quantizes to 32 levels to get pixel data for histogram
  // Hue varies from 0 to 170
  float hranges[] = {0, 180};
  const float* phranges = hranges; // pixel range is only based on hue range

  if(!cap.isOpened()) { // cv::Error for unsuccessful opening of the camera

    cerr << "Cannot open Camera" << endl;

    return -1;
  }
  createTrackbars();
  cv::namedWindow("Histogram", 0);          // Histrogram box disaply name
  cv::namedWindow("Object cv::Tracker", 0); // Object tracker box display name
  cv::setMouseCallback("Object cv::Tracker", userMouse,
                       0); // User can draw inside of the Object tracker box display

  cv::Mat frame, hsv, hue, mask, hist, histimg = cv::Mat::zeros(400, 640, CV_8UC3),
                                       backproj; // setting up the matrix holding the colors and frames of histrogram image
  bool pause = false;

  for(;;) {
    if(!pause) {
      cap >> frame;
      if(frame.empty())
        break;
    }

    frame.copyTo(image);

    if(!pause) {
      cv::cvtColor(image, hsv, COLOR_BGR2HSV); // Determining the color range of the selected image

      if(inputWindowSize) {
        int _vmin = vmin, _vmax = vmax;

        cv::inRange(hsv,
                    cv::Scalar(0, smin, MIN(_vmin, _vmax)),
                    cv::Scalar(180, 256, MAX(_vmin, _vmax)),
                    mask); // takes range of trackbar values in order to adjust noise
        int ch[] = {0, 0};
        hue.create(hsv.size(), hsv.depth());
        cv::mixChannels(&hsv, 1, &hue, 1, ch, 1);

        if(inputWindowSize < 0) {
          // Object has been selected by user, set up CAMShift search properties once
          cv::Mat roi(hue, boundingbox), maskroi(mask, boundingbox); // creating a matrix that will hold the bounding box
          cv::calcHist(&roi, 1, 0, maskroi, hist, 1, &hue_size, &phranges);
          cv::normalize(hist, hist, 0, 255, NORM_MINMAX);
          cv::Mat image_save = image(boundingbox).clone(); // savind the image of the bounding box from the roi
          cv::imwrite("save.jpg", image_save);             // saves boundingbox as an image
          //-------------------------
          //---------------------
          trackingBox = boundingbox;
          inputWindowSize = 1; // Don't set up again, unless user selects new ROI
          // Setting up the bars of the histrogram once user has selected cv::rectangle around object
          // Detecting possible color scheme from that object
          histimg = cv::Scalar::all(0);
          int binW = histimg.cols / hue_size;
          cv::Mat buf(1, hue_size, CV_8UC3);
          for(int i = 0; i < hue_size; i++) buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i * 180. / hue_size), 255, 255);
          cv::cvtColor(buf, buf, COLOR_HSV2BGR);

          for(int i = 0; i < hue_size; i++) {
            int val = saturate_cast<int>(hist.at<float>(i) * histimg.rows / 255); // value of the histogram
            cv::rectangle(histimg,
                          cv::Point(i * binW,
                                    histimg.rows), // value from the cv::rectangle of selected object to
                                                   // convert into the
                          // colored histogram
                          cv::Point((i + 1) * binW, histimg.rows - val),
                          cv::Scalar(buf.at<Vec3b>(i)),
                          -1,
                          8);
          }
        }

        // Perform CAMShift algorithm
        cv::calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
        backproj &= mask;
        cv::RotatedRect trackBox = cv::CamShift(backproj,
                                                trackingBox,
                                                TermCriteria(TermCriteria::EPS | TermCriteria::COUNT,
                                                             0,
                                                             1)); // using opencv's function criteria for tracking window
        if(trackingBox.area() <= 1) {
          int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
          trackingBox =
              cv::Rect(trackingBox.x - r, trackingBox.y - r, trackingBox.x + r, trackingBox.y + r) & cv::Rect(0, 0, cols, rows);
        }

        if(backprojMode)
          cv::cvtColor(backproj, image, cv::COLOR_GRAY2BGR);
        cv::ellipse(image, trackBox, cv::Scalar(255, 0, 0), 12, FILLED); // changing cv::format of ellipse
      }
    } else if(inputWindowSize < 0)
      pause = false;

    if(objectSelection && boundingbox.width > 0 && boundingbox.height > 0) {
      cv::Mat roi(image, boundingbox);
      cv::bitwise_not(roi, roi);
    }
    // cv::Tracker and histogram display
    cv::imshow("Object cv::Tracker", image);
    cv::imshow("Histogram", histimg);

    char q = (char)cv::waitKey(30);
    if(q == 27)
      break;
    switch(q) {
      case 'p': // creating a still frame for user interaction
        pause = !pause;
        break;
      default:;
    }
  }

  // next few lines to access caffe model files and output errors are referenced from cv::dnn module of
  // OpenCV library rest of the code is created from scratch by learning OpenCV 3
  CV_TRACE_FUNCTION();                                      // openCV function
  cv::String modelTxt = "bvlc_googlenet.prototxt";          // Accesing the caffe model files
  cv::String modelBin = "bvlc_googlenet.caffemodel";        // Accesing the caffe model files
  cv::String imageFile = (argc > 1) ? argv[1] : "save.jpg"; // calls boundingbox image instead of entire camera frame
  Net net = cv::dnn::readNetFromCaffe(modelTxt, modelBin);
  if(net.empty()) { // if reading from caffe model is not successful
    std::cerr << "Can't load network by using the following files: " << std::endl;
    std::cerr << "prototxt:   " << modelTxt << std::endl;
    std::cerr << "caffemodel: " << modelBin << std::endl;
    std::cerr << "bvlc_googlenet.caffemodel can be downloaded here:" << std::endl;
    std::cerr << "http://dl.caffe.berkeleyvision.org/bvlc_googlenet.caffemodel" << std::endl;
    exit(-1);
  }
  cv::Mat img = cv::imread(imageFile);
  // cv::Mat img = roi(image, boundingbox);
  if(img.empty()) { // checking if image exists in the file
    std::cerr << "Can't cv::read image from the file: " << imageFile << std::endl;
    exit(-1);
  }
  // GoogLeNet accepts only specific sized RGB-images
  cv::Mat inputBlob =
      blobFromImage(img, 1, cv::Size(224, 224), cv::Scalar(104, 117, 123)); // Convert cv::Mat to batch of images
  cv::Mat prob;
  cv::TickMeter t;
  for(int i = 0; i < 10; i++) { // setting # of iterations for blob cv::dnn method recognition
    CV_TRACE_REGION("forward");
    net.setInput(inputBlob, "data"); // set the network input
    t.start();
    prob = net.forward("prob"); // compute output
    t.stop();
  }
  int classId;
  double classProb;
  getMaxClass(prob, &classId, &classProb); // find the best class

  // Where caffe model detection begins
  std::vector<cv::String> classNames = readClassNames();
  // This is where caffe model will be used to determine the what the object is that is being
  // tracked
  cout << "Best class: #" << classId << " Object Identified as '" << classNames.at(classId) << "'" << endl;
  cout << "Probability: " << classProb * 100 << "%"
       << endl; // It also determines the probability of that object being the detected correctly
  // The time it took to detected the correct item
  cout << "Time: " << (double)t.getTimeMilli() / t.getCounter() << " ms (average from " << t.getCounter() << " iterations)"
       << endl;
}
// Declearing variables for the contour program
cv::Mat src;
cv::Mat src_gray;
cv::Mat new_image;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

int
bg_sub_contour(int argc, char** argv) {

  cout << "A camera will be turned on that displays a user's cv::threshold feed." << endl;
  cout << "This feed lets the user adjust the lightness of the colour, saturation and hue using "
          "the provided trackbars."
       << endl;
  cout << "After the user gets a satisfying frame, press ESC on the feed" << endl;
  cout << "Use OpenCV's built in cv::Canny algorithm to color the contours how the user prefers" << endl; // instructions
  cout << "Press Enter on the Contour screen to exit the program" << endl << endl;

  cv::Mat streamFeed;    // matrix storing frames from webcam feed
  cv::Mat hsv;           // stores hsv image
  cv::Mat cv::threshold; // stires binary cv::threshold image

  createTrackbarsBG(); // creating trackbars for this program

  cv::VideoCapture stream(0);

  if(!stream.isOpened()) {
    cerr << "Camera did not open" << endl << endl;
    return -1;
  }

  for(;;) {                  // iterates a camera feed
    stream.read(streamFeed); // stores image to matrix

    cv::cvtColor(streamFeed, hsv, COLOR_BGR2HSV); // converting from BGR color space to HSV
    cv::inRange(hsv,
                cv::Scalar(hueMin, smin, vmin),
                cv::Scalar(hueMax, smax, vmax),
                cv::threshold); // takes range of min and max values and outputs into cv::threshold matrix

    cv::imshow("Threshold feed", cv::threshold); // displays the produced thresh image very smoothly

    int k = cv::waitKey(1);
    if(k == 27) { // press ESC to exit the thresh feed
      break;
    }
    cv::imwrite("backsub.jpg", cv::threshold); // saves the last frame from cv::threshold feed as an image
  }

  src = cv::imread("backsub.jpg", cv::IMREAD_COLOR); // cv::read the saved image
  if(src.empty()) {
    cerr << "No image supplied ..." << endl; // cv::error check
    return -1;
  }
  cv::cvtColor(src, src_gray, COLOR_BGR2GRAY);
  cv::blur(src_gray, src_gray, cv::Size(3, 3)); // blurs saved image using a normalized filter -> openCV helper function
  const char* source_window = "Source";
  cv::namedWindow(source_window, cv::WINDOW_AUTOSIZE); // using canny algorithm from openCV
  cv::imshow(source_window, src);
  cv::createTrackbar(" cv::Canny thresh:", "Source", &thresh, max_thresh, contour_figure);
  contour_figure(0, 0); // call reference function
  cv::waitKey(0);       // program exits when user presses ENTER
}

// contour_figure is a modified function of an opencv tutorial
void
contour_figure(int, void*) {
  cv::Mat canny_output; // stores canny output
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Canny(src_gray, canny_output, thresh, thresh * 2,
            3); // gathers the edges of the image, marks them in the output map
  cv::findContours(canny_output,
                   contours,
                   hierarchy,
                   RETR_TREE,
                   CHAIN_APPROX_SIMPLE,
                   cv::Point(0, 0)); // retrives the countour from the binary image

  // draw contours
  cv::Mat drawing = cv::Mat::zeros(canny_output.size(),
                                   CV_8UC3); // stores the output of canny to the columns of cv::Mat
  for(size_t i = 0; i < contours.size(); i++) {
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255),
                                  rng.uniform(0, 255),
                                  rng.uniform(0, 255)); // changes scalar colours based on various edges
    cv::drawContours(drawing, contours, (int)i, color, 2, 8, hierarchy, 0,
                     cv::Point()); // adds colours while also drawing the
    // contour lines
  }

  // displays the image
  cv::namedWindow("Contours", cv::WINDOW_AUTOSIZE);
  cv::imshow("Contours", drawing); // modfied to save and show images to work with our program
  cv::imwrite("contour_pic.jpg", drawing);
}

int
main() {

  int option = 0;

  centerstring("PURSUIT OF COLOR SILHOUETTE");
  cout << endl << endl;
  cout << "Welcome to Pursuit of Color Silhouette!" << endl << endl << endl;
  cout << "An interactive program that lets users have with objects near their surroundings." << endl << endl;
  cout << "Lets begin by selecting one of the following options:" << endl
       << endl
       << "(Instructions are provided after selecting an option)" << endl
       << endl;
  cout << "1. Track a selected object based on its color! " << endl << endl;
  cout << "2. Create Abstract Art with your surrounding objects using contours! " << endl << endl;
  cout << "Press 1 or 2 and then ENTER" << endl << endl;
  scanf_s("%d", &option);

  cout << endl << endl << endl << endl;

  if(option < 1 && option > 2) {

    cerr << "Invalid option" << endl;
    return -1;
  }

  if(option == 1) {

    track_and_identify(0, 0);

  } // option selection cases

  else if(option == 2) {
    bg_sub_contour(0, 0);
  }

  cout << endl << endl << endl << endl; // after options are terminated and program wants to end
  cout << "Thank You for using our Multimedia program! :) " << endl;
  return 0;
}
