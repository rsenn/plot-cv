// OpenCVWebcamTest.cpp

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
typedef std::vector<cv::Point> PointVec;
typedef std::vector<cv::Point2f> Point2fVec;
// Function that calculates the absolute value

template <class T, class O>
O&
operator<<(O& os, const cv::Point_<T>& pt) {
  os << '[';
  os << pt.x;
  os << ',';
  os << pt.y;
  os << ']';
}

template <class P, class O>
O& 
operator<<(O& os, const std::vector<P>& pl) {
  size_t i, n = pl.size();
  for(i = 0; i < n; ++i) {
    if(i > 0) os << ',';
    os << pl[i];
  }
}

template<class T>
std::string
to_string(const T& t) {
 std::ostringstream oss;
 oss << t;
 return oss.str();
}

// of a double type.
double
numAbs(double num) {
  double inv = num * -1;
  return (num <= 0) ? inv : num;
}
// Function that calculates the area given a
// std::vector of vertices in the XY plane.
template <class P>
double
polygonArea(std::vector<P> list) {

  if(list.size() < 3)
    return 0;
  double area = 0;                     // Total Area
  double diff = 0;                     // Difference Of Y{i + 1} - Y{i - 1}
  unsigned int last = list.size() - 1; // Size Of Vector - 1
  /* Given vertices from 1 to n, we first loop through
  the vertices 2 to n - 1. We will take into account
  vertex 1 and vertex n sepereately */
  for(unsigned int i = 1; i < last; i++) {
    diff = list[i + 1].y - list[i - 1].y;
    area += list[i].x * diff;
  }
  /* Now We Consider The Vertex 1 And The Vertex N */
  diff = list[1].y - list[last].y;
  area += list[0].x * diff; // Vertex 1
  diff = list[0].y - list[last - 1].y;
  area += list[last].x * diff; // Vertex N
  /* Calculate The Final Answer */
  area = 0.5 * numAbs(area);
  return area; // Return The Area
}

cv::Mat
imageToBinary(cv::Mat start) {
  cv::Mat gray_image, thresh_image;
  cvtColor(start, gray_image, CV_BGR2GRAY);
  threshold(gray_image, thresh_image, 100, 255, cv::THRESH_BINARY);

  medianBlur(thresh_image, thresh_image, 5);

  return thresh_image;
}

std::vector<PointVec>
getContours(cv::Mat start, std::vector<cv::Vec4i>& hierarchy, int flag = CV_RETR_EXTERNAL) {

  cv::Mat dst = cv::Mat::zeros(start.rows, start.cols, CV_8UC3);
  std::vector<PointVec> contours;

  start = start > 1;

  cv::findContours(start, contours, hierarchy, flag, CV_CHAIN_APPROX_SIMPLE);

  return contours;
}

std::vector<cv::Point2f>
getMassCenters(std::vector<std::vector<cv::Point>> contours) {

  std::vector<cv::Moments> mu(contours.size());
  std::vector<cv::Point2f> mc(contours.size());

  for(size_t i = 0; i < contours.size(); i++) {
    mu[i] = cv::moments(contours[i], false);
  }

  for(size_t i = 0; i < contours.size(); i++) {
    mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
  }

  return mc;
}

template <class InputType>
std::vector<cv::Point>
ToPointVec(const std::vector<InputType>& v) {
  std::vector<cv::Point> ret;

  std::for_each(v.cbegin(), v.cend(), [&ret](const InputType& pt) { ret.push_back(cv::Point(pt.x, pt.y)); });

  return ret;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
int
main() {
  cv::VideoCapture capWebcam(0); // declare a VideoCapture object and associate to webcam, 0 => use 1st webcam

  if(capWebcam.isOpened() == false) { // check if VideoCapture object was associated to webcam successfully
    std::cout << "error: capWebcam not accessed successfully\n\n"; // if not, print error message to std out
    getchar();                                                     // may have to modify this line if not using Windows
    return (0);                                                    // and exit program
  }

  cv::Mat imgOriginal, imgTemp, imgGrayscale, imgBlurred, imgCanny; // Canny edge image

  char charCheckForEscKey = 0;

  // declare windows
  //  note: you can use CV_WINDOW_NORMAL which allows resizing the window
  cv::namedWindow("imgOriginal", CV_WINDOW_AUTOSIZE);
  // or CV_WINDOW_AUTOSIZE for a fixed size window matching the resolution of the image
  cv::namedWindow("imgCanny", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("imgGrayscale", CV_WINDOW_AUTOSIZE);

  while(charCheckForEscKey != 27 && capWebcam.isOpened()) { // until the Esc key is pressed or webcam connection is lost
    bool blnFrameReadSuccessfully = capWebcam.read(imgOriginal); // get next frame

    if(!blnFrameReadSuccessfully || imgOriginal.empty()) { // if frame not read successfully
      std::cout << "error: frame not read from webcam\n";  // print error message to std out
      break;                                               // and jump out of while loop
    }
    // cv::normalize(imgOriginal,imgTemp,0,255,cv::NORM_L1);

    cvtColor(imgOriginal, imgTemp, CV_BGR2GRAY); // convert to grayscale

    /// Apply Histogram Equalization
    cv::equalizeHist(imgTemp, imgGrayscale);

    cv::GaussianBlur(imgGrayscale,   // input image
                     imgBlurred,     // output image
                     cv::Size(5, 5), // smoothing window width and height in pixels
                     1.8);           // sigma value, determines how much the image will be blurred

    cv::Canny(imgBlurred, // input image
              imgCanny,   // output image
              10,         // low threshold
              20,         // high threshold
              3);

    std::vector<Point2fVec> contours2;
    std::vector<cv::Vec4i> hier;
    std::vector<PointVec> contours = getContours(imgCanny, hier, CV_RETR_TREE);

    std::string contoursStr = to_string(contours);

    std::cout << contoursStr << std::endl;

    std::for_each(contours.cbegin(), contours.cend(), [&contours2](const std::vector<cv::Point>& a) {
      double area = polygonArea(a);

      if(a.size() >= 3 && area > 8) {
        Point2fVec b;
        cv::approxPolyDP(a, b, 0.5, true);
        contours2.push_back(b);
      }
    });

    std::sort(contours2.begin(), contours2.end(), [](Point2fVec a, Point2fVec b) -> bool {
      return polygonArea<cv::Point2f>(a) >= polygonArea<cv::Point2f>(b);
    });

    for(size_t i = 0; i < std::min<size_t>(100, contours2.size()); ++i) {
      int npts = contours2[i].size();
      double area = polygonArea(contours2[i]);
      std::cout << i << ": " << area << std::endl;

      if(npts > 0) {
        std::vector<cv::Point> pl = ToPointVec(contours2[i]);
        cv::polylines(imgOriginal, pl, true, cv::Scalar(0, 0, 255), 1);
      }
    }

    // CV_WINDOW_AUTOSIZE is the default
    cv::imshow("imgOriginal", imgOriginal);   // show windows
    cv::imshow("imgCanny", imgCanny);         //
    cv::imshow("imgGrayscale", imgGrayscale); //

    charCheckForEscKey = cv::waitKey(1); // delay (in ms) and get key press, if any
  }                                      // end while

  return (0);
}
