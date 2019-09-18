// OpenCVWebcamTest.cpp

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

#include <iostream>
typedef std::vector<cv::Point> PointVec;
typedef std::vector<cv::Point2f> Point2fVec;

int thresh = 10;
int max_thresh = 255;

// Function that calculates the absolute value

template <class T, class O>
void
out_point(O& os, const cv::Point_<T>& pt) {
  os << pt.x;
  os << ',';
  os << pt.y;
}

template <class O>
void
out_hier(O& os, const cv::Vec4i& v) {
  os << '{';
  os << "next:" << v[0];
  os << ",prev:" << v[1];
  os << ",children:" << v[2];
  os << ",parent:" << v[3];
  os << '}';
}

/**
 * @brief      Ôutput point list
 * @return     { description_of_the_return_value }
 */
template <class O>
void
out_points(O& os, const std::vector<cv::Point>& pl) {
  size_t i, n = pl.size();
  for(i = 0; i < n; ++i) {
    if(i > 0)
      os << ' ';
    out_point(os, pl[i]);
  }
}

template <class T>
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

void
applyCLAHE(const cv::Mat& bgr_image, cv::Mat& image_clahe) {
  cv::Mat lab_image;
  cv::cvtColor(bgr_image, lab_image, CV_BGR2Lab);

  // Extract the L channel
  std::vector<cv::Mat> lab_planes(3);
  cv::split(lab_image, lab_planes); // now we have the L image in lab_planes[0]

  // apply the CLAHE algorithm to the L channel
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(4);
  cv::Mat dst;
  clahe->apply(lab_planes[0], dst);

  // Merge the the color planes back into an Lab image
  dst.copyTo(lab_planes[0]);
  cv::merge(lab_planes, lab_image);

  // convert back to RGB
  cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);
}

/**
 *  \brief Automatic brightness and contrast optimization with optional histogram clipping
 *  \param [in]src Input image GRAY or BGR or BGRA
 *  \param [out]dst Destination image
 *  \param clipHistPercent cut wings of histogram at given percent tipical=>1, 0=>Disabled
 *  \note In case of BGRA image, we won't touch the transparency
 */
void
BrightnessAndContrastAuto(const cv::Mat& src, cv::Mat& dst, float clipHistPercent = 0) {

  CV_Assert(clipHistPercent >= 0);
  CV_Assert((src.type() == CV_8UC1) || (src.type() == CV_8UC3) || (src.type() == CV_8UC4));

  int histSize = 256;
  float alpha, beta;
  double minGray = 0, maxGray = 0;

  // to calculate grayscale histogram
  cv::Mat gray;
  if(src.type() == CV_8UC1)
    gray = src;
  else if(src.type() == CV_8UC3)
    cvtColor(src, gray, CV_BGR2GRAY);
  else if(src.type() == CV_8UC4)
    cvtColor(src, gray, CV_BGRA2GRAY);
  if(clipHistPercent == 0) {
    // keep full available range
    cv::minMaxLoc(gray, &minGray, &maxGray);
  } else {
    cv::Mat hist; // the grayscale histogram

    float range[] = {0, 256};
    const float* histRange = {range};
    bool uniform = true;
    bool accumulate = false;
    calcHist(&gray, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

    // calculate cumulative distribution from the histogram
    std::vector<float> accumulator(histSize);
    accumulator[0] = hist.at<float>(0);
    for(int i = 1; i < histSize; i++) {
      accumulator[i] = accumulator[i - 1] + hist.at<float>(i);
    }

    // locate points that cuts at required value
    float max = accumulator.back();
    clipHistPercent *= (max / 100.0); // make percent as absolute
    clipHistPercent /= 2.0;           // left and right wings
    // locate left cut
    minGray = 0;
    while(accumulator[minGray] < clipHistPercent) minGray++;

    // locate right cut
    maxGray = histSize - 1;
    while(accumulator[maxGray] >= (max - clipHistPercent)) maxGray--;
  }

  // current range
  float inputRange = maxGray - minGray;

  alpha = (histSize - 1) / inputRange; // alpha expands current range to histsize range
  beta = -minGray * alpha;             // beta shifts current range so that minGray will go to 0

  // Apply brightness and contrast normalization
  // convertTo operates with saurate_cast
  src.convertTo(dst, -1, alpha, beta);

  // restore alpha channel from source
  if(dst.type() == CV_8UC4) {
    int from_to[] = {3, 3};
    cv::mixChannels(&src, 4, &dst, 1, from_to, 1);
  }
  return;
}
cv::Mat
imageToBinary(cv::Mat start) {
  cv::Mat gray_image, thresh_image;
  cvtColor(start, gray_image, CV_BGR2GRAY);
  threshold(gray_image, thresh_image, 100, 255, cv::THRESH_BINARY);

  medianBlur(thresh_image, thresh_image, 5);

  return thresh_image;
}

/**
 * @brief      Gets the contours.
 *
 * @param[in]  start      The start
 * @param      hierarchy  The hierarchy
 * @param[in]  flag       The flag
 *
 * @return     The contours.
 */
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
main(int argc, char* argv[]) {
  int camID = argc > 1 ? atoi(argv[1]) : 0;

  cv::VideoCapture capWebcam(camID); // declare a VideoCapture object and associate to webcam, 0 => use 1st webcam

  if(capWebcam.isOpened() == false) { // check if VideoCapture object was associated to webcam successfully
    std::cout << "error: capWebcam not accessed successfully\n\n"; // if not, print error message to std out
    getchar();                                                     // may have to modify this line if not using Windows
    return (0);                                                    // and exit program
  }

  cv::Mat imgRaw, imgOriginal, imgTemp, imgGrayscale, imgBlurred, imgCanny; // Canny edge image

  char charCheckForEscKey = 0;

  // declare windows
  //  note: you can use CV_WINDOW_NORMAL which allows resizing the window
  cv::namedWindow("imgOriginal", CV_WINDOW_AUTOSIZE);
  // or CV_WINDOW_AUTOSIZE for a fixed size window matching the resolution of the image
  cv::namedWindow("imgCanny", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("imgGrayscale", CV_WINDOW_AUTOSIZE);

  while(charCheckForEscKey != 27 && capWebcam.isOpened()) { // until the Esc key is pressed or webcam connection is lost
    bool blnFrameReadSuccessfully = capWebcam.read(imgRaw); // get next frame

    if(!blnFrameReadSuccessfully || imgRaw.empty()) {     // if frame not read successfully
      std::cout << "error: frame not read from webcam\n"; // print error message to std out
      break;                                              // and jump out of while loop
    }
    // cv::normalize(imgOriginal,imgTemp,0,255,cv::NORM_L1);
    imgRaw.copyTo(imgOriginal);

    cvtColor(imgOriginal, imgGrayscale, CV_BGR2GRAY); // convert to grayscale

    /// Apply Histogram Equalization
    //  cv::equalizeHist(imgTemp, imgGrayscale);

    cv::GaussianBlur(imgGrayscale, imgBlurred, cv::Size(5, 5), 1.75);

    cv::Canny(imgBlurred, imgCanny, thresh, thresh * 2, 3);
    equalizeHist(imgGrayscale, imgGrayscale);

    //  applyCLAHE(imgOriginal, imgOriginal);XY

    std::vector<Point2fVec> contours2;
    std::vector<cv::Vec4i> hier;
    std::vector<PointVec> contours = getContours(imgCanny, hier, CV_RETR_TREE);

    std::ostringstream contourStr;

    std::for_each(contours.cbegin(), contours.cend(), [&](const std::vector<cv::Point>& a) {
      if(a.size() >= 3) {

        if(contourStr.str().size())
          contourStr << "\n";
        out_points(contourStr, a);
      }
    });

    std::cout << contourStr.str() << std::endl;

    for(size_t i = 0; i < contours.size(); ++i) {
      std::vector<cv::Point> c = contours[i];
      cv::Vec4i h = hier[i];

      std::cout << '#' << i << std::endl;
      out_points(std::cout, c);
      ///  std::cout << c << std::endl;
      out_hier(std::cout, h);
      std::cout << std::endl;
    }

    //       cv:: drawContours( imgOriginal, contours, -1, cv::Scalar(0,0,255), 1, cv::LINE_AA, hier );

    /*
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
        //  std::cout << i << ": " << area << std::endl;

          if(npts > 0) {
            std::vector<cv::Point> pl = ToPointVec(contours2[i]);
            cv::polylines(imgOriginal, pl, true, cv::Scalar(0, 0, 255), 1);
          }
        }
    */
    // CV_WINDOW_AUTOSIZE is the default
    cv::imshow("imgOriginal", imgOriginal);   // show windows
    cv::imshow("imgCanny", imgCanny);         //
    cv::imshow("imgGrayscale", imgGrayscale); //

    charCheckForEscKey = cv::waitKey(1); // delay (in ms) and get key press, if any
  }                                      // end while

  return (0);
}
