// OpenCVWebcamTest.cpp

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgcodecs.hpp>
#include <unistd.h>

#include "simple_svg_1.0.0.hpp"
#include "psimpl.h"
#include "line.h"
#include "matrix.h"

#include <type_traits>
#include <iostream>
#include <algorithm>
#include <list>
#include <functional>
#include <unordered_map>
#include <map>
typedef std::vector<cv::Point> PointVec;
typedef std::vector<cv::Point2f> Point2fVec;

int thresh = 10;
int max_thresh = 255;
int largestContour = -1;
double max_svg_width = 1200; // pixels
double max_svg_height = 900; // pixels
int thresholdValue = 155;
bool showDiagnostics = false;
std::vector<cv::Point2f> bigContour;
double epsilon = 3;

template <class ValueT>
ValueT*
coord_pointer(cv::Point_<ValueT>* point_ptr) {
  return reinterpret_cast<ValueT*>(point_ptr);
}
template <class ValueT>
const ValueT*
coord_pointer(const cv::Point_<ValueT>* point_ptr) {
  return reinterpret_cast<const ValueT*>(point_ptr);
}

template <class PointT>
std::vector<PointT>
simplify_polyline(const std::vector<PointT>& points) {
  typedef typename PointT::value_type coord_type;
  std::vector<PointT> ret;
  ret.resize(points.size());

  psimpl::PolylineSimplification<2, const coord_type*, coord_type*> psimpl;
  auto output = coord_pointer(ret.data());

  // auto end = psimpl.NthPoint(coord_pointer(points.data()), coord_pointer(&points.data()[points.size()]), 20, output);
  // auto end = psimpl.RadialDistance(coord_pointer(points.data()), coord_pointer(&points.data()[points.size()]), 10,
  // output);
  auto end = psimpl.Opheim(coord_pointer(points.data()), coord_pointer(&points.data()[points.size()]), 4, 30, output);
  size_t outn = std::distance(output, end) / 2;

  // std::cout << "simplification 1:" << ((double)points.size() / outn) << std::endl;
  ret.resize(outn);
  return ret;
}

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

// finds the largest contour and stores the bigContour and stores it index which are both global variables.

void
filter_contours(std::vector<std::vector<cv::Point2f>> contours_un) {
  double maxArea = 0.0;
  for(size_t i = 0; i < contours_un.size(); i++) {
    double area = contourArea(contours_un[i]);
    if(showDiagnostics) {
      std::cout << "Area: " + to_string(area) + "Index: " + to_string(i) << std::endl;
    }
    if(area > maxArea) {
      maxArea = area;
      largestContour = i;
    }
  }
  bigContour = contours_un.at(largestContour);
}

void
polyline_from_contour(svg::Document& doc, const std::vector<cv::Point2f>& contour_arg, std::function<svg::Color(const std::vector<cv::Point2f>&)> color_fn) {
  svg::Polyline polyline(svg::Stroke(1, color_fn(contour_arg)));

  for(size_t i = 0; i < contour_arg.size(); i++) {
    svg::Point tmp = svg::Point(contour_arg.at(i).x, contour_arg.at(i).y);
    polyline << tmp;
  }

  doc << polyline;
}

/*
 * H(Hue): 0 - 360 degree (integer)
 * S(Saturation): 0 - 1.00 (double)
 * V(Value): 0 - 1.00 (double)
 *
 * output[3]: Output, array size 3, int
 */
cv::Scalar
hs_vto_rgb(int H, double S, double V) {
  double C = S * V;
  double X = C * (1 - abs(fmod(H / 60.0, 2) - 1));
  double m = V - C;
  double Rs, Gs, Bs;

  if(H >= 0 && H < 60) {
    Rs = C;
    Gs = X;
    Bs = 0;
  } else if(H >= 60 && H < 120) {
    Rs = X;
    Gs = C;
    Bs = 0;
  } else if(H >= 120 && H < 180) {
    Rs = 0;
    Gs = C;
    Bs = X;
  } else if(H >= 180 && H < 240) {
    Rs = 0;
    Gs = X;
    Bs = C;
  } else if(H >= 240 && H < 300) {
    Rs = X;
    Gs = 0;
    Bs = C;
  } else {
    Rs = C;
    Gs = 0;
    Bs = X;
  }

  return cv::Scalar((int)((Rs + m) * 255), (int)((Gs + m) * 255), (int)((Bs + m) * 255));
}

svg::Color
from_scalar(const cv::Scalar& s) {
  return svg::Color(s[0], s[1], s[2]);
}

template <class FromT, class ToT>
void
convert_points(const std::vector<cv::Point_<FromT>>& from, std::vector<cv::Point_<ToT>>& to) {
  std::transform(from.begin(), from.end(), std::back_inserter(to), [](cv::Point_<FromT> p) -> cv::Point_<ToT> { return cv::Point_<ToT>(p.x, p.y); });
}

template <class FromT, class ToT>
std::vector<cv::Point_<ToT>>
transform_points(const std::vector<cv::Point_<FromT>>& from) {
  std::vector<cv::Point_<ToT>> ret;
  convert_points<FromT, ToT>(from, ret);
  return ret;
}

template <class PointType>
void
export_svg(const std::vector<std::vector<PointType>>& contours, std::string output_file) {

  std::cout << "Saving '" << output_file << "'" << std::endl;

  svg::Dimensions dimensions(max_svg_width, max_svg_height);
  svg::Document doc(output_file, svg::Layout(dimensions, svg::Layout::TopLeft));
  svg::LineChart chart(5.0);
  std::vector<double> areas;
  std::transform(contours.begin(), contours.end(), std::back_inserter(areas), [](const std::vector<PointType>& contour) -> double { return cv::contourArea(transform_points<float, int>(contour)); });
  const auto& it = std::max_element(areas.begin(), areas.end());
  double max_area = 0;
  if(it != areas.end()) {
    max_area = *it;
  }

  std::cout << "Max area: " << max_area << std::endl;

  const auto& cfn = [&](const std::vector<PointType>& contour) -> svg::Color {
    const double area = cv::contourArea(contour);
    return from_scalar(hs_vto_rgb(area / max_area * 360, 1, 1));
  };
  std::for_each(contours.begin(), contours.end(), std::bind(&polyline_from_contour, std::ref(doc), std::placeholders::_1, cfn));
  //  polyline_from_contour(doc, contour_arg, svg::Color(255, 0, 0));

  doc.save();

  std::cout << "EXPORTED SVG" << std::endl;
}

/** @function main */

// Function that calculates the area given a
// std::vector of vertices in the XY plane.
template <class P>
double
polygon_area(std::vector<P> list) {

  if(list.size() < 3)
    return 0;
  double area = 0;                     // Total Area
  double diff = 0;                     // Difference Of Y{i + 1} - Y{i - 1}
  unsigned int last = list.size() - 1; // Size Of Vector - 1
  /* Given vertices from 1 to n, we first loop through
  the vertices 2 to n - 1. We will take into account
  vertex 1 and vertex n sepereately */
  for(size_t i = 1; i < last; i++) {
    diff = list[i + 1].y - list[i - 1].y;
    area += list[i].x * diff;
  }
  /* Now We Consider The Vertex 1 And The Vertex N */
  diff = list[1].y - list[last].y;
  area += list[0].x * diff; // Vertex 1
  diff = list[0].y - list[last - 1].y;
  area += list[last].x * diff; // Vertex N
  /* Calculate The Final Answer */
  area = 0.5 * fabs(area);
  return area; // Return The Area
}

void
apply_clahe(const cv::Mat& bgr_image, cv::Mat& image_clahe) {
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
brightness_and_contrast_auto(const cv::Mat& src, cv::Mat& dst, float clipHistPercent = 0) {

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
image_to_binary(cv::Mat start) {
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
get_contours(cv::Mat start, std::vector<cv::Vec4i>& hierarchy, int flag = CV_RETR_EXTERNAL) {
  cv::Mat dst = cv::Mat::zeros(start.rows, start.cols, CV_8UC3);
  std::vector<PointVec> contours;
  start = start > 1;
  cv::findContours(start, contours, hierarchy, flag, CV_CHAIN_APPROX_SIMPLE);
  return contours;
}

std::vector<cv::Point2f>
get_mass_centers(std::vector<PointVec> contours) {
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
to_point_vec(const std::vector<InputType>& v) {
  std::vector<cv::Point> ret;
  std::for_each(v.begin(), v.end(), [&ret](const InputType& pt) { ret.push_back(cv::Point(pt.x, pt.y)); });
  return ret;
}

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
static double
angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

void
find_rectangles(const std::vector<PointVec>& contours, std::vector<PointVec>& squares) {

  // test each contour
  for(size_t i = 0; i < contours.size(); i++) {
    PointVec approx;
    double arcLen = cv::arcLength(cv::Mat(contours[i]), true);

    // approximate contour with accuracy proportional
    // to the contour perimeter
    cv::approxPolyDP(cv::Mat(contours[i]), approx, arcLen * 0.02, true);

    // square contours should have 4 vertices after approximation
    // relatively large area (to filter out noisy contours)
    // and be convex.
    // Note: absolute value of an area is used because
    // area may be positive or negative - in accordance with the
    // contour orientation
    if(arcLen > 80 || fabs(cv::contourArea(cv::Mat(approx))) > 200 /*|| cv::isContourConvex(cv::Mat(approx))*/) {
      /*      double maxCosine = 0;

            for(int j = 2; j < 5; j++) {
              // find the maximum cosine of the angle between joint edges
              double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
              maxCosine = MAX(maxCosine, cosine);
            }*/

      // if cosines of all angles are small
      // (all angles are ~90 degree) then write quandrange
      // vertices to resultant sequence

      // if(maxCosine < 0.3)
      squares.push_back(approx);
    }
  }
}

// the function draws all the squares in the image

template <class PointT>
static void
draw_polylines(cv::Mat& image, const std::vector<std::vector<PointT>>& polylines, const cv::Scalar& color = cv::Scalar(0, 255, 0)) {

  cv::polylines(image, polylines, true, color, 2, cv::LINE_AA);
}

void
invert_color(cv::Mat& img) {
  for(int i = 0; i < img.rows; i++)
    for(int j = 0; j < img.cols; j++) img.at<uchar>(i, j) = 255 - img.at<uchar>(i, j);
}

std::vector<cv::Vec4i>
hough_lines(cv::Mat& imgToMap) {
  cv::Mat imgToMapProc;
  cvtColor(imgToMap, imgToMapProc, CV_BGR2GRAY);
  cv::threshold(imgToMapProc, imgToMapProc, thresholdValue, 255, 0);
  std::vector<cv::Vec4i> lines;
  invert_color(imgToMapProc);
  cv::HoughLinesP(imgToMapProc, lines, 1, CV_PI / 180, 30, 30, 10);

  return lines;
}

void
draw_lines(cv::Mat& target, const std::vector<cv::Vec4i>& lines) {
  for(size_t i = 0; i < lines.size(); i++) cv::line(target, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 255, 255), 1);
}

void
corner_harris_detection(cv::Mat& src, cv::Mat& src_gray) {
  int thresh = 200;
  int max_thresh = 255;
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;
  cv::Mat dst = cv::Mat::zeros(src.size(), CV_32FC1);
  cv::Mat gray = cv::Mat::zeros(src.size(), CV_32FC1);
  src_gray.convertTo(gray, CV_32FC1);
  cv::cornerHarris(gray, dst, blockSize, apertureSize, k);
  cv::Mat dst_norm, dst_norm_scaled;
  cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
  cv::convertScaleAbs(dst_norm, dst_norm_scaled);
  for(int i = 0; i < dst_norm.rows; i++) {
    for(int j = 0; j < dst_norm.cols; j++) {
      if((int)dst_norm.at<float>(i, j) > thresh) {
        cv::circle(dst_norm_scaled, cv::Point(j, i), 5, cv::Scalar(1), 2, 8, 0);
      }
    }
  }
  cv::namedWindow("corners");
  cv::imshow("corners", dst_norm_scaled);
}

/**
 * @brief      F
 *
 * @param[in]  input  The input
 * @param      u      { parameter_description }
 */
void
trackbar(int input, void* u) {
  const char* name = reinterpret_cast<const char*>(const_cast<const void*>(u));
  static int epsilon, blur;
  if(!strcmp(name, "eps"))
    epsilon = input;
  if(!strcmp(name, "blur"))
    blur = input;
};

void
write_image(const cv::Mat& img) {
  static int count = 0;
  std::ostringstream filename;
  filename << "frame-";
  filename << to_string((++count) % 100, 3, '0');
  filename << ".png";

  cv::imwrite(cv::String(filename.str()), img);
}

void
draw_all_contours(cv::Mat& out, std::vector<PointVec>& contours) {

  for(size_t i = 0; i < contours.size(); i++) {
    const cv::Scalar color = hs_vto_rgb((i * 360 * 10 / contours.size()) % 360, 1.0, 1.0);
    auto contour = simplify_polyline(contours[i]);
    contours[i] = contour;

    const double area = cv::contourArea(contours[i], false);
    if(area < 1)
      continue;

    cv::drawContours(out, contours, i, color, 1, cv::LINE_AA);
  }
}

template <class Container>
void
draw_all_lines(cv::Mat& out, const Container& lines, const std::function<int(int, size_t)>& hue = [](int index, size_t len) -> int { return (index * 360 * 10 / len) % 360; }) {

  typedef typename Container::const_iterator iterator_type;
  for(iterator_type it = lines.begin(); it != lines.end(); it++) {
    size_t i = std::distance(lines.begin(), it);
    const cv::Scalar color = hs_vto_rgb(hue(i, lines.size()), 1.0, 1.0);

    const double len = it->length();
    if(len < 8)
      continue;

    cv::line(out, cv::Point(it->a), cv::Point(it->b), color, 1, cv::LINE_AA);
  }
}

template <class InputIterator, class Pred>
std::vector<int>
filter_lines(InputIterator from, InputIterator to, Pred predicate) {
  typedef InputIterator iterator_type;
  typedef typename std::iterator_traits<InputIterator>::value_type value_type;
  std::vector<int> ret;
  size_t index = 0;
  for(iterator_type it = from; it != to; ++it) {

    if(predicate(*it, index++)) {

      std::size_t index = std::distance(from, it);
      ret.push_back(index);
    }
  }
  return ret;
}

template <class T> class PredicateTraits {
public:
  typedef bool type(const Line<T>&, size_t);
  typedef std::function<bool(const Line<T>&, size_t)> function;
};
template <class T, class Pred>
std::vector<int>
filter_lines(const std::vector<T>& c, bool (&pred)(const Line<T>&, size_t)) {
  return filter_lines<std::vector<Line<T>>::iterator, bool(Line<T>&, size_t)>(c.begin(), c.end(), pred);
}

template <class ValueT, class InputIterator>
std::vector<typename std::iterator_traits<InputIterator>::value_type::value_type>
angle_diffs(Line<ValueT>& line, InputIterator from, InputIterator to) {
  typedef InputIterator iterator_type;
  typedef typename std::iterator_traits<InputIterator>::value_type point_type;
  typedef typename point_type::value_type value_type;
  typedef std::vector<value_type> ret_type;

  ret_type ret;
  value_type distance = 1e10;
  iterator_type index = to;

  for(iterator_type it = from; it != to; ++it) {
    value_type d;

    ret.push_back((*it).angle_diff(line));
  }
  return ret;
}

template <class InputIterator>
std::vector<float>
line_distances(typename std::iterator_traits<InputIterator>::value_type& line, InputIterator from, InputIterator to) {
  typedef InputIterator iterator_type;
  typedef typename std::iterator_traits<InputIterator>::value_type line_type;
  typedef typename line_type::value_type value_type;
  typedef std::vector<float> ret_type;

  ret_type ret;
  value_type distance = 1e10;
  iterator_type index = to;

  for(iterator_type it = from; it != to; ++it) {
    /* if(line == *it)
       continue;*/
    ret.push_back(it->min_distance(line));
  }
  return ret;
}

template <class Char, class Value>
std::basic_ostream<Char>&
operator<<(std::basic_ostream<Char>& os, const std::vector<Value>& c) {
  typedef typename std::vector<Value>::const_iterator iterator_type;
  iterator_type end = c.end();
  for(iterator_type it = c.begin(); it != end; ++it) {
    os << ' ';
    os << to_string(*it);
  }
  return os;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
int
main(int argc, char* argv[]) {
  using std::array;
  using std::back_inserter;
  using std::cout;
  using std::distance;
  using std::endl;
  using std::for_each;
  using std::iterator_traits;
  using std::transform;
  using std::vector;

  int camID = argc > 1 ? atoi(argv[1]) : 0;
  int count = 0;

  cv::VideoCapture capWebcam;

  capWebcam.open(camID, cv::CAP_V4L2); // declare a VideoCapture object and associate to webcam, 0 => use 1st webcam

  if(capWebcam.isOpened() == false) {                         // check if VideoCapture object was associated to webcam successfully
    cout << "error: capWebcam not accessed successfully\n\n"; // if not, print error message to std out
    getchar();                                                // may have to modify this line if not using Windows
    return (0);                                               // and exit program
  }

  cv::Mat imgRaw, imgOriginal, imgTemp, imgGrayscale, imgBlurred, imgCanny; // Canny edge image

  char charCheckForEscKey = 0;
  int levels = 3;
  int eps = 8;
  int blur = 4;

  cv::namedWindow("imgOriginal", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("imgCanny", CV_WINDOW_AUTOSIZE);
  // cv::namedWindow("imgGrayscale", CV_WINDOW_AUTOSIZE);
  cv::createTrackbar("epsilon", "contours", &eps, 7, trackbar, (void*)"eps");
  cv::createTrackbar("blur", "contours", &blur, 7, trackbar, (void*)"blur");

  while(charCheckForEscKey != 27 /*&& capWebcam.isOpened()*/) { // until the Esc key is pressed or webcam connection is lost
    bool blnFrameReadSuccessfully = true;

    blnFrameReadSuccessfully = capWebcam.read(imgRaw); // get next frame
                                                       // imgRaw = cv::imread("input.png");

    if(!blnFrameReadSuccessfully || imgRaw.empty()) { // if frame not read successfully
      cout << "error: frame not read from webcam\n";  // print error message to std out
      break;                                          // and jump out of while loop
    }

    write_image(imgRaw);

    cout << "got frame" << endl;
    // cv::normalize(imgRaw,imgOriginal,0,255,cv::NORM_L1);
    imgRaw.copyTo(imgOriginal);

    cv::Mat frameLab, frameLabCn[3];
    cv::cvtColor(imgOriginal, frameLab, cv::COLOR_BGR2Lab);
    cv::split(frameLab, frameLabCn);
    frameLabCn[0].copyTo(imgGrayscale);

    // cvtColor(imgOriginal, imgGrayscale, CV_BGR2GRAY); // convert to grayscale
    vector<cv::Vec3f> circles;

    // corner_harris_detection(imgOriginal, imgGrayscale);

    /// Apply Histogram Equalization
    // cv::equalizeHist(imgGrayscale, imgGrayscale);

    cv::GaussianBlur(imgGrayscale, imgBlurred, cv::Size(5, 5), 1.75, 1.75);

    cv::Canny(imgBlurred, imgCanny, thresh, thresh * 2, 3);
    cv::cvtColor(imgGrayscale, imgGrayscale, cv::COLOR_GRAY2BGR);

    //  apply_clahe(imgOriginal, imgOriginal);XY
    {

      vector<Point2fVec> contours2;
      vector<cv::Vec4i> hier;
      vector<PointVec> contours = get_contours(imgCanny, hier, CV_RETR_TREE);

      imgCanny = cv::Scalar::all(255) - imgCanny;

      // imgCanny.convertTo(imgCanny, CV_8UC3);
      cv::cvtColor(imgCanny, imgCanny, cv::COLOR_GRAY2BGR);

      // cvtColor(imgOriginal, imgGrayscale, CV_GRAY);
      cv::drawContours(imgCanny, contours, -1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

      cv::imshow("imgCanny", imgCanny); //

      if(contours.empty()) {
        cout << "No contours" << endl;
        charCheckForEscKey = cv::waitKey(100);
        continue;
      }

      std::ostringstream contourStr;
      double maxArea = 0;

      typedef Line<float> line_type;
      typedef vector<line_type> line_list;
      typedef vector<int> ref_list;
      vector<line_type> lines;
      std::map<int, ref_list> adjacency_list;

      for_each(contours.begin(), contours.end(), [&](const vector<cv::Point>& a) {
        size_t i;

        if(a.size() >= 3) {

          Point2fVec c;
          cv::approxPolyDP(a, c, 8, true);
          double area = cv::contourArea(c);
          if(area > maxArea)
            maxArea = area;
          contours2.push_back(c);

          if(contourStr.str().size())
            contourStr << "\n";
          out_points(contourStr, a);
        }
      });

      if(maxArea == 0) {
        charCheckForEscKey = cv::waitKey(16);
        cout << "No contour area" << endl;
        continue;
      }

      for_each(contours2.begin(), contours2.end(), [&lines](const vector<cv::Point2f>& a) {
        double len = cv::arcLength(a, false);
        double area = cv::contourArea(a);

        if(len >= 2) {

          for(size_t i = 0; i + 1 < a.size(); i++) {

            Line<float> l(a[i], a[i + 1]);

            if(l.length() > 0)
              lines.push_back(l);
          }
        }
      });

      cout << "Num contours: " << contours.size() << endl;
      /*
            for_each(lines.begin(), lines.end(), [&](Line<float>& l) {
              Line<float>& nearest = find_nearest_line(l, lines);
            });*/
      std::list<Line<float>> filteredLines;
      vector<bool> takenLines;
      vector<float> lineLengths;

      array<int, 8> histogram = {0, 0, 0, 0, 0, 0, 0, 0};
      array<float, 8> angles = {0, 0, 0, 0, 0, 0, 0, 0};

      vector<cv::Vec4i> linesHier;
      linesHier.resize(lines.size());
      takenLines.resize(lines.size());

      //     sort(lines.begin(), lines.end());

      transform(lines.begin(), lines.end(), back_inserter(lineLengths), [&](Line<float>& l) -> float { return l.length(); });

      float avg = accumulate(lineLengths.begin(), lineLengths.end(), 0) / lineLengths.size();

      const int binsize = 180;
      for(size_t i = 0; i < lines.size(); ++i) {
        Line<float>& line = lines[i];
        double length = line.length();
        double range = (length - avg) / 2;
        if(length > (length - range)) {
          double angle = line.angle() * (double)histogram.size() / M_PI;
          float degrees = angle * 180 / M_PI;
          int deg = (int)degrees % binsize;

          vector<float> distances; // = line_distances(line, lines.begin(), lines.end());
          vector<float> angleoffs = angle_diffs(line, lines.begin(), lines.end());
          vector<LineEnd<float>> line_ends;
          vector<Line<float>*> adjacent_lines;

          vector<int> adjacent = filter_lines(lines.begin(), lines.end(), [&](Line<float>& l2, size_t index) -> bool {
            size_t point_index;
            double min_dist = line.min_distance(l2, &point_index);
            bool intersects = line.intersect(l2);
            bool ok = (/*intersects ||*/ min_dist < 10);
            if(ok)
              distances.push_back(min_dist);
            return ok;
          });

          auto it = min_element(distances.begin(), distances.end());
          int min = *it;

          transform(adjacent.begin(), adjacent.end(), back_inserter(adjacent_lines), [&](int index) -> Line<float>* { return &lines[index]; });

          vector<int> parallel = filter_lines(lines.begin(), lines.end(), [&line](Line<float>& l2, size_t) { return fabs((line.angle() - l2.angle()) * 180 / M_PI) < 3; });

          std::cout << "adjacent " << adjacent << endl;
          std::cout << "parallel " << parallel << endl;

          distances.clear();
          transform(adjacent_lines.begin(), adjacent_lines.end(), back_inserter(distances), [&line](Line<float>* l2) -> float { return line.min_distance(*l2); });
          transform(adjacent_lines.begin(), adjacent_lines.end(), back_inserter(line_ends), [&line](Line<float>* l2) -> LineEnd<float> {
            LineEnd<float> end;
            line.nearest_end(*l2, end);
            return end;
          });
          angleoffs.clear();
          transform(adjacent_lines.begin(), adjacent_lines.end(), back_inserter(angleoffs), [&line](Line<float>* l2) -> float { return line.angle_diff(*l2); });
          vector<int> angleoffs_i;

          transform(angleoffs.begin(), angleoffs.end(), back_inserter(angleoffs_i), [](const float ang) -> int { return int(ang * 180 / M_PI) % 180; });

          vector<cv::Point> centers;
          transform(adjacent_lines.begin(), adjacent_lines.end(), back_inserter(centers), [](Line<float>* line) -> cv::Point { return line->center(); });

          cout << "adjacent(" << i << ")" << adjacent << std::endl;
          std::cout << "distances(" << i << ")" << distances << endl;
          cout << "angleoffs(" << i << ")" << angleoffs_i << endl;

          int minIndex = distance(distances.begin(), it);
          adjacency_list.emplace(make_pair(i, adjacent));

          float index = (float)degrees / (binsize - 1);

          int angleIndex = (int)(index * double(histogram.size() - 1)) % histogram.size();
          histogram[angleIndex] += length - (length - range);
          angles[angleIndex] = double(angleIndex) / (histogram.size() - 1) * (M_PI);
          filteredLines.push_back(line);
        }
        Matrix<double> m = Matrix<double>::identity();
        Matrix<double> s = Matrix<double>::scale(3);
        Matrix<double> r = Matrix<double>::rotation(M_PI);
        Matrix<double> t = Matrix<double>::translation(120, -60);
        Matrix<double> mult;

        mult = t * r * s;

        cout << "matrix x " << to_string(mult) << endl;
        cout << "matrix init " << to_string(m) << endl;
        cout << "matrix scale " << to_string(s) << endl;
        cout << "matrix rotate " << to_string(r) << endl;
        cout << "matrix translate " << to_string(t) << endl;

        cv::Point2f p(100, 50);
        std::vector<cv::Point2f> pl = {p};
        std::vector<cv::Point2f> ol;

        mult.transform_points(pl.cbegin(), pl.cend(), std::back_inserter(ol));
        cout << "transformed point: " << ol << endl;
      }

      cout << "histogram:";

      for_each(histogram.begin(), histogram.end(), [](const int count) { cout << ' ' << count; });
      cout << endl;

      cout << "angles:";
      for_each(angles.begin(), angles.end(), [](const float a) { cout << ' ' << (int)(a * 180 / M_PI); });
      cout << endl;

      draw_all_lines(imgGrayscale, filteredLines, [&](int index, size_t len) -> int { return lines[index].length() * 10; });

      cout << "Num lines: " << lines.size() << endl;
      cout << "Num filteredLines: " << filteredLines.size() << endl;

      //   draw_all_lines(imgOriginal, filteredLines);

      //  cout << contourStr.str() << endl;
      /*
            for(size_t i = 0; i < contours.size(); ++i) {
              vector<cv::Point> c = contours[i];
              cv::Vec4i h = hier[i];
      }*/

      std::ostringstream filename;
      filename << "contour.svg.tmp";

      // filename << "contour-" << ++count << ".svg";

      // filter_contours(contours2);
      export_svg<cv::Point2f>(contours2, filename.str());

      //      unlink("contour.svg");
      rename("contour.svg.tmp", "contour.svg");

      vector<PointVec> squares;

      find_rectangles(contours, squares);

      // Draw the circles detected
      for(size_t i = 0; i < circles.size(); i++) {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle(imgOriginal, center, 3, cv::Scalar(255, 0, 0), -1, 8, 0);     // circle center
        cv::circle(imgOriginal, center, radius, cv::Scalar(255, 0, 0), 3, 8, 0); // circle outline
        cout << "center : " << center << "\nradius : " << radius << endl;
      }

      //   draw_polylines<cv::Point>(imgOriginal, contours, cv::Scalar(0, 255, 0));

      vector<PointVec> approxim;

      transform(contours2.begin(), contours2.end(), back_inserter(approxim), [](const Point2fVec& p) -> PointVec { return transform_points<float, int>(p); });

      // [](const Point2fVec &p) -> cv::Point { return cv::Point(p.x, p.y); });

      for_each(approxim.begin(), approxim.end(), [&](const PointVec& c) {
        const double length = cv::arcLength(c, false);
        const double area = cv::contourArea(c, false);
        cv::Rect rect = cv::boundingRect(c);
        vector<PointVec> list;
        list.push_back(c);
        // cv::drawContours(imgOriginal, list, -1, cv::Scalar(255, 255, 0), 1);
      });

      draw_all_contours(imgOriginal, contours);

      // CV_WINDOW_AUTOSIZE is the default
      cv::imshow("imgOriginal", imgOriginal);

      cv::imshow("imgGrayscale", imgGrayscale); //

      // show windows
      // cv::imshow("imgGrayscale", imgBlurred); //

      // cv::createTrackbar("Thre", "demoProc", &thresholdValue, 255, &trackbar);

      charCheckForEscKey = cv::waitKey(100); // delay (in ms) and get key press, if any
    }
  } // end while

  return (0);
}
