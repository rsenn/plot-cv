#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgcodecs.hpp>
#include <unistd.h>

#include <type_traits>
#include <iostream>
#include <algorithm>
#include <list>
#include <functional>
#include <unordered_map>
#include <map>
#include <filesystem>
#include <sys/stat.h>
#include <unistd.h>

#include "color.h"
#include "data.h"
#include "geometry.h"
#include "js.h"
#include "jsbindings.h"
#include "line.h"
#include "matrix.h"
#include "plot-cv.h"
#include "polygon.h"
#include "psimpl.h"

int thresh = 10;
int max_thresh = 255;
double max_svg_width = 1200; // pixels
double max_svg_height = 900; // pixels
int thresholdValue = 155;
bool showDiagnostics = false;
double epsilon = 3;
const int max_frames = 100000;
jsrt js;

image_type imgRaw, imgVector, imgOriginal, imgTemp, imgGrayscale, imgBlurred, imgCanny,
    im_oc; // Canny edge image

image_type *mptr = nullptr, *dptr = nullptr;

std::ofstream logfile("plot-cv.log", std::ios_base::out | std::ios_base::ate);

std::string
make_filename(const std::string& name, int count, const std::string& ext, const std::string& dir) {
  const int pad = 5;
  char buf[40];
  std::ostringstream filename;
  time_t now = time(NULL);
  struct tm lt;
  struct timeval tv;
  uint64_t msecs;
  gettimeofday(&tv, NULL);
  msecs = (tv.tv_usec / 1000) % 1000;

  strftime(buf, sizeof(buf), "%Y%m%d-%H%M%S", localtime_r(&now, &lt));

  filename << dir << "/" << name << "-" << buf << "." << std::setfill('0') << std::setw(3) << msecs
           << "-" << std::setfill('0') << std::setw(pad) << count << "." << ext;
  return filename.str();
}

// Function that calculates the absolute value

template<class T, class O>
void
out_point(O& os, const cv::Point_<T>& pt) {
  os << pt.x;
  os << ',';
  os << pt.y;
}

template<class O>
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
 * @brief      ������utput point list
 * @return     { description_of_the_return_value }
 */
template<class O>
void
out_points(O& os, const point_vector& pl) {
  size_t i, n = pl.size();
  for(i = 0; i < n; ++i) {
    if(i > 0)
      os << ' ';
    out_point(os, pl[i]);
  }
}

/**
 * @brief      finds the largest contour
 *
 * @param[in]  contours_un     The contours list
 * @param      bigContour  Receives largest contour
 *
 * @return     Index of largest Contour
 */
int
get_largest_contour(contour2f_vector contours_un, point2f_vector& bigContour) {
  double maxArea = 0.0;
  int largestContour = -1;
  for(size_t i = 0; i < contours_un.size(); i++) {
    double area = contourArea(contours_un[i]);
    if(showDiagnostics) {
      logfile << "Area: " + to_string(area) + "Index: " + to_string(i) << std::endl;
    }
    if(area > maxArea) {
      maxArea = area;
      largestContour = i;
    }
  }
  bigContour = contours_un.at(largestContour);
  return largestContour;
}

void
svg_draw_polyline(svg::Document& doc,
                  const point2f_vector& contour_arg,
                  std::function<svg::Color(const point2f_vector&)> color_fn) {
  svg::Polyline polyline(svg::Stroke(1, color_fn(contour_arg)));

  for(size_t i = 0; i < contour_arg.size(); i++) {
    svg::Point tmp = svg::Point(contour_arg.at(i).x, contour_arg.at(i).y);
    polyline << tmp;
  }

  doc << polyline;
}

template<class PointType>
void
svg_export_file(const typename vector_vector_traits<PointType>::type& contours,
                std::string output_file) {

  logfile << "Saving '" << output_file << "'" << std::endl;

  svg::Dimensions dimensions(max_svg_width, max_svg_height);
  svg::Document doc(output_file, svg::Layout(dimensions, svg::Layout::TopLeft));
  svg::LineChart chart(5.0);
  std::vector<double> areas;
  std::transform(contours.begin(),
                 contours.end(),
                 std::back_inserter(areas),
                 [](const std::vector<PointType>& contour) -> double {
                   return cv::contourArea(transform_points<int>(contour));
                 });
  const auto& it = std::max_element(areas.begin(), areas.end());
  double max_area = 0;
  if(it != areas.end()) {
    max_area = *it;
  }

  logfile << "Max area: " << max_area << std::endl;

  const auto& cfn = [&](const std::vector<PointType>& contour) -> svg::Color {
    const double area = cv::contourArea(contour);
    return from_scalar(hsv_to_rgb(area / max_area * 360, 1, 1));
  };

  std::for_each(contours.begin(),
                contours.end(),
                std::bind(&svg_draw_polyline, std::ref(doc), std::placeholders::_1, cfn));
  //  svg_draw_polyline(doc, contour_arg, svg::Color(255, 0, 0));

  doc.save();

  logfile << "EXPORTED SVG" << std::endl;
}

/** @function main */

void
apply_clahe(const image_type& bgr_image, image_type& image_clahe) {
  image_type lab_image;
  cv::cvtColor(bgr_image, lab_image, CV_BGR2Lab);

  // Extract the L channel
  std::vector<image_type> lab_planes(3);
  cv::split(lab_image, lab_planes); // now we have the L image in lab_planes[0]

  // apply the CLAHE algorithm to the L channel
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(4);
  image_type dst;
  clahe->apply(lab_planes[0], dst);

  // Merge the the color planes back into an Lab image
  dst.copyTo(lab_planes[0]);
  cv::merge(lab_planes, lab_image);

  // convert back to RGB
  cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);
}

/**
 *  \brief Automatic brightness and contrast optimization with optional
 * histogram clipping \param [in]src Input image GRAY or BGR or BGRA \param
 * [out]dst Destination image \param clipHistPercent cut wings of histogram at
 * given percent tipical=>1, 0=>Disabled \note In case of BGRA image, we won't
 * touch the transparency
 */
void
brightness_and_contrast_auto(const image_type& src, image_type& dst, float clipHistPercent = 0) {

  CV_Assert(clipHistPercent >= 0);
  CV_Assert((src.type() == CV_8UC1) || (src.type() == CV_8UC3) || (src.type() == CV_8UC4));

  int histSize = 256;
  float alpha, beta;
  double minGray = 0, maxGray = 0;

  // to calculate grayscale histogram
  image_type gray;
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
    image_type hist; // the grayscale histogram

    float range[] = {0, 256};
    const float* histRange = {range};
    bool uniform = true;
    bool accumulate = false;
    calcHist(&gray, 1, 0, image_type(), hist, 1, &histSize, &histRange, uniform, accumulate);

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

image_type
image_to_binary(image_type start) {
  image_type gray_image, thresh_image;
  cvtColor(start, gray_image, CV_BGR2GRAY);
  cv::threshold(gray_image, thresh_image, 100, 255, cv::THRESH_BINARY);

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
contour_vector
get_contours(image_type start, std::vector<cv::Vec4i>& hierarchy, int flag = CV_RETR_TREE) {
  image_type dst = image_type::zeros(start.rows, start.cols, CV_8UC3);
  contour_vector contours;
  start = start > 1;
  cv::findContours(start, contours, hierarchy, flag, CV_CHAIN_APPROX_SIMPLE);
  return contours;
}

template<class InputType>
point_vector
to_point_vec(const std::vector<InputType>& v) {
  point_vector ret;
  std::for_each(v.begin(), v.end(), [&ret](const InputType& pt) {
    ret.push_back(point2i_type(pt.x, pt.y));
  });
  return ret;
}

void
find_rectangles(const contour_vector& contours, contour_vector& squares) {

  // test each contour
  for(size_t i = 0; i < contours.size(); i++) {
    point_vector approx;
    double arcLen = cv::arcLength(image_type(contours[i]), true);

    // approximate contour with accuracy proportional
    // to the contour perimeter
    cv::approxPolyDP(image_type(contours[i]), approx, arcLen * 0.02, true);

    // square contours should have 4 vertices after approximation
    // relatively large area (to filter out noisy contours)
    // and be convex.
    // Note: absolute value of an area is used because
    // area may be positive or negative - in accordance with the
    // contour orientation
    if(arcLen > 80 || fabs(cv::contourArea(image_type(approx))) >
                          200 /*|| cv::isContourConvex(image_type(approx))*/) {
      /*      double maxCosine = 0;

            for(int j = 2; j < 5; j++) {
              // find the maximum cosine of the angle between joint edges
              double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j
         - 1])); maxCosine = MAX(maxCosine, cosine);
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

template<class PointT>
static void
draw_polylines(image_type& image,
               const typename vector_vector_traits<PointT>::type& polylines,
               const color_type& color = color_type(0, 255, 0)) {

  cv::polylines(image, polylines, true, color, 2, cv::LINE_AA);
}

void
invert_color(image_type& img) {
  for(int i = 0; i < img.rows; i++)
    for(int j = 0; j < img.cols; j++) img.at<uchar>(i, j) = 255 - img.at<uchar>(i, j);
}

std::vector<cv::Vec4i>
hough_lines(image_type& imgToMap) {
  image_type imgToMapProc;
  cvtColor(imgToMap, imgToMapProc, CV_BGR2GRAY);
  cv::threshold(imgToMapProc, imgToMapProc, thresholdValue, 255, 0);
  std::vector<cv::Vec4i> lines;
  invert_color(imgToMapProc);
  cv::HoughLinesP(imgToMapProc, lines, 1, CV_PI / 180, 30, 30, 10);

  return lines;
}

void
draw_lines(image_type& target, const std::vector<cv::Vec4i>& lines) {
  for(size_t i = 0; i < lines.size(); i++)
    cv::line(target,
             point2i_type(lines[i][0], lines[i][1]),
             point2i_type(lines[i][2], lines[i][3]),
             color_type(0, 255, 255),
             1);
}

void
corner_harris_detection(image_type& src, image_type& src_gray) {
  int thresh = 200;
  int max_thresh = 255;
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;
  image_type dst = image_type::zeros(src.size(), CV_32FC1);
  image_type gray = image_type::zeros(src.size(), CV_32FC1);
  src_gray.convertTo(gray, CV_32FC1);
  cv::cornerHarris(gray, dst, blockSize, apertureSize, k);
  image_type dst_norm, dst_norm_scaled;
  cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, image_type());
  cv::convertScaleAbs(dst_norm, dst_norm_scaled);
  for(int i = 0; i < dst_norm.rows; i++) {
    for(int j = 0; j < dst_norm.cols; j++) {
      if((int)dst_norm.at<float>(i, j) > thresh) {
        cv::circle(dst_norm_scaled, point2i_type(j, i), 5, color_type(1), 2, 8, 0);
      }
    }
  }
  //  display_image(dst_norm_scaled);
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
write_image(image_type img) {
  static int count = 0;

  std::filesystem::create_directories("tmp");
  std::string file = make_filename("frame", ((++count) % max_frames), "png");

  cv::imwrite(cv::String(file), img);
}

void
draw_all_contours(image_type& out, contour_vector& contours) {

  for(size_t i = 0; i < contours.size(); i++) {
    const color_type color = hsv_to_rgb((i * 360 * 10 / contours.size()) % 360, 1.0, 1.0);
    auto contour = simplify_polyline(contours[i]);
    contours[i] = contour;

    const double area = cv::contourArea(contours[i], false);
    if(area < 1)
      continue;

    cv::drawContours(out, contours, i, color, 1, cv::LINE_AA);
  }
}

template<class Container>
void
draw_all_lines(image_type& out,
               const Container& lines,
               const std::function<int(int, size_t)>& hue = [](int index, size_t len) -> int {
                 return (index * 360 * 10 / len) % 360;
               }) {
  for(typename Container::const_iterator it = lines.begin(); it != lines.end(); it++) {
    size_t i = std::distance(lines.begin(), it);
    const color_type color = hsv_to_rgb(hue(i, lines.size()), 1.0, 1.0);

    const double len = it->length();
    if(len < 8)
      continue;

    cv::line(out, point2i_type(it->a), point2i_type(it->b), color, 1, cv::LINE_AA);
  }
}

image_type
get_alpha_channel(image_type m) {
  image_type mask;
  cv::cvtColor(m, mask, CV_RGB2GRAY);
  mask = (mask == 0);
  cv::threshold(mask, mask, 100, 255, cv::THRESH_BINARY);
  return mask;
}

void
display_image(image_type* m) {
  if(m == nullptr)
    return;
  image_type out(cvSize(m->cols, m->rows), m->type());
  cv::rectangle(*dptr,
                point2i_type(50, 50),
                point2i_type(out.cols - 50, out.rows - 50),
                cv::Scalar(255, 0, 255, 255));
  cv::rectangle(*dptr,
                point2i_type(51, 51),
                point2i_type(out.cols - 51, out.rows - 51),
                cv::Scalar(255, 0, 255, 255));
  image_type mask = image_type::zeros(m->size(), CV_8UC1);
  image_type alpha = get_alpha_channel(*dptr);
  dptr->copyTo(out);
  cv::bitwise_and(*m, *m, out, alpha);
  cv::imshow("img", out);
}

JSValue
vec4i_to_js(const cv::Vec4i& v) {
  return pointer_to_js(js, (const int*)&v[0], (int)4);
}

template<class P>
JSValue
points_to_js(const std::vector<P>& v) {
  std::function<JSValue(const P&)> fn(
      [](const P& point) -> JSValue { return js.create_point(point.x, point.y); });
  return vector_to_js(js, v, fn);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
int
main(int argc, char* argv[]) {
  using std::array;
  using std::back_inserter;
  using std::distance;
  using std::endl;
  using std::for_each;
  using std::iterator_traits;
  using std::transform;
  using std::vector;
  unsigned int show_image = 0, ret;
  int32_t newmt, mt = -1;
  JSValue processFn;

  js.init(argc, argv);

  JSValue* fn = js.get_function("drawContour");
  jsrt::value glob = js.global_object();

  // ret = js.eval_file("lib.js", 1);

  auto check_eval = [&newmt, &mt, &processFn, &glob]() -> int {
    int ret = -1;
    newmt = get_mtime("js/test.js");

    /*    std::cerr << "test.js mtime new=" << newmt << " old=" << mt
                  << " diff=" << (newmt - mt) << std::endl; */
    if(newmt > mt) {
      mt = newmt;
      std::cerr << "js/test.js changed, reloading..." << std::endl;

      ret = js.eval_file("js/test.js");
      processFn = js.get_property(glob, "process");
    }
    return ret;
  };

  check_eval();

  js.add_function("drawContour", &js_draw_contour, 2);
  js.add_function("drawLine", &js_draw_line, 2);
  js.add_function("drawRect", &js_draw_rect, 2);
  js.add_function("drawPolygon", &js_draw_polygon, 2);
  js.add_function("drawCircle", &js_draw_circle, 2);
  /*
    std::cerr << "property names: " << js.property_names(glob) << std::endl;
    std::cerr << "'console' property names: " << js.property_names(js.get_property(glob, "console"))
              << std::endl;
  */
  JSValue testFn = js.get_property(glob, "test");
  JSValue drawContourFn = js.get_property(js.global_object(), "drawContour");
  JSValue jsPoint = js.create_point(150, 100);

  std::cerr << "js.eval_file ret=" << ret << " globalObj=" << js.to_str(glob)
            << " testFn=" << js.to_str(testFn) << " processFn="
            << js.to_str(processFn)
            /* << " property_names=" << js.property_names(jsPoint, false, true)*/
            << std::endl;
  if(ret < 0)
    return ret;

  int camID = (argc > 1 && isdigit(argv[1][0])) ? strtol(argv[1], nullptr, 10) : -1;
  int count = 0;

  std::string filename;

  if(camID == -1 && argc > 1)
    filename = argv[1];

  cv::VideoCapture capWebcam(camID);
  image_type imgInput;

  if(camID >= 0) {
    // capWebcam.open((int)camID, (int)cv::CAP_V4L2); // declare a VideoCapture
    // object and associate to webcam, 0 => use 1st webcam

    if(capWebcam.isOpened() == false) { // check if VideoCapture object was
                                        // associated to webcam successfully
      logfile << "error: capWebcam not accessed successfully\n\n"; // if not, print
                                                                   // error message
                                                                   // to std out
      getchar();  // may have to modify this line if not using Windows
      return (0); // and exit program
    }
  } else {
    imgInput = cv::imread(filename.empty() ? "input.png" : filename);
  }

  char keycode = 0;
  int levels = 3;
  int eps = 8;
  int blur = 4;

  cv::namedWindow("img", CV_WINDOW_AUTOSIZE);

  // cv::namedWindow("imgGrayscale", CV_WINDOW_AUTOSIZE);
  cv::createTrackbar("epsilon", "contours", &eps, 7, trackbar, (void*)"eps");
  cv::createTrackbar("blur", "contours", &blur, 7, trackbar, (void*)"blur");

  mptr = &imgOriginal;

  while(keycode != 27) { // until the Esc key is pressed or webcam connection is lost

    switch(keycode) {
      case 27: return 0;
      case 'c':
      case 'C': show_image = CANNY; break;
      case 'p':
      case 'P': show_image = OPEN_CLOSE; break;
      case 'o':
      case 'O': show_image = ORIGINAL; break;
      case 'g':
      case 'G': show_image = GRAYSCALE; break;
      case 83:
      case -106: show_image--; break;
      case 81:
      case -104: show_image++; break;
      case -1: break;
      default: std::cerr << "Unknown keycode: '" << (int)(char)keycode << "'" << std::endl; break;
    }
    show_image %= 4;

    std::cerr << "Show image: " << show_image << std::endl;

    switch(show_image) {
      case OPEN_CLOSE: mptr = &im_oc; break;
      case ORIGINAL: mptr = &imgOriginal; break;
      case GRAYSCALE: mptr = &imgGrayscale; break;
      case CANNY:
      default: mptr = &imgCanny; break;
    }

    bool blnFrameReadSuccessfully = false;
    if(capWebcam.isOpened()) {
      blnFrameReadSuccessfully = capWebcam.read(imgRaw); // get next frame
                                                         //
    } else {
      imgInput.copyTo(imgRaw);
      blnFrameReadSuccessfully = imgRaw.cols > 0 && imgRaw.rows > 0;
    }

    if(!blnFrameReadSuccessfully || imgRaw.empty()) {   // if frame not read successfully
      logfile << "error: frame not read from webcam\n"; // print error message
                                                        // to std out
      break;                                            // and jump out of while loop
    }

    image_type imgOutput;

    imgRaw.copyTo(imgOutput);
    imgRaw.copyTo(imgOriginal);

    write_image(imgOutput);

    logfile << "got frame" << std::endl;
    // cv::normalize(imgRaw,imgOriginal,0,255,cv::NORM_L1);
    //    imgRaw.copyTo(imgOriginal);

    if(dptr == nullptr) {
      imgVector = image_type::zeros(cvSize(imgOriginal.cols, imgOriginal.rows), imgOriginal.type());
      dptr = &imgVector;
    }

    image_type frameLab, frameLabCn[3];
    imgOriginal.copyTo(frameLab);
    cv::cvtColor(frameLab, frameLab, cv::COLOR_BGR2Lab);
    cv::split(frameLab, frameLabCn);
    frameLabCn[0].copyTo(imgGrayscale);

    // cvtColor(imgOriginal, imgGrayscale, CV_BGR2GRAY); // convert to grayscale
    vector<cv::Vec3f> circles;

    // corner_harris_detection(imgOriginal, imgGrayscale);

    /// Apply Histogram Equalization
    // cv::equalizeHist(imgGrayscale, imgGrayscale);

    cv::GaussianBlur(imgGrayscale, imgBlurred, cv::Size(5, 5), 1.75, 1.75);

    cv::Canny(imgBlurred, imgCanny, thresh, thresh * 2, 3);

    // open and close to highlight objects
    image_type strel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(19, 19));

    imgCanny.copyTo(im_oc);
    cv::cvtColor(im_oc, im_oc, cv::COLOR_GRAY2BGR);

    //   cv::morphologyEx(im_oc, im_oc, cv::MORPH_OPEN,
    //   cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)));
    cv::morphologyEx(im_oc,
                     im_oc,
                     cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2)));
    //    cv::morphologyEx(im_oc, im_oc, cv::MORPH_CLOSE, strel);
    /*
        if(show_image == OPEN_CLOSE)
          display_image(im_oc);*/
    display_image(mptr);

    cv::cvtColor(imgGrayscale, imgGrayscale, cv::COLOR_GRAY2BGR);

    //  apply_clahe(imgOriginal, imgOriginal);XY
    {

      vector<point2f_vector> contours2;
      vector<cv::Vec4i> hier;
      vector<point_vector> contours = get_contours(imgCanny, hier, CV_RETR_TREE);

      imgCanny = color_type::all(255) - imgCanny;

      // imgCanny.convertTo(imgCanny, CV_8UC3);
      cv::cvtColor(imgCanny, imgCanny, cv::COLOR_GRAY2BGR);

      // cvtColor(imgOriginal, imgGrayscale, CV_GRAY);

      cv::drawContours(imgCanny, contours, -1, color_type(0, 0, 255), 1, cv::LINE_AA);
      /*
            if(show_image == CANNY)
              /*/
      display_image(mptr);

      if(contours.empty()) {
        logfile << "No contours" << std::endl;
        keycode = cv::waitKey(100);
        continue;
      }

      std::ostringstream contourStr;
      double maxArea = 0;

      vector<line_type> lines;
      std::map<int, ref_list> adjacency_list;

      const auto& contourDepth = [&hier](int i) {
        size_t depth = 0;
        while(i != -1) {
          i = hier[i][3];
          ++depth;
        };
        return depth;
      };

      int i = 0;
      for(contour_vector::const_iterator it = contours.cbegin(); it != contours.cend(); ++i, ++it) {
        const vector<point2i_type>& a = *it;
        int depth = contourDepth(i);
      }
      i = 0;
      for(contour_vector::const_iterator it = contours.cbegin(); it != contours.cend(); ++i, ++it) {
        const vector<point2i_type>& a = *it;

        if(a.size() >= 3) {

          point2f_vector c;
          cv::approxPolyDP(a, c, 8, true);
          double area = cv::contourArea(c);
          int depth = contourDepth(i);
          if(area > maxArea)
            maxArea = area;
          contours2.push_back(c);

          if(contourStr.str().size())
            contourStr << "\n";
          out_points(contourStr, a);
          /*    logfile << "hier[i] = {" << hier[i][0] << ", " << hier[i][1] <<
             ", " << hier[i][2] << ", " << hier[i][3] << ", "
                        << "} " << std::endl;
              logfile << "contourDepth(i) = " << depth << std::endl;
    */
          cv::drawContours(
              imgGrayscale, contours, i, hsv_to_rgb(depth * 10, 1.0, 1.0), 2, cv::LINE_AA);
        }
      }

      display_image(mptr);

      if(maxArea == 0) {
        keycode = cv::waitKey(16);
        logfile << "No contour area" << std::endl;
        continue;
      }

      for_each(contours2.begin(), contours2.end(), [&lines](const vector<point2f_type>& a) {
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

      logfile << "Num contours: " << contours.size() << std::endl;

      std::list<Line<float>> filteredLines;
      vector<bool> takenLines;
      vector<float> lineLengths;

      std::array<int, 8> histogram = {0, 0, 0, 0, 0, 0, 0, 0};
      std::array<float, 8> angles = {0, 0, 0, 0, 0, 0, 0, 0};

      vector<cv::Vec4i> linesHier;
      linesHier.resize(lines.size());
      takenLines.resize(lines.size());

      //     sort(lines.begin(), lines.end());

      transform(lines.begin(),
                lines.end(),
                back_inserter(lineLengths),
                [&](Line<float>& l) -> float { return l.length(); });

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

          vector<int> adjacent =
              filter_lines(lines.begin(), lines.end(), [&](Line<float>& l2, size_t index) -> bool {
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

          std::transform(adjacent.begin(),
                         adjacent.end(),
                         back_inserter(adjacent_lines),
                         [&](int index) -> Line<float>* { return &lines[index]; });

          vector<int> parallel =
              filter_lines(lines.begin(), lines.end(), [&line](Line<float>& l2, size_t) {
                return fabs((line.angle() - l2.angle()) * 180 / M_PI) < 3;
              });
          /*
                    logfile << "adjacent " << adjacent << std::endl;
                    logfile << "parallel " << parallel << std::endl;
          */
          distances.clear();
          std::transform(adjacent_lines.begin(),
                         adjacent_lines.end(),
                         back_inserter(distances),
                         [&line](Line<float>* l2) -> float { return line.min_distance(*l2); });
          std::transform(adjacent_lines.begin(),
                         adjacent_lines.end(),
                         back_inserter(line_ends),
                         [&line](Line<float>* l2) -> LineEnd<float> {
                           LineEnd<float> end;
                           line.nearest_end(*l2, end);
                           return end;
                         });
          angleoffs.clear();
          std::transform(adjacent_lines.begin(),
                         adjacent_lines.end(),
                         back_inserter(angleoffs),
                         [&line](Line<float>* l2) -> float { return line.angle_diff(*l2); });
          std::vector<int> angleoffs_i;

          std::transform(angleoffs.begin(),
                         angleoffs.end(),
                         back_inserter(angleoffs_i),
                         [](const float ang) -> int { return int(ang * 180 / M_PI) % 180; });

          point_vector centers;
          std::transform(adjacent_lines.begin(),
                         adjacent_lines.end(),
                         back_inserter(centers),
                         [](Line<float>* line) -> point2i_type { return line->center(); });

          Matrix<double> rot = Matrix<double>::rotation(-line.angle());

          Line<float> l(line);
          logfile << "angle: " << (line.angle() * 180 / M_PI) << std::endl;
          logfile << "a: " << l.a << " b: " << l.b << std::endl;
          l.a = rot.transform_point(l.a);
          l.b = rot.transform_point(l.b);
          logfile << "a: " << l.a << " b: " << l.b << std::endl;
          /*
                    logfile << "adjacent(" << i << ")" << adjacent << std::endl;
                    logfile << "distances(" << i << ")" << distances << std::endl;
                    logfile << "angleoffs(" << i << ")" << angleoffs_i << std::endl;
          */
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
        Matrix<double> r = Matrix<double>::rotation(M_PI / 4, point2f_type(50, 50));
        Matrix<double> t = Matrix<double>::translation(120, -60);
        Matrix<double> mult;

        mult = t * r * s;

        logfile << "matrix x " << to_string(mult) << std::endl;
        logfile << "matrix init " << to_string(m) << std::endl;
        logfile << "matrix scale " << to_string(s) << std::endl;
        logfile << "matrix rotate " << to_string(r) << std::endl;
        logfile << "matrix translate " << to_string(t) << std::endl;

        point2f_type p(100, 50);
        point2f_vector pl = {p};
        point2f_vector ol;

        mult.transform_points(pl.cbegin(), pl.cend(), std::back_inserter(ol));
        logfile << "transformed point: " << ol << std::endl;
      }

      logfile << "histogram:";

      for_each(histogram.begin(), histogram.end(), [](const int count) {
        logfile << ' ' << count;
      });
      logfile << std::endl;

      logfile << "angles:";
      for_each(angles.begin(), angles.end(), [](const float a) {
        logfile << ' ' << (int)(a * 180 / M_PI);
      });
      logfile << std::endl;

      draw_all_lines(imgGrayscale, filteredLines, [&](int index, size_t len) -> int {
        return lines[index].length() * 10;
      });

      logfile << "Num lines: " << lines.size() << std::endl;
      logfile << "Num filteredLines: " << filteredLines.size() << std::endl;

      std::string svg = make_filename("contour", ++count, "svg");
      // filename << "contour-" << ++count << ".svg";

      svg_export_file<point2f_type>(contours2, svg);

      unlink("contour.svg");
      rename("contour.svg.tmp", "contour.svg");

      vector<point_vector> squares;

      {
        JSValue args[2] = {vector_to_js(js, contours, &points_to_js<point2i_type>),
                           vector_to_js(js, hier, &vec4i_to_js)};

        check_eval();

        auto before = std::chrono::high_resolution_clock::now();

        js.call(processFn, 2, args);
        auto after = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> fp_ms = after - before;
        auto int_ms = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
        std::chrono::duration<long, std::micro> int_usec = int_ms;

        std::cout << "f() took " << fp_ms.count() << " ms, "
                  << "or " << int_ms.count() << " whole milliseconds "
                  << "(which is " << int_usec.count() << " whole microseconds)" << std::endl;
      }

      {
        point2f_vector src = {point2f_type(50, 50),
                              point2f_type(100, 50),
                              point2f_type(100, 100),
                              point2f_type(50, 100)};

        point2f_vector dst = {point2f_type(100, 0),
                              point2f_type(150, 0),
                              point2f_type(150, 50),
                              point2f_type(100, 50)};

        image_type perspective = cv::getPerspectiveTransform(src, dst);

        std::cerr << "perspective:" << perspective << std::endl;
      }

      find_rectangles(contours, squares);

      // Draw the circles detected
      for(size_t i = 0; i < circles.size(); i++) {
        point2i_type center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle(imgOriginal, center, 3, color_type(255, 0, 0), -1, 8,
                   0); // circle center
        cv::circle(imgOriginal, center, radius, color_type(255, 0, 0), 3, 8,
                   0); // circle outline
        logfile << "center : " << center << "\nradius : " << radius << std::endl;
      }

      vector<point_vector> approxim;
      transform(contours2.begin(),
                contours2.end(),
                back_inserter(approxim),
                [](const point2f_vector& p) -> point_vector { return transform_points<int>(p); });

      for_each(approxim.begin(), approxim.end(), [&](const point_vector& c) {
        const double length = cv::arcLength(c, false);
        const double area = cv::contourArea(c, false);
        cv::Rect rect = cv::boundingRect(c);
        vector<point_vector> list;
        list.push_back(c);
        // cv::drawContours(imgOriginal, list, -1, color_type(255, 255, 0), 1);
      });

      //  draw_all_contours(imgOriginal, contours);
      display_image(mptr);

      /*
            if(show_image == ORIGINAL)
              display_image(imgOriginal);
            else if(show_image == GRAYSCALE)
              display_image(imgGrayscale);*/

      keycode = cv::waitKey(1); // delay (in ms) and get key press, if any
    }
  } // end while

  return (0);
}
