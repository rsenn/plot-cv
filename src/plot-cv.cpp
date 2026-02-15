#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/photo.hpp>
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
#include <ctime>
#include <sys/stat.h>
#include <unistd.h>
#include <iomanip> // std::setbase

#include "color.hpp"
#include "data.hpp"
#include "../qjs-opencv/include/geometry.hpp"
#include "js.hpp"
#include "../qjs-opencv/include/line.hpp"
#include "matrix.hpp"
#include "plot-cv.hpp"
#include "polygon.hpp"
#include "../qjs-opencv/include/psimpl.hpp"
#include "auto_canny.hpp"
#include "../qjs-opencv/include/jsbindings.hpp"
#include "../qjs-opencv/js_point_iterator.hpp"
#include "../qjs-opencv/js_contour.hpp"
#include "../qjs-opencv/js_rect.hpp"
#include "../qjs-opencv/js_size.hpp"
#include "../qjs-opencv/js_point.hpp"
#include "../qjs-opencv/js_line.hpp"
#include "../qjs-opencv/js_cv.hpp"
#include "../qjs-opencv/js_mat.hpp"

using std::string;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

typedef Line<float> line_type;

extern "C" {

// const char* image_names[] = {"CANNY", "ORIGINAL", "GRAYSCALE", "MORPHOLOGY"};
int num_iterations = 0;

int morphology_enable = 2;
// image_type* mptr = nullptr;

Timer timer;

int thresh = 10, thresh2 = 20, apertureSize = 3;
int max_thresh = 255;
double max_svg_width = 1200; // pixels
double max_svg_height = 900; // pixels
int thresholdValue = 155;
bool show_diagnostics = false;
double epsilon = 3;
const int max_frames = 100000;

config_values config = {.morphology_kernel_size = 1,
                        .morphology_operator = 0,
                        .blur_kernel_size = 2,
                        .blur_sigma = 175,
                        .blur_sigma_s = 6000,
                        .blur_sigma_r = 40,
                        .hough_rho = 99,
                        .hough_theta = 25,
                        .hough_threshold = 900,
                        .hough_minlinelen = 127,
                        .hough_maxlinegap = 199};

image_type imgRaw, imgVector, imgOriginal, imgTemp, imgGrayscale, imgBlurred, imgCanny,
    imgMorphology; // Canny edge image
}

int32_t newmt, mt = -1;
JSValue processFn;

jsrt::value
check_eval() {
  jsrt::value ret = js._undefined, global_obj;
  newmt = get_mtime("plot-cv.js");

  /* std::cerr << "plot-cv.js mtime new=" << newmt << " old=" << mt
     << " diff=" << (newmt - mt) << std::endl; */
  if(newmt > mt) {
    mt = newmt;
    if(show_diagnostics)
      std::cerr << "plot-cv.js changed, reloading..." << std::endl;

    ret = js.eval_file("plot-cv.js");
    global_obj = JS_GetGlobalObject(js.ctx); // js.global_object();
                                             //
    std::cerr << "global_obj: " << js.typestr(global_obj) << std::endl;

    processFn = js.get_property<const char*>(global_obj, "process");
    std::cerr << "processFn: " << js.typestr(processFn) << std::endl;
  }
  return ret;
};

string
make_filename(const string& name, int count, const string& ext, const string& dir) {
  const int pad = 5;
  char buf[40];
  std::ostringstream filename;
  time_t now = time(NULL);
  struct tm lt;
  struct timeval tv;
  uint64_t msecs;
  gettimeofday(&tv, NULL);
  msecs = (tv.tv_usec / 1000) % 1000;

  strftime(buf,
           sizeof(buf),
           "%Y%m%d-%H%M%S",
#ifdef _WIN32
           localtime(&now)
#else
           localtime_r(&now, &lt)
#endif
  );

  filename << dir << "/" << name << "-" << buf << "." << std::setfill('0') << std::setw(3)
           << msecs << "-" << std::setfill('0') << std::setw(pad) << count << "." << ext;
  return filename.str();
}

/**
 * @brief      Draws contours as SVG polyline
 *
 * @param      doc          The document
 * @param[in]  contour_arg  The contour argument
 * @param[in]  color_fn     The color function
 */
void
svg_draw_polyline(svg::Document& doc,
                  const point_vector<float>& contour_arg,
                  std::function<svg::Color(const point_vector<float>&)> color_fn) {
  svg::Polyline polyline(svg::Stroke(1, color_fn(contour_arg)));

  for(size_t i = 0; i < contour_arg.size(); i++) {
    svg::Point tmp = svg::Point(contour_arg.at(i).x, contour_arg.at(i).y);
    polyline << tmp;
  }

  doc << polyline;
}

/**
 * @brief      Perform a "Contrast Limited Adaptive Histogram Equalization"
 *
 * @param[in]  src   The source image
 * @param      dst   The destination image
 */
void
apply_clahe(const image_type& src, image_type& dst) {
  image_type lab_image;
  cv::cvtColor(src, lab_image, CV_BGR2Lab);

  // Extract the L channel
  std::vector<image_type> lab_planes(3);
  cv::split(lab_image, lab_planes); // now we have the L image in lab_planes[0]

  // apply the CLAHE algorithm to the L channel
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(4);
  image_type out;
  clahe->apply(lab_planes[0], out);

  // Merge the the color planes back into an Lab image
  out.copyTo(lab_planes[0]);
  cv::merge(lab_planes, lab_image);

  // convert back to RGB
  cv::cvtColor(lab_image, dst, CV_Lab2BGR);
}

/**
 *  \brief Automatic brightness and contrast optimization with optional
 * histogram clipping \param [in]src Input image GRAY or BGR or BGRA \param
 * [out]dst Destination image \param clipHistPercent cut wings of histogram at
 * given percent tipical=>1, 0=>Disabled \note In case of BGRA image, we won't
 * touch the transparency
 */
void
brightness_and_contrast_auto(const image_type& src,
                             image_type& dst,
                             float clipHistPercent = 0) {

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
    while(accumulator[minGray] < clipHistPercent)
      minGray++;

    // locate right cut
    maxGray = histSize - 1;
    while(accumulator[maxGray] >= (max - clipHistPercent))
      maxGray--;
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

/**
 * @brief      Converts image to monochrome
 *
 * @param[in]  start  The start
 *
 * @return     The image type.
 */
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
contour_vector<int>
get_contours(image_type src, std::vector<cv::Vec4i>& hierarchy, int flag) {
  image_type dst = image_type::zeros(src.rows, src.cols, CV_8UC3);
  contour_vector<int> contours;
  src = src > 1;
  cv::findContours(src, contours, hierarchy, flag, CV_CHAIN_APPROX_SIMPLE);
  return contours;
}

template<class InputIterator>
point_vector<int>
to_point_vec(InputIterator start, InputIterator end) {
  point_vector<int> ret;
  std::for_each(start, end, [&ret](const typename InputIterator::value_type& pt) {
    ret.push_back(point_type<int>(pt.x, pt.y));
  });
  return ret;
}

/*
template<class Container>
point_vector<int>
to_point_vec(const Container& c) {
  return to_point_vec(c.cbegin(), c.cend());
}

*/

void
find_rectangles(const contour_vector<int>& contours, contour_vector<int>& squares) {

  // test each contour
  for(size_t i = 0; i < contours.size(); i++) {
    point_vector<int> approx;
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

template<class T>
static void
draw_polylines(image_type& image,
               const typename contour_list<T>::type& polylines,
               const color_type& color = color_type(0, 255, 0)) {

  cv::polylines(image, polylines, true, color, 2, cv::LINE_AA);
}

void
invert_color(image_type& img) {
  for(int i = 0; i < img.rows; i++)
    for(int j = 0; j < img.cols; j++)
      img.at<uchar>(i, j) = 255 - img.at<uchar>(i, j);
}

/*
void
hough_lines(image_type& img, std::vector<point_vector<int>>& ret) {

  std::vector<cv::Vec2f> lines;

  invert_color(img);
  cv::HoughLines(img, lines, 1, CV_PI / 180, 30, 30, 10);

  std::for_each(lines.cbegin(), lines.cend(), [&ret](const cv::Vec2f& v) {
    float rho = v[0], theta = v[1];
    point_type<int> pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a * rho, y0 = b * rho;
    point_vector<int> l;
    pt1.x = cvRound(x0 + 1000 * (-b));
    pt1.y = cvRound(y0 + 1000 * (a));
    pt2.x = cvRound(x0 - 1000 * (-b));
    pt2.y = cvRound(y0 - 1000 * (a));

    l.emplace_back(pt1.x, pt1.y);
    l.emplace_back(pt2.x, pt2.y);

    ret.push_back(l);
  });
}*/

void
hough_lines(image_type& img, std::vector<cv::Vec4i>& out) {
  // invert_color(img);
  cv::HoughLinesP(img,
                  out,
                  (double)(config.hough_rho + 1) / 10.0,
                  CV_PI / (double)(config.hough_theta + 1),
                  (double)(config.hough_threshold + 1) / 10.0,
                  (double)(config.hough_minlinelen + 1) / 10,
                  (double)(config.hough_maxlinegap + 1) / 10);
}

void
hough_lines(image_type& img, const std::function<void(int, int, int, int)>& fn) {

  std::vector<cv::Vec4i> lines;

  hough_lines(img, lines);
  std::for_each(lines.cbegin(), lines.cend(), [fn](const cv::Vec4i& vec) {
    fn(vec[0], vec[1], vec[2], vec[3]);
  });
}

template<class InputIterator>
void
draw_lines(image_type& target,
           InputIterator start,
           InputIterator end,
           const cv::Scalar& color,
           int thickness = 1,
           int lineType = cv::LINE_8) {

  std::for_each(start, end, [target, color, thickness, lineType](const Line<int>& line) {
    cv::line(target, line.a, line.b, color, thickness, lineType);
  });
}

void
draw_lines(image_type& target,
           std::vector<cv::Vec4i>::const_iterator start,
           std::vector<cv::Vec4i>::const_iterator end,
           const cv::Scalar& color,
           int thickness = 1,
           int lineType = cv::LINE_8) {

  std::for_each(start, end, [target, color, thickness, lineType](const cv::Vec4i& vec) {
    cv::line(target,
             point_type<int>(vec[0], vec[1]),
             point_type<int>(vec[2], vec[3]),
             color,
             thickness,
             lineType);
  });
}

void
draw_lines(image_type& target,
           const std::vector<Line<int>>& lines,
           const cv::Scalar& color,
           int thickness = 1,
           int lineType = cv::LINE_8) {
  draw_lines(target, lines.cbegin(), lines.cend(), color, thickness, lineType);
}

void
draw_lines(image_type& target,
           const std::vector<cv::Vec4i>& lines,
           const cv::Scalar& color,
           int thickness = 1,
           int lineType = cv::LINE_8) {
  return draw_lines(target, lines.cbegin(), lines.cend(), color, thickness, lineType);
}

// https://stackoverflow.com/questions/6555629/algorithm-to-detect-corners-of-paper-sheet-in-photo
void
contour_detect(const image_type& input, image_type& drawing) {
  image_type mat;
  cv::cvtColor(input, mat, CV_BGR2GRAY);
  cv::GaussianBlur(mat, mat, cv::Size(3, 3), 0);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, point_type<int>(9, 9));
  cv::Mat dilated;
  cv::dilate(mat, dilated, kernel);

  cv::Mat edges;
  cv::Canny(dilated, edges, 84, 3);

  std::vector<cv::Vec4i> lines;
  lines.clear();
  cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 25);
  std::vector<cv::Vec4i>::iterator it = lines.begin();
  for(; it != lines.end(); ++it) {
    cv::Vec4i l = *it;
    cv::line(edges,
             point_type<int>(l[0], l[1]),
             point_type<int>(l[2], l[3]),
             cv::Scalar(255, 0, 0),
             2,
             8);
  }
  std::vector<std::vector<point_type<int>>> contours;
  cv::findContours(edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);
  std::vector<std::vector<point_type<int>>> contoursCleaned;
  for(int i = 0; i < contours.size(); i++) {
    if(cv::arcLength(contours[i], false) > 100)
      contoursCleaned.push_back(contours[i]);
  }
  std::vector<std::vector<point_type<int>>> contoursArea;

  for(int i = 0; i < contoursCleaned.size(); i++) {
    if(cv::contourArea(contoursCleaned[i]) > 10000) {
      contoursArea.push_back(contoursCleaned[i]);
    }
  }
  std::vector<std::vector<point_type<int>>> contoursDraw(contoursCleaned.size());
  for(int i = 0; i < contoursArea.size(); i++) {
    cv::approxPolyDP(cv::Mat(contoursArea[i]), contoursDraw[i], 40, true);
  }
  //  Mat drawing = cv::Mat::zeros(mat.size(), CV_8UC3);
  cv::drawContours(drawing, contoursDraw, -1, cv::Scalar(0, 255, 0), 1);
}

void
corner_harris_detection(image_type& src,
                        const std::function<void(const point_type<int>& point)>& fn) {
  int thresh = 200;
  int max_thresh = 255;
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;
  image_type dst = image_type::zeros(src.size(), CV_32FC1);
  image_type gray = image_type::zeros(src.size(), CV_32FC1);
  src.convertTo(gray, CV_32FC1);
  cv::cornerHarris(gray, dst, blockSize, apertureSize, k);
  image_type dst_norm, dst_norm_scaled;
  cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, image_type());
  cv::convertScaleAbs(dst_norm, dst_norm_scaled);

  for(int i = 0; i < dst_norm.rows; i++) {
    for(int j = 0; j < dst_norm.cols; j++) {
      if((int)dst_norm.at<float>(i, j) > thresh) {
        fn(point_type<int>(j, i));
      }
    }
  }
  //  display_image(dst_norm_scaled);
}

void
write_image(image_type img) {
  static int count = 0;

#if 0
  std::filesystem::path outdir(std::string("./data"));

  if(!std::filesystem::exists(outdir))
    std::filesystem::create_directories(outdir);
#else
  {
    const char* outdir = "data";
    struct stat st;
    if(stat(outdir, &st) == -1) {
      if(errno == ENOENT) {
        errno = 0;
        if(mkdir(outdir
#ifndef _WIN32
                 ,
                 1777
#endif
                 ) == -1) {
          std::cerr << "Failed making directory '" << outdir << "': " << strerror(errno)
                    << std::endl;
          return;
        }
      }
    }
  }
#endif

  string file = make_filename("frame", ((++count) % max_frames), "png");

  cv::imwrite(cv::String(file), img);
}

void
draw_all_contours_except(image_type& out,
                         contour_vector<int>& contours,
                         int except = -1,
                         int thickness = 1) {
  for(int i = 0; i < contours.size(); i++) {
    if(i == except)
      continue;
    const color_type color = hsv_to_rgb((i * 360 * 10 / contours.size()) % 360, 1.0, 1.0);
    auto contour = simplify_polyline(contours[i]);
    contours[i] = contour;
    const double area = cv::contourArea(contours[i], false);
    if(area < 1)
      continue;
    cv::drawContours(out, contours, i, color, thickness, cv::LINE_AA);
  }
}

void
draw_all_contours(image_type& out, contour_vector<int>& contours, int thickness) {
  draw_all_contours_except(out, contours, -1, thickness);
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
image_info(image_type img) {
  std::cerr << "image cols=" << img.cols << " rows=" << img.rows
            << " channels=" << img.channels() << " depth="
            << (img.depth() == CV_8U    ? "CV_8U"
                : img.depth() == CV_32F ? "CV_32F"
                                        : "CV_??")
            << std::endl;
}

JSValue
vec4i_to_js(const cv::Vec4i& v) {
  return pointer_to_js(js, (const int*)&v[0], (int)4);
}

template<>
JSValue
points_to_js<point_type<int>>(const std::vector<point_type<int>>& v) {
  std::function<JSValue(const point_type<int>&)> fn(
      [](const point_type<int>& point) -> JSValue {
        return js.create_point(point.x, point.y);
      });
  return vector_to_js(js, v, fn);
}

jsrt::value
contours_to_array(JSContext* ctx, const contour_vector<int>& contours) {
  JSValue ret = JS_NewArray(ctx);
  uint32_t i, n = contours.size();
  for(i = 0; i < n; i++) {
    JS_SetPropertyUint32(ctx, ret, i, js_contour_new(ctx, contours[i]));
  }
  return ret;
}

Line<int>
lineFrom(const cv::Vec4i& vec) {
  return Line<int>(vec[0], vec[1], vec[2], vec[3]);
}

void
process_raster(std::function<void(std::string, cv::Mat*)> display_image, int show_image) {
  display_image("imgOriginal", &imgOriginal);

  image_type frameLab, frameLabCn[3];
  imgOriginal.copyTo(frameLab);
  cv::cvtColor(frameLab, frameLab, cv::COLOR_BGR2Lab);
  cv::split(frameLab, frameLabCn);

  // cv::equalizeHist(frameLabCn[0], imgGrayscale);
  cv::normalize(frameLabCn[0], imgGrayscale, 0, 255, cv::NORM_MINMAX);

  display_image("imgGrayscale", &imgGrayscale);
  // cv::GaussianBlur(imgGrayscale, imgBlurred, cv::Size(config.blur_kernel_size * 2 + 1,
  // config.blur_kernel_size* 2 + 1), (double)config.blur_sigma * 0.01);
  // cv::edgePreservingFilter(imgGrayscale, imgBlurred, cv::NORMCONV_FILTER,
  // (double)config.blur_sigma_r * 0.01, (double)config.blur_sigma_s * 0.01);
  cv::bilateralFilter(imgGrayscale,
                      imgBlurred,
                      -1,
                      (double)config.blur_sigma_r * 0.01,
                      config.blur_kernel_size * 2 + 1);

  // auto_canny(imgBlurred, imgCanny,1);
  cv::Canny(imgBlurred, imgCanny, thresh, thresh2, apertureSize);

  image_type strel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(19, 19));

  cv::morphologyEx(imgCanny,
                   imgMorphology,
                   config.morphology_operator ? cv::MORPH_DILATE : cv::MORPH_CLOSE,
                   cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                             cv::Size(config.morphology_kernel_size + 1,
                                                      config.morphology_kernel_size + 1)));

  cv::cvtColor(imgBlurred, imgBlurred, cv::COLOR_GRAY2BGR);
  /*
   hough_lines((morphology_enable > 1) ? imgMorphology : imgCanny, [&hough, &houghLines](int x1,
   int y1, int x2, int y2) { cv::Vec4i vec(x1, y1, x2, y2); hough.push_back(vec);
     houghLines.push_back(Line<int>(x1, y1, x2, y2));
   });*/

  corner_harris_detection(imgGrayscale, [&](const point_type<int>& pt) {
    cv::circle(imgCanny, pt, 10, cv::Scalar(0, 255, 0, 255), 2, cv::LINE_8);
  });

  if(show_diagnostics)
    image_info(imgMorphology);

  imgVector = color_type(0, 0, 0, 0);

  display_image("imgBlurred", &imgBlurred);
  display_image("imgCanny", &imgCanny);
  display_image("imgMorphology", &imgMorphology);

  cv::cvtColor(imgGrayscale, imgGrayscale, cv::COLOR_GRAY2BGR);
}

void
process_geometry(std::function<void(std::string, cv::Mat*)> display_image, int show_image) {
  typedef std::vector<Line<int>> line_vector;
  std::vector<cv::Vec4i> hough;
  line_vector houghLines;
  std::vector<cv::Vec3f> circles;
  std::vector<point_vector<float>> contours2;
  std::vector<cv::Vec4i> hier;
  //  apply_clahe(imgOriginal, imgOriginal);XY
  (morphology_enable > 1) ? imgMorphology.copyTo(imgRaw) : imgCanny.copyTo(imgRaw);
  std::vector<point_vector<int>> contours =
      get_contours((morphology_enable > 1) ? imgMorphology : imgCanny, hier, CV_RETR_TREE);
  std::vector<point_vector<int>> external =
      get_contours(morphology_enable > 1 ? imgMorphology : imgCanny, hier, CV_RETR_EXTERNAL);
  draw_all_contours(imgGrayscale, external, 1);

  // imgMorphology.convertTo(imgMorphology, CV_32SC1);

  /* imgCanny = color_type::all(255) - imgCanny;
   imgMorphology = color_type::all(255) - imgMorphology;

   cv::cvtColor(imgCanny, imgCanny, cv::COLOR_GRAY2BGR);
   cv::cvtColor(imgMorphology, imgMorphology, cv::COLOR_GRAY2BGR);
*/
  if(show_diagnostics)
    std::cerr << "Num contours: " << contours.size() << std::endl;
  point_vector<int> largestContour;
  int largestIndex = get_largest_contour(contours, largestContour);
  if(largestIndex != -1) {
    draw_all_contours_except(imgVector, contours, largestIndex, 1);
    if(show_diagnostics)
      std::cerr << "largestIndex: " << largestIndex << std::endl;
    cv::drawContours(
        imgVector, contours, largestIndex, color_type(0, 0, 255, 255), 2, cv::LINE_8);
  } else {
    draw_all_contours(imgVector, contours, 1);
  }
  display_image("imgVector", &imgVector);
  cv::cvtColor(imgCanny, imgCanny, cv::COLOR_GRAY2BGR);
  draw_lines(imgCanny, houghLines, cv::Scalar(0, 0, 255, 255), 1, cv::LINE_8);
  display_image("imgCanny", &imgCanny);

  /* if(dptr != nullptr)
     cv::drawContours(*dptr, contours, -1, color_type(0, 0, 255), 1, cv::LINE_AA);*/

  if(contours.empty()) {
    logfile << "No contours" << std::endl;
    // keycode = cv::waitKey(100);
    return;
  }

  std::ostringstream contourStr;
  double maxArea = 0;
  std::vector<line_type> lines;
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
  for(contour_vector<int>::const_iterator it = contours.cbegin(); it != contours.cend();
      ++i, ++it) {
    const std::vector<point_type<int>>& a = *it;
    int depth = contourDepth(i);
  }
  i = 0;
  for(contour_vector<int>::const_iterator it = contours.cbegin(); it != contours.cend();
      ++i, ++it) {
    const std::vector<point_type<int>>& a = *it;
    if(a.size() >= 3) {
      point_vector<float> c;
      cv::approxPolyDP(a, c, 8, true);
      double area = cv::contourArea(c);
      int depth = contourDepth(i);
      if(area > maxArea)
        maxArea = area;
      contours2.push_back(c);
      if(contourStr.str().size())
        contourStr << "\n";
      /*  out_points(contourStr, a);
        logfile << "hier[i] = {" << hier[i][0] << ", " << hier[i][1] << ", " << hier[i][2] << ",
        "
        << hier[i][3] << ", "
                << "} " << std::endl;
        logfile << "contourDepth(i) = " << depth << std::endl;

        if(dptr != nullptr)
          cv::drawContours(*dptr, contours, i, hsv_to_rgb(depth * 10, 1.0, 1.0), 2,
        cv::LINE_AA);*/
    }
  }

  display_image("imgGrayscale", &imgGrayscale);

  if(maxArea == 0) {
    // keycode = cv::waitKey(16);
    logfile << "No contour area" << std::endl;
    return;
  }

  for_each(contours2.begin(),
           contours2.end(),
           [&lines](const std::vector<point_type<float>>& a) {
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

  if(false) {

    std::list<Line<float>> filteredLines;
    std::vector<bool> takenLines;
    std::vector<float> lineLengths;
    std::array<int, 8> histogram = {0, 0, 0, 0, 0, 0, 0, 0};
    std::array<float, 8> angles = {0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<cv::Vec4i> linesHier;
    linesHier.resize(lines.size());
    takenLines.resize(lines.size());

    //     sort(lines.begin(), lines.end());

    transform(lines.begin(),
              lines.end(),
              back_inserter(lineLengths),
              [&](Line<float>& l) -> float { return l.length(); });
    std::cout << "Num lines: " << lines.size() << std::endl;
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

        std::vector<float> distances; // = line_distances(line, lines.begin(), lines.end());
        std::vector<float> angleoffs = angle_diffs(line, lines.begin(), lines.end());
        std::vector<LineEnd<float>> line_ends;
        std::vector<Line<float>*> adjacent_lines;

        std::vector<int> adjacent = filter_lines(lines.begin(),
                                                 lines.end(),
                                                 [&](Line<float>& l2, size_t index) -> bool {
                                                   size_t point_index;
                                                   double min_dist =
                                                       line.minDistance(l2, &point_index);
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

        std::vector<int> parallel =
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
                       [&line](Line<float>* l2) -> float { return line.minDistance(*l2); });
        std::transform(adjacent_lines.begin(),
                       adjacent_lines.end(),
                       back_inserter(line_ends),
                       [&line](Line<float>* l2) -> LineEnd<float> {
                         LineEnd<float> end;
                         line.nearestEnd(*l2, end);
                         return end;
                       });
        angleoffs.clear();
        std::transform(adjacent_lines.begin(),
                       adjacent_lines.end(),
                       back_inserter(angleoffs),
                       [&line](Line<float>* l2) -> float { return line.angleDiff(*l2); });
        std::vector<int> angleoffs_i;

        std::transform(angleoffs.begin(),
                       angleoffs.end(),
                       back_inserter(angleoffs_i),
                       [](const float ang) -> int { return int(ang * 180 / M_PI) % 180; });

        point_vector<int> centers;
        std::transform(adjacent_lines.begin(),
                       adjacent_lines.end(),
                       back_inserter(centers),
                       [](Line<float>* line) -> point_type<int> { return line->center(); });

        Matrix<double> rot = Matrix<double>::rotation(-line.angle());

        Line<float> l(line);
        logfile << "angle: " << (line.angle() * 180 / M_PI) << std::endl;
        logfile << "a: " << l.a << " b: " << l.b << std::endl;
        l.a = rot.transform_point(l.a);
        l.b = rot.transform_point(l.b);
        logfile << "a: " << l.a << " b: " << l.b << std::endl;
        /*logfile << "adjacent(" << i << ")" << adjacent << std::endl;
          logfile << "distances(" << i << ")" << distances << std::endl;
          logfile << "angleoffs(" << i << ")" << angleoffs_i << std::endl;*/
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
      Matrix<double> r = Matrix<double>::rotation(M_PI / 4, point_type<float>(50, 50));
      Matrix<double> t = Matrix<double>::translation(120, -60);
      Matrix<double> mult;

      mult = t * r * s;

      logfile << "matrix x " << to_string(mult) << std::endl;
      logfile << "matrix init " << to_string(m) << std::endl;
      logfile << "matrix scale " << to_string(s) << std::endl;
      logfile << "matrix rotate " << to_string(r) << std::endl;
      logfile << "matrix translate " << to_string(t) << std::endl;

      point_type<float> p(100, 50);
      point_vector<float> pl = {p};
      point_vector<float> ol;

      mult.transform_points(pl.cbegin(), pl.cend(), std::back_inserter(ol));
      logfile << "transformed point: " << ol << std::endl;
    }

    logfile << "histogram:";
    std::for_each(histogram.begin(), histogram.end(), [](const int count) {
      logfile << ' ' << count;
    });
    logfile << std::endl;
    logfile << "angles:";
    std::for_each(angles.begin(), angles.end(), [](const float a) {
      logfile << ' ' << (int)(a * 180 / M_PI);
    });
    logfile << std::endl;
    draw_all_lines(imgGrayscale, filteredLines, [&](int index, size_t len) -> int {
      return lines[index].length() * 10;
    });
    logfile << "Num lines: " << lines.size() << std::endl;
    logfile << "Num filteredLines: " << filteredLines.size() << std::endl;
  }
  // filename << "contour-" << ++count << ".svg";

  if(0) {
    std::string svg = make_filename("contour", ++num_iterations, "svg");
    svg_export_file<float>(contours2, svg);
    unlink("contour.svg");
    rename("contour.svg.tmp", "contour.svg");
  }

  std::vector<point_vector<int>> squares;

  {
    jsrt::value args[3] = {contours_to_array(js.ctx, contours),
                           vector_to_js(js, hier, &vec4i_to_js)};

    js.set_global("contours", args[0]);
    js.set_global("hier", args[1]);

    js.set_global("HIER_NEXT", js.create(0));
    js.set_global("HIER_PREVIOUS", js.create(1));
    js.set_global("HIER_FIRSTCHILD", js.create(2));
    js.set_global("HIER_PARENT", js.create(3));

    check_eval();

    jsrt::value obj = args[2] = js.create_object();

    js.set_property(obj, "cols", js.create(imgRaw.cols));
    js.set_property(obj, "rows", js.create(imgRaw.rows));
    js.set_property(obj, "imgRaw", js_mat_wrap(js.ctx, imgRaw));
    js.set_property(obj, "imgVector", js_mat_wrap(js.ctx, imgVector));
    js.set_property(obj, "imgOriginal", js_mat_wrap(js.ctx, imgOriginal));
    js.set_property(obj, "imgTemp", js_mat_wrap(js.ctx, imgTemp));
    js.set_property(obj, "imgGrayscale", js_mat_wrap(js.ctx, imgGrayscale));
    js.set_property(obj, "imgBlurred", js_mat_wrap(js.ctx, imgBlurred));
    js.set_property(obj, "imgCanny", js_mat_wrap(js.ctx, imgCanny));
    js.set_property(obj, "imgMorphology", js_mat_wrap(js.ctx, imgMorphology));

    if(js.is_function(processFn))
      js.call(processFn, 3, args);
  }

  {
    point_vector<float> src = {point_type<float>(50, 50),
                               point_type<float>(100, 50),
                               point_type<float>(100, 100),
                               point_type<float>(50, 100)};
    point_vector<float> dst = {point_type<float>(100, 0),
                               point_type<float>(150, 0),
                               point_type<float>(150, 50),
                               point_type<float>(100, 50)};
    image_type perspective = cv::getPerspectiveTransform(src, dst);
    logfile << "perspective:" << perspective << std::endl;
  }

  find_rectangles(contours, squares);

  // Draw the circles detected
  for(size_t i = 0; i < circles.size(); i++) {
    point_type<int> center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);
    cv::circle(imgOriginal, center, 3, color_type(255, 0, 0), -1, 8, 0);     // circle center
    cv::circle(imgOriginal, center, radius, color_type(255, 0, 0), 3, 8, 0); // circle outline
    logfile << "center : " << center << "\nradius : " << radius << std::endl;
  }

  std::vector<point_vector<int>> approxim;
  transform(contours2.begin(),
            contours2.end(),
            back_inserter(approxim),
            [](const point_vector<float>& p) -> point_vector<int> {
              return transform_points<int, float>(p);
            });

  for_each(approxim.begin(), approxim.end(), [&](const point_vector<int>& c) {
    const double length = cv::arcLength(c, false);
    const double area = cv::contourArea(c, false);
    cv::Rect rect = cv::boundingRect(c);
    std::vector<point_vector<int>> list;
    list.push_back(c);
    // cv::drawContours(imgOriginal, list, -1, color_type(255, 255, 0), 1);
  });

  /*draw_all_contours(imgVector, contours);
  display_image("imgVector", &imgVector);*/
}

void
process_image(std::function<void(std::string, cv::Mat*)> display_image, int show_image) {
  /*  switch(show_image) {
      case MORPHOLOGY: mptr = &imgMorphology; break;
      case ORIGINAL: mptr = &imgOriginal; break;
      case GRAYSCALE: mptr = &imgGrayscale; break;
      case CANNY:
      default: mptr = &imgCanny; break;
    }*/
  timer.start();
  logfile << "Got frame (resolution: " << imgOriginal.cols << "x" << imgOriginal.rows << ")"
          << std::endl;
  process_raster(display_image, show_image);
  auto before = high_resolution_clock::now();
  process_geometry(display_image, show_image);
  JSValue test_arr = js.get_global("test_array");
  std::vector<int32_t> num_vec;

  std::transform(js.begin(test_arr),
                 js.end(test_arr),
                 std::back_inserter(num_vec),
                 [&](const JSValue& test_arr) -> int32_t {
                   int32_t num;
                   js.get_number(test_arr, num);
                   //  std::cerr << "array member <" << js.typestr(test_arr) << ">: " << num
                   //  << std::endl;
                   return num;
                 });

  std::string str = js.to_string(test_arr);

  //   std::cerr << "array object <" << js.typestr(test_arr) << ">: " << str << std::endl;

  auto after = high_resolution_clock::now();
  bool do_timing = false;

  if(do_timing) {
    duration<double, std::milli> fp_ms = after - before;
    auto int_ms = duration_cast<milliseconds>(after - before);
    duration<long, std::micro> int_usec = int_ms;
    std::cout << "f() took " << fp_ms.count() << " ms, " << "or " << int_ms.count()
              << " whole milliseconds " << "(which is " << int_usec.count()
              << " whole microseconds)" << std::endl;
  }
  timer.stop();

  //  std::cerr << "\033[1mTimer duration\033[0m " << (int)timer.elapsedSeconds() << "s " <<
  //  ((int)timer.elapsedMilliseconds() % 1000) << "ms" << std::endl;
}

/**********************************************************/

static JSValue
js_print(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  int i;
  const char* str;

  for(i = 0; i < argc; i++) {
    if(i != 0)
      logfile << ' ';
    str = JS_ToCString(ctx, argv[i]);
    if(!str)
      return JS_EXCEPTION;
    std::cerr << str;
    JS_FreeCString(ctx, str);
  }
  std::cerr << std::endl;
  return JS_UNDEFINED;
}

extern "C" {

int
js_init(int argc, char* argv[]) {

  js.init(argc, argv);

  //  JSValue* fn = js.get_function("drawContour");
  JSValue global_obj = js.global_object();

  /*  js_point_constructor(js.ctx, js.global_object(), nullptr);
    js_rect_constructor(js.ctx, js.global_object(), nullptr);
    js_size_constructor(js.ctx, js.global_object(), nullptr);
    js_mat_constructor(js.ctx, js.global_object(), nullptr);
    js_contour_constructor(js.ctx, js.global_object(), nullptr);
    js_point_iterator_constructor(js.ctx, js.global_object(), nullptr);
  */
  js_init_module_point(js.ctx, "point");
  js_init_module_size(js.ctx, "size");
  js_init_module_rect(js.ctx, "rect");
  js_init_module_mat(js.ctx, "mat");
  js_init_module_point_iterator(js.ctx, "point-iterator");
  js_init_module_contour(js.ctx, "contour");
  js_init_module_line(js.ctx, "line");
  //js_init_module_draw(js.ctx, "draw");
  //js_init_module_cv(js.ctx, "cv");
  //js_init_module_video_capture(js.ctx, "video-capture");

  jsrt::value console = js.get_global("console");
  JS_SetPropertyStr(js.ctx, console, "log", JS_NewCFunction(js.ctx, js_print, "log", 1));
  jsrt::value ctor = js.get_global("Point");
  //  std::cerr << "function_name: " << js.function_name(ctor) << std::endl;

  // ret = js.eval_file("lib.js", 1);

  // js_draw_functions(js.ctx, js.global_object());
  /*  js.add_function("drawContour", &js_draw_contour, 2);
    js.add_function("drawLine", &js_draw_line, 2);
    js.add_function("pdrawRect", &js_draw_rect, 2);
    js.add_function("drawPolygon", &js_draw_polygon, 2);
    js.add_function("drawCircle", &js_draw_circle, 2);
  */

  // check_eval();

  /*
    std::cerr << "property names: " << js.property_names(global_obj) << std::endl;
    std::cerr << "'console' property names: " << js.property_names(js.get_property<const
    char*>(global_obj, "console"))
              << std::endl;
  */
  /*  JSValue testFn = js.get_property<const char*>(global_obj, "test");
    JSValue drawContourFn = js.get_property<const char*>(js.global_object(), "drawContour");
    JSValue jsPoint = js.create_point(150, 100);
   */ /*
    if(show_diagnostics)
      std::cerr << "js.eval_file ret=" << ret << " globalObj=" << js.to_str(global_obj)
                << " testFn=" << js.to_str(testFn) << " processFn="
                << js.to_str(processFn)
                << std::endl;*/
  return 0;
}
}
