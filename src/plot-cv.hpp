#ifndef PLOT_CV_HPP
#define PLOT_CV_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>
#include <fstream>
#include <ctime>
#include <unistd.h>
#include <sys/time.h>
#include <unistd.h>
#include <functional>
#include <chrono>
#include <iostream>

#include "color.hpp"
#include "js.hpp"
#include "../qjs-opencv/include/geometry.hpp"
#include "../qjs-opencv/include/util.hpp"

typedef std::vector<int> ref_list;
typedef cv::Mat image_type;
/*
typedef std::vector<cv::Vec4i> vec4i_list;
typedef std::vector<cv::Point> point_vector<int>;
typedef std::vector<cv::Point> point_vector<int>;
typedef std::vector<cv::Point2f> point_vector<float>;
typedef std::vector<cv::Point2f> point_vector<float>;
typedef std::vector<cv::Point2d> point_vector<double>;
typedef std::vector<cv::Point2d> point2d_vector;

typedef std::vector<point_vector<int>> contour_vector<int>;
typedef std::vector<point_vector<float>> contour_vector<float>;*/

/*
template<class T> struct vector_vector_traits {
  typedef T value_type;
  typedef std::vector<std::vector<T>> type;
};*/
extern "C" {

int js_init(int argc, char*[]);
extern cv::Mat* dptr;
}

enum { CANNY = 0, ORIGINAL, GRAYSCALE, MORPHOLOGY };

extern std::ofstream logfile;
extern "C" {

extern int thresh;
extern int max_thresh;
extern double max_svg_width;
extern double max_svg_height;
extern int thresholdValue;
extern bool show_diagnostics;
extern double epsilon;
extern const char* image_names[];
extern int morphology_enable;
extern JSValue processFn, global_obj;
extern int thresh, thresh2, apertureSize;
};

image_type get_alpha_channel(image_type m);

extern "C" image_type imgRaw, imgVector, imgOriginal, imgTemp, imgGrayscale, imgBlurred,
    imgCanny,
    imgMorphology; // Canny edge image

void image_info(image_type img);
std::vector<point_vector<int>> get_contours(image_type src,
                                            std::vector<cv::Vec4i>& hierarchy,
                                            int flag = CV_RETR_TREE);

void svg_draw_polyline(svg::Document& doc,
                       const point_vector<float>& contour_arg,
                       std::function<svg::Color(const point_vector<float>&)> color_fn);

struct config_values {
  int morphology_kernel_size;
  int morphology_operator;
  int blur_kernel_size;
  int blur_sigma;
  int blur_sigma_s;

  int blur_sigma_r;

  int hough_rho;
  int hough_theta;
  int hough_threshold;
  int hough_minlinelen;
  int hough_maxlinegap;
};

extern "C" config_values config;

/**
 * @brief      finds the largest contour
 *
 * @param[in]  contours_un     The contours list
 * @param      bigContour  Receives largest contour
 *
 * @return     Index of largest Contour
 */

template<class T>
inline int
get_largest_contour(const std::vector<std::vector<cv::Point_<T>>>& contours_un,
                    std::vector<cv::Point_<T>>& bigContour) {
  double maxArea = 0.0;
  int largestContour = -1;
  for(size_t i = 0; i < contours_un.size(); i++) {
    double area = cv::contourArea(contours_un[i]);
    if(show_diagnostics) {
      std::cerr << "Area: " + to_string(area) + " Index: " + to_string(i) << std::endl;
    }
    if(area > maxArea) {
      maxArea = area;
      largestContour = i;
    }
  }
  if(largestContour != -1) {
    if(show_diagnostics)
      std::cerr << "largestContour:" << largestContour << std::endl;

    auto largest = contours_un.at(largestContour);

    bigContour.clear();
    // bigContour.resize(largest.size());
    std::copy(largest.cbegin(), largest.cend(), std::back_inserter(bigContour));
  }
  return largestContour;
}

extern config_values config;

void draw_all_contours(image_type& out,
                       std::vector<point_vector<int>>& contours,
                       int thickness = 1);

// Function that calculates the absolute value

template<class T, class O>
inline void
out_point(O& os, const cv::Point_<T>& pt) {
  os << pt.x;
  os << ',';
  os << pt.y;
}

template<class O>
inline void
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
inline void
out_points(O& os, const point_vector<int>& pl) {
  size_t i, n = pl.size();
  for(i = 0; i < n; ++i) {
    if(i > 0)
      os << ' ';
    out_point(os, pl[i]);
  }
}

template<class Container>
inline void
draw_all_lines(
    image_type& out,
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

    cv::line(out, point_type<int>(it->a), point_type<int>(it->b), color, 1, cv::LINE_AA);
  }
}

template<class T>
inline void
svg_export_file(const std::vector<std::vector<cv::Point_<T>>>& contours,
                std::string output_file) {

  logfile << "Saving '" << output_file << "'" << std::endl;

  svg::Dimensions dimensions(max_svg_width, max_svg_height);
  svg::Document doc(output_file, svg::Layout(dimensions, svg::Layout::TopLeft));
  svg::LineChart chart(5.0);
  std::vector<double> areas;
  std::transform(contours.begin(),
                 contours.end(),
                 std::back_inserter(areas),
                 [](const std::vector<cv::Point_<T>>& contour) -> double {
                   point_vector<float> vec = contour;
                   return cv::contourArea(vec);
                 });
  const auto& it = std::max_element(areas.begin(), areas.end());
  double max_area = 0;
  if(it != areas.end()) {
    max_area = *it;
  }

  logfile << "Max area: " << max_area << std::endl;

  const auto& cfn = [&](const std::vector<cv::Point_<T>>& contour) -> svg::Color {
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

template<class P>
inline JSValue
points_to_js(const std::vector<P>& v) {
  std::function<JSValue(const P&)> fn(
      [](const P& point) -> JSValue { return js.create_point(point.x, point.y); });
  return vector_to_js(js, v, fn);
}

template<> JSValue points_to_js<cv::Point>(const std::vector<cv::Point>& v);

void find_rectangles(const contour_vector<int>& contours, contour_vector<int>& squares);
JSValue vec4i_to_js(const cv::Vec4i& v);

void write_image(image_type img);

template<class InputIterator>
std::string
implode(InputIterator begin, InputIterator end, const std::string& separator) {
  std::ostringstream os;
  int i = 0;

  while(begin != end) {
    if(i > 0)
      os << separator;
    os << to_string(*begin);
    i++;
    ++begin;
  }
  return os.str();
}

void process_image(std::function<void(std::string, image_type*)> display, int show_image);

jsrt::value check_eval();

class Timer {
public:
  void start() {
    start_time = std::chrono::system_clock::now();
    running = true;
  }

  void stop() {
    end_time = std::chrono::system_clock::now();
    running = false;
  }

  double elapsedMilliseconds() {
    std::chrono::time_point<std::chrono::system_clock> end;

    if(running) {
      end = std::chrono::system_clock::now();
    } else {
      end = end_time;
    }

    return std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time).count();
  }

  double elapsedSeconds() { return elapsedMilliseconds() / 1000.0; }

private:
  std::chrono::time_point<std::chrono::system_clock> start_time;
  std::chrono::time_point<std::chrono::system_clock> end_time;
  bool running = false;
};

#endif
