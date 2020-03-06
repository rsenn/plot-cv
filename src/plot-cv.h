#ifndef PLOT_CV_H
#define PLOT_CV_H

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>
#include <iomanip>
#include <fstream>
#include <ctime>
#include <unistd.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>
#include <functional>
#include <iostream>

#include "geometry.h"
#include "color.h"
#include "js.h"

typedef std::vector<int> ref_list;
typedef cv::Mat image_type;
typedef cv::Point point2i_type;
typedef cv::Point2d point2d_type;
typedef cv::Point2f point2f_type;
/*
template<class T> struct vector_vector_traits {
  typedef T value_type;
  typedef std::vector<std::vector<T>> type;
};*/

enum { CANNY = 0, ORIGINAL, GRAYSCALE, MORPHOLOGY };

std::string to_string(const cv::Scalar& scalar);

std::string make_filename(const std::string& name,
                          int count,
                          const std::string& ext,
                          const std::string& dir = "tmp");

inline int32_t
get_mtime(const char* filename) {
#if __STDC_VERSION__ >= 201710L
  return std::filesystem::last_write_time(filename);
#else
  struct stat st;
  if(stat(filename, &st) != -1) {
    uint32_t ret = st.st_mtime;
    return ret;
  }
#endif
  return -1;
}

template<class Char, class Value>
inline std::ostream&
operator<<(std::ostream& os, const std::vector<Value>& c) {
  typedef typename std::vector<Value>::const_iterator iterator_type;
  iterator_type end = c.end();
  for(iterator_type it = c.begin(); it != end; ++it) {
    os << ' ';
    os << to_string(*it);
  }
  return os;
}

inline std::string
to_string(const cv::Scalar& scalar) {
  const int pad = 3;
  std::ostringstream oss;
  oss << '[' << std::setfill(' ') << std::setw(pad) << scalar[0] << ',' << std::setfill(' ')
      << std::setw(pad) << scalar[1] << ',' << std::setfill(' ') << std::setw(pad) << scalar[2]
      << ',' << std::setfill(' ') << std::setw(pad) << scalar[3] << ']';
  return oss.str();
}

extern std::ofstream logfile;

extern int thresh;
extern int max_thresh;
extern double max_svg_width;
extern double max_svg_height;
extern int thresholdValue;
extern bool show_diagnostics;
extern double epsilon;

image_type get_alpha_channel(image_type m);

extern image_type imgRaw, imgVector, imgOriginal, imgTemp, imgGrayscale, imgBlurred, imgCanny,
    imgMorphology; // Canny edge image
extern const char* image_names[];

void image_info(image_type img);
contour2i_vector get_contours(image_type src,
                              std::vector<cv::Vec4i>& hierarchy,
                              int flag = CV_RETR_TREE);

void svg_draw_polyline(svg::Document& doc,
                       const point2f_vector& contour_arg,
                       std::function<svg::Color(const point2f_vector&)> color_fn);

struct config_values {
  int morphology_kernel_size;
  int blur_kernel_size;
};

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

void draw_all_contours(image_type& out, contour2i_vector& contours, int thickness = 1);

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
out_points(O& os, const point2i_vector& pl) {
  size_t i, n = pl.size();
  for(i = 0; i < n; ++i) {
    if(i > 0)
      os << ' ';
    out_point(os, pl[i]);
  }
}

template<class Container>
inline void
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

template<class T>
inline void
svg_export_file(const typename contour_list<T>::type& contours, std::string output_file) {

  logfile << "Saving '" << output_file << "'" << std::endl;

  svg::Dimensions dimensions(max_svg_width, max_svg_height);
  svg::Document doc(output_file, svg::Layout(dimensions, svg::Layout::TopLeft));
  svg::LineChart chart(5.0);
  std::vector<double> areas;
  std::transform(contours.begin(),
                 contours.end(),
                 std::back_inserter(areas),
                 [](const typename point_list<T>::type& contour) -> double {
                   return cv::contourArea(transform_points<int, float>(contour));
                 });
  const auto& it = std::max_element(areas.begin(), areas.end());
  double max_area = 0;
  if(it != areas.end()) {
    max_area = *it;
  }

  logfile << "Max area: " << max_area << std::endl;

  const auto& cfn = [&](const typename point_list<T>::type& contour) -> svg::Color {
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

void find_rectangles(const contour2i_vector& contours, contour2i_vector& squares);
JSValue vec4i_to_js(const cv::Vec4i& v);

void write_image(image_type img);

void process_image(std::function<void(std::string, image_type*)> display, int show_image);
extern image_type* mptr;
extern int morphology_operator, morphology_enable;
extern JSValue processFn, global_obj;
extern int thresh, thresh2, apertureSize;

int check_eval();

int js_init(int argc, char* argv[]);
#endif
