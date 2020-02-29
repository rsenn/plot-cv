#ifndef PLOT_CV_HPP
#define PLOT_CV_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>
#include <iomanip>
#include <fstream>

#include "line.h"

typedef Line<float> line_type;
typedef std::vector<line_type> line_list;
typedef std::vector<int> ref_list;
typedef cv::Scalar color_type;
typedef cv::Mat image_type;

enum { CANNY = 0, ORIGINAL, GRAYSCALE, OPEN_CLOSE, CORNERS };

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

#endif