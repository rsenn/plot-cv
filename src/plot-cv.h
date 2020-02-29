#ifndef PLOT_CV_H
#define PLOT_CV_H

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>
#include <iomanip>
#include <fstream>
#include <ctime>
#include <unistd.h>
#include <sys/time.h>

#include "line.h"

typedef Line<float> line_type;
typedef std::vector<line_type> line_list;
typedef std::vector<int> ref_list;
typedef cv::Mat image_type;

template<class T> struct vector_vector_traits {
  typedef T value_type;
  typedef std::vector<std::vector<T>> type;
};

enum { CANNY = 0, ORIGINAL, GRAYSCALE, OPEN_CLOSE };

std::string to_string(const cv::Scalar& scalar);

std::string make_filename(const std::string& name,
                          int count,
                          const std::string& ext,
                          const std::string& dir = "tmp");

extern std::ofstream logfile;

#endif
