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
#include <sys/stat.h>
#include <unistd.h>

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

#endif
