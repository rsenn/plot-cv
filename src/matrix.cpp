#include "matrix.hpp"
#include "../qjs-opencv/include/line.hpp"
#include "plot-cv.hpp"

std::string
to_string(const cv::Mat& mat) {
  std::ostringstream oss;
  oss << "rows: " << mat.rows;
  oss << " cols: " << mat.cols;
  for(int i = 0; i < mat.rows; ++i) {
    if(i)
      oss << ",\n ";
    else
      oss << " [";
    oss << "[";
    for(int j = 0; j < mat.cols; ++j) {
      if(j)
        oss << "],[";
      if(mat.type() == CV_64F)
        oss << to_string(mat.at<double>(i, j), 4);
      else if(mat.type() == CV_32F)
        oss << to_string(mat.at<float>(i, j), 4);
      else
        throw std::runtime_error("to_string");
    }
    oss << " ]";
  }
  oss << "]\n";
  return oss.str();
}
