#ifndef PALETTE_HPP
#define PALETTE_HPP

#include <opencv2/core.hpp>

template<class Array>
static cv::Mat
palette_apply(const cv::Mat& mat, Array palette) {
  cv::Mat result = cv::Mat::zeros(mat.size(), CV_8UC4);

  for(int y = 0; y < mat.rows; y++) {
    for(int x = 0; x < mat.cols; x++) {
      uchar index = mat.at<uchar>(y, x);
      result.at<uint32_t>(y, x) = palette[index];
    }
  }
  return result;
}

#endif /* PALETTE_HPP */
