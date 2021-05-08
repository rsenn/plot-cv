#ifndef PIXEL_NEIGHBORHOOD_HPP
#define PIXEL_NEIGHBORHOOD_HPP

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::Mat
pixelNeighborhood(cv::Mat& mat) {
  cv::Mat result(mat.size(), mat.type());

  for(int y = 1; y < im.rows - 1; y++) {
    uchar* prev = mat.ptr(y - 1, 0);
    uchar* current = mat.ptr(y, 0);
    uchar* next = mat.ptr(y + 1, 0);

    for(int x = 1; x < im.cols - 1; x++) {
      int count = !!prev[-1] + !!prev[0] + !!prev[1] + !!current[-1] + !!current[0] + !!current[1] + !!next[-1] +
                  !!next[0] + !!next[1];

      result.at<uchar>(y, x) = count;
    }
  }
  return result;
}

#endif /* PIXEL_NEIGHBORHOOD_HPP */
