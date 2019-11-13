#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

template <class ValueT> class Line {
public:
  Line(const cv::Point_<ValueT>& p1, const cv::Point_<ValueT>& p2) : a(p1), b(p2) {}

  Line(ValueT x1, ValueT y1, ValueT x2, ValueT y2) : a(x1, y1), b(x2, y2) {}

  cv::Point_<ValueT> a, b;
};