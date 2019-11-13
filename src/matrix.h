#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/imgproc/types_c.h>

template <class T = double> class Matrix : public cv::Mat_<T> {
public:
  typedef cv::Mat_<T> base_type;

  Matrix() : base_type(3,3, std::is_same<T, double>::value ? CV_64F : CV_32F) {}
  Matrix(cv::Mat m) : base_type(m) {}

  void
  transform_points(std::vector<cv::Point_<T>>& pt) {
    std::vector<cv::Point3_<T>> in = pt;
    cv::transform(in, in, *this);
    pt = in;
  }

  void
  transform_point(cv::Point_<T>& pt) {
    std::vector<cv::Point3_<T>> v = {pt};
    cv::transform(v, v, *this);
    pt = v[0];
  }

  operator cv::Mat_<T>() const { return *this; }

  static Matrix<T>
  rotation(double angle) {
    return cv::getRotationMatrix2D(cv::Point2f(0, 0), angle, 1);
  }
  static Matrix<T>
  scale(double scale) {
    return cv::getRotationMatrix2D(cv::Point2f(0, 0), 0, scale);
  }
};

template <class T>
inline std::string
to_string(const Matrix<T>& mat) {
  std::ostringstream oss;
  oss << "rows: " << mat.rows;
  oss << " cols: " << mat.cols;
  return oss.str();
}
