#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/imgproc/types_c.h>

template <class T = double> class Matrix : public cv::Mat {
public:
  typedef cv::Mat_<T> typed_type;
  typedef cv::Mat base_type;

  static const int typeId = std::is_same<T, double>::value ? CV_64F : CV_32F;

  Matrix() : base_type(typed_type(3, 3)) {}
  Matrix(base_type m) : base_type(m) {}
  Matrix(const typed_type& m) : base_type(m) {}
  template <class OtherT> Matrix(const Matrix<OtherT>& m) : base_type(m) {}

  template <class OtherT>

  /**
   * @brief      { function_description }
   *
   * @param[in]  m     { parameter_description }
   * @param      pt    The point
   */
  static void
  transform_points(const cv::Mat& m, std::vector<cv::Point_<OtherT>>& pt) {
    std::vector<cv::Point3_<OtherT>> in;
    std::vector<cv::Point3_<OtherT>> out;
    std::transform(pt.cbegin(), pt.cend(), std::back_inserter(in), [](const cv::Point_<OtherT>& p) -> cv::Point3_<OtherT> { return cv::Point3_<OtherT>(p.x, p.y, 0); });
    out.resize(in.size());
    cv::transform(in, out, m);
    std::transform(out.cbegin(), out.cend(), pt.begin(), [](const cv::Point3_<OtherT>& pt3) -> cv::Point_<OtherT> { return cv::Point_<OtherT>(pt3.x, pt3.y); });
  }


  Matrix<T>&
  operator=(const cv::MatExpr& expr) {
    base_type::operator=(expr);
    return *this;
  }

  template <class OtherT>
  static void
  transform_point(const cv::Mat& m, cv::Point_<OtherT>& pt) {
    std::vector<cv::Point_<OtherT>> v = {pt};
    transform_points(m, v);
    pt = v[0];
  }

  operator base_type() const { return *this; }

  static cv::Mat
  rotation(double angle) {
    return (typed_type(3, 3) << std::cos(angle), std::sin(angle), 0, -std::sin(angle), std::cos(angle), 0, 0, 0, 1);
  }

  static cv::Mat
  scale(double scale) {
    return (cv::Mat_<T>(3, 3) << scale, 0, 0, 0, scale, 0, 0, 0, 1);
  }

  template <class OtherT>
  static cv::Mat
  translation(OtherT x, OtherT y) {
    return (cv::Mat_<T>(3, 3) << 1, 0, T(x), 0, 1, T(y), 0, 0, 1);
  }
  static cv::Mat
  identity() {
    return (cv::Mat_<T>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  }


  cv::Mat
  operator*(const Matrix<T>& other) {

  return (cv::Mat_<T>(3, 3) << 
    (at<T>(0,0) * other.at<T>(0,0) + at<T>(1,0) * other.at<T>(1,0)),
    (at<T>(0,1) * other.at<T>(0,0) + at<T>(1,1) * other.at<T>(0,1)),
    (at<T>(0,0) * other.at<T>(0,2) + at<T>(1,0) * other.at<T>(1,2) + at<T>(0,2)),
    (at<T>(0,0) * other.at<T>(1,0) + at<T>(1,0) * other.at<T>(1,1)),
    (at<T>(0,1) * other.at<T>(1,0) + at<T>(1,1) * other.at<T>(1,1)),
    (at<T>(0,1) * other.at<T>(0,2) + at<T>(1,1) * other.at<T>(1,2) + at<T>(1,2))
  );
};
};

inline std::string
to_string(const cv::Mat& mat) {
  std::ostringstream oss;
  oss << "rows: " << mat.rows;
  oss << " cols: " << mat.cols;
  for(int i = 0; i < mat.rows; ++i) {
    if(i)
      oss << " ";
    oss << "(";
    for(int j = 0; j < mat.cols; ++j) {
      if(j)
        oss << ", ";
      if(mat.type() == CV_64F)
        oss << mat.at<double>(i, j);
      else if(mat.type() == CV_32F)
        oss << mat.at<float>(i, j);
    }
    oss << ")";
  }
  return oss.str();
}
template <class Char, class Value>
inline std::basic_ostream<Char>&
operator<<(std::basic_ostream<Char>& os, const Matrix<Value>& m) {
  os << to_string(m) << std::endl;
}