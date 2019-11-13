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

  /**
   * @brief      { function_description }
   *
   * @param[in]  m     { parameter_description }
   * @param      pt    The point
   */
  template <class InputIterator, class OutputIterator>
  void
  transform_points(InputIterator from, InputIterator to, OutputIterator out) const {
    std::transform(from, to, out, std::bind(&Matrix<T>::transform_point, this, std::placeholders::_1));
  }
  template <class InputIterator>
  void
  transform_points(InputIterator from, InputIterator to) const {
    std::for_each(from, to, std::bind(&Matrix<T>::convert_point, this, std::placeholders::_1, std::placeholders::_1));
  }

  Matrix<T>&
  operator=(const cv::MatExpr& expr) {
    base_type::operator=(expr);
    return *this;
  }

  cv::Point_<T>
  transform_point(const cv::Point_<T>& pt) const {
    T x = at<T>(0, 0) * pt.x + at<T>(0, 1) * pt.y + at<T>(0, 2);
    T y = at<T>(1, 0) * pt.x + at<T>(1, 1) * pt.y + at<T>(1, 2);
    return cv::Point_<T>(x, y);
  };

  cv::Point_<T>&
  convert_point(const cv::Point_<T>& in, cv::Point_<T>& out) const {
    out.x = at<T>(0, 0) * in.x + at<T>(0, 1) * in.y + at<T>(0, 2);
    out.y = at<T>(1, 0) * in.x + at<T>(1, 1) * in.y + at<T>(1, 2);
    return out;
  };

  operator base_type() const { return *this; }

  /*static cv::Mat
  rotation(double angle) {
    return (typed_type(3, 3) << std::cos(angle), std::sin(angle), 0, -std::sin(angle), std::cos(angle), 0, 0, 0, 1);
  }
*/
  template <class OtherT = float>
  static Matrix<T> 
  rotation(double angle, const cv::Point_<OtherT>& origin = cv::Point_<OtherT>(0, 0)) {

    Matrix<T> ret(cv::Mat_<T>(3, 3) << (1, 0, T(-origin.x), 0, 1, T(-origin.y), 0, 0, 1));
    ret *=  (cv::Mat_<T>(3, 3) << (std::cos(angle), std::sin(angle), 0, -std::sin(angle), std::cos(angle), 0, 0, 0, 1));
    ret *=  (cv::Mat_<T>(3, 3) << (1, 0, T(origin.x), 0, 1, T(origin.y), 0, 0, 1));

    return ret;
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

  cv::Mat operator*(const Matrix<T>& other) const {
    return *reinterpret_cast<const base_type*>(this) * *reinterpret_cast<const base_type*>(&other);
  }
/*    return (cv::Mat_<T>(3, 3) << 
            (at<T>(0, 0) * other.at<T>(0, 0) + at<T>(1, 0) * other.at<T>(1, 0)),
            (at<T>(0, 1) * other.at<T>(0, 0) + at<T>(1, 1) * other.at<T>(0, 1)),
            (at<T>(0, 0) * other.at<T>(0, 2) + at<T>(1, 0) * other.at<T>(1, 2) + at<T>(0, 2)),
            (at<T>(0, 0) * other.at<T>(1, 0) + at<T>(1, 0) * other.at<T>(1, 1)),
            (at<T>(0, 1) * other.at<T>(1, 0) + at<T>(1, 1) * other.at<T>(1, 1)),
            (at<T>(0, 1) * other.at<T>(0, 2) + at<T>(1, 1) * other.at<T>(1, 2) + at<T>(1, 2)),
            0,
            0,
            1);
*/ 

  Matrix<T>
  operator*=(const Matrix<T>& other) {
    cv::Mat prod = (*this) * other;
    *this = prod;
    return *this;
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
        oss << to_string(mat.at<double>(i, j));
      else if(mat.type() == CV_32F)
        oss << to_string(mat.at<float>(i, j));
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