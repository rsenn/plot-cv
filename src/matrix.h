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
  static Matrix<T>
  rotation(double angle) {
    return cv::Mat(cv::Mat_<T>(3, 3) << (std::cos(angle), std::sin(angle), 0, -std::sin(angle), std::cos(angle), 0, 0, 0, 1));
  }

  cv::Affine3<T>
  affine() const {
    return cv::Affine3<T>(*this);
  }

  template <class OtherT = float>
  static Matrix<T>
  rotation(double angle, const cv::Point_<OtherT>& origin) {

    cv::Mat ret;
    const cv::Point_<OtherT> zero(0, 0);

    Matrix<T> a(Matrix<T>::identity());
    Matrix<T> b(Matrix<T>::identity());
    Matrix<T> c(Matrix<T>::identity());

    if(origin != zero)
      a = cv::Mat(cv::Mat_<T>(3, 3) << (1, 0, T(-origin.x), 0, 1, T(-origin.y), 0, 0, 1));

    b = cv::Mat(cv::Mat_<T>(3, 3) << (std::cos(angle), std::sin(angle), 0, -std::sin(angle), std::cos(angle), 0, 0, 0, 1));

    if(origin != zero)
      c = cv::Mat(cv::Mat_<T>(3, 3) << (1, 0, T(origin.x), 0, 1, T(origin.y), 0, 0, 1));

    if(origin != zero)

    return a.multiply(b).multiply(c);

  return b;
  }

  static Matrix<T>
  scale(double scale) {
    return cv::Mat(cv::Mat_<T>(3, 3) << (scale, 0, 0, 0, scale, 0, 0, 0, 1));
  }

  template <class OtherT>
  static Matrix<T>
  translation(OtherT x, OtherT y) {
    return cv::Mat(cv::Mat_<T>(3, 3) << (1, 0, T(x), 0, 1, T(y), 0, 0, 1));
  }
  static Matrix<T>
  identity() {
    return cv::Mat(cv::Mat_<T>(3, 3) << (1, 0, 0, 0, 1, 0, 0, 0, 1));
  }

  Matrix<T> operator*(const Matrix<T>& other) const { return product(other); }

  Matrix<T>&
  set(int row, int col, const T& value) {
    *ptr(row, col) = value;
    return *this;
  }

  T
  get(int row, int col) const { return *base_type::ptr(); }

  T*
  operator[](int row) { return ptr(row, 0); }
  T const*
  operator[](int row) const { return ptr(row, 0); }

  const T&
  ref(int row, int col) const { return *ptr(row, col); }

   T&
  ref(int row, int col) { return *ptr(row, col); }

  const T*
  ptr(int row, int col) const {
    const T* ptr = nullptr;
    if(base_type::type() == CV_64F)
      ptr = (T const*)base_type::ptr<double>(row, col);
    if(base_type::type() == CV_32F)
      ptr = (T const*)base_type::ptr<float>(row, col);
    return ptr;
  }

  T*
  ptr(int row, int col) {
    T* ptr = nullptr;
    if(base_type::type() == CV_64F)
      ptr = (T*)base_type::ptr<double>(row, col);
    if(base_type::type() == CV_32F)
      ptr = (T*)base_type::ptr<float>(row, col);
    return ptr;
  }

  Matrix<T>
  product(const Matrix<T>& other) const {
    T product;
    Matrix< T> ret(cv::Mat(cv::Mat::zeros(3,3, typeId)));
    int i, j, k;
    for(i = 0; i < base_type::rows; i++) {
      for(j = 0; j < base_type::cols; j++) {
        product = 0;
        for(k = 0; k < base_type::cols; k++) {
          product += (*this)[i][k] * other[k][j];
        }
        ret[i][j] += product;
      }
    }
    return ret;
  }
  Matrix<T>&
  multiply(const Matrix<T>& other) {
    *this = product(other);
    return *this;
  }
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