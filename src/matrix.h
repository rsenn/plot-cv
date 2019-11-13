#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/imgproc/types_c.h>

template <class T = double> class Matrix : public cv::Mat {
public:
  typedef cv::Mat_<T> typed_type;
  typedef cv::Mat base_type;

  static const int typeId = std::is_same<T, double>::value ? CV_64F : CV_32F;

  Matrix() : base_type(cv::Mat::zeros(3, 3, typeId)) { init({1, 0, 0}, {0, 1, 0}, {0, 0, 1}); }
  Matrix(int xx, int xy, int yx, int yy, int tx, int ty) : base_type(3, 3, typeId) { init(xx, xy, yx, yy, tx, ty); }
  Matrix(const base_type& m) : base_type(3, 3, typeId) { init(m); }
  Matrix(const typed_type& m) : base_type(3, 3, typeId) { init(m); }
  template <class OtherT> Matrix(const OtherT& m) : base_type(3, 3, typeId) { init(m); }

  template <class R = std::array<T, 3>> Matrix(R row0, R row1, R row2 = {0, 0, 1}) : base_type(3, 3, typeId) { init(row0, row1, row2); }
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
    init(cv::Mat(expr));
    return *this;
  }

  Matrix<T>&
  operator=(const Matrix<T>& other) {
    init(other);
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
    return Matrix<T>({std::cos(angle), std::sin(angle), 0}, {-std::sin(angle), std::cos(angle), 0});
  }

  static Matrix<T>
  scale(double scale) {
    return Matrix<T>({scale, 0, 0}, {0, scale, 0}, {0, 0, 1});
  }

  template <class OtherT>
  static Matrix<T>
  translation(OtherT x, OtherT y) {
    return Matrix<T>({1, 0, T(x)}, {0, 1, T(y)}, {0, 0, 1});
  }
  static Matrix<T>
  identity() {
    return Matrix<T>({1, 0, 0}, {0, 1, 0}, {0, 0, 1});
  }

  cv::Affine3<T>
  affine() const {
    return cv::Affine3<T>(*this);
  }

  static Matrix<T>
  create(T xx, T xy, T yx, T yy, T tx, T ty) {
    return Matrix<T>({xx, xy, yx}, {yy, tx, ty});
  }

  template <class R = std::array<T, 3>>
  Matrix<T>&
  init(const R& row0, const R& row1, const R& row2) {
    setRow(0, row0);
    setRow(1, row1);
    setRow(2, row2);
    return *this;
  }

  Matrix<T>&
  init(T xx, T xy, T yx, T yy, T tx, T ty) {
    setRow(0, {xx, xy, tx});
    setRow(1, {yx, yy, ty});
    setRow(2, {0, 0, 1});
    return *this;
  }

  Matrix<T>&
  init() {
    init({1, 0, 0}, {0, 1, 0}, {0, 0, 1});
    return *this;
  }

  Matrix<T>&
  init(const cv::Mat& other) {
    if(other.rows > 2)
      init({other.at<T>(0, 0), other.at<T>(0, 1), other.at<T>(0, 2)}, {other.at<T>(1, 0), other.at<T>(1, 1), other.at<T>(1, 2)}, {other.at<T>(2, 0), other.at<T>(2, 1), other.at<T>(2, 2)});
    else
      init({other.at<T>(0, 0), other.at<T>(0, 1), other.at<T>(0, 2)}, {other.at<T>(1, 0), other.at<T>(1, 1), other.at<T>(1, 2)}, {0, 0, 1});
    return *this;
  }

  Matrix<T>&
  init(const Matrix<T>& other) {
    if(other.rows > 2)
      init(other[0], other[1], other[2]);
    else
      init(other[0], other[1], {0, 0, 1});
    return *this;
  }

  template <class OtherT = float>
  static Matrix<T>
  rotation(double angle, const cv::Point_<OtherT>& origin) {

    Matrix<T> ret = Matrix<T>::identity();

    const cv::Point_<OtherT> zero(0, 0);
    /*
        Matrix<T> a = Matrix<T>::identity();
        Matrix<T> b = Matrix<T>::identity();
        Matrix<T> c = Matrix<T>::identity();
    */
    if(origin != zero)
      ret.multiplicate(Matrix<T>(1, 0, -T(origin.x), 0, 1, -T(origin.y)));

    ret.multiplicate(Matrix<T>(T(std::cos(angle)), T(std::sin(angle)), 0, -T(std::sin(angle)), T(std::cos(angle)), 0));

    if(origin != zero)
      ret.multiplicate(Matrix<T>(1, 0, T(origin.x), 0, 1, T(origin.y)));

    return ret;
  }

  std::array<T, 3>& operator[](int row) { return *reinterpret_cast<std::array<T, 3>*>(ptr(row, 0)); }
  std::array<T, 3> const& operator[](int row) const { return *reinterpret_cast<std::array<T, 3> const*>(ptr(row, 0)); }

  Matrix<T>&
  multiplicate(const Matrix<T>& matrix2) {
    Matrix<T> const& matrix1 = *this;
    Matrix<T> product;
    for(int x = 0; x < 3; ++x)
      for(int y = 0; y < 3; ++y) {
        T sum = 0;
        for(int z = 0; z < 3; ++z) sum += matrix1[x][z] * matrix2[z][y];
        product.set(x, y, sum);
      }
    product.copyTo(*this);
    return *this;
  }

  Matrix<T>
  product(const Matrix<T>& other) const {
    T product;
    Matrix<T> ret;
    int i, j, k;
    for(i = 0; i < base_type::rows; i++) {
      for(j = 0; j < base_type::cols; j++) {
        product = 0;
        for(k = 0; k < base_type::cols; k++) {
          product += get(i, k) * other.get(k, j);
        }
        ret.set(i, j, product);
      }
    }
    return ret;
  }

  Matrix<T> operator*(const Matrix<T>& other) const { return product(other); }
  Matrix<T>&
  operator*=(const Matrix<T>& other) {
    return multiplicate(other);
  }

protected:
  Matrix<T>&
  set(int row, int col, const T& value) {
    *ptr(row, col) = value;
    return *this;
  }
  template <class R = std::array<T, 3>>
  Matrix<T>&
  setRow(int row, R arr) {
    for(int i = 0; i < base_type::cols; ++i) set(row, i, T(arr[i]));
    return *this;
  }

  T
  get(int row, int col) const {
    return *base_type::ptr();
  }

  const T&
  ref(int row, int col) const {
    return *ptr(row, col);
  }

  T&
  ref(int row, int col) {
    return *ptr(row, col);
  }

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