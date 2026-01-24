#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <array>
#include <functional>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/imgproc/types_c.h>

template<class T> class Matrix : public cv::Mat {
public:
  typedef cv::Mat_<T> typed_type;
  typedef cv::Mat base_type;

  static const int dim = 3;

  static const int typeId = std::is_same<T, double>::value ? CV_64F : CV_32F;

  Matrix()
      : cv::Mat(cv::Mat::zeros(dim, dim, typeId)) {
    init({1, 0, 0}, {0, 1, 0}, {0, 0, 1});
  }
  Matrix(int xx, int xy, int yx, int yy, int tx, int ty)
      : base_type(dim, dim, typeId) {
    init(xx, xy, yx, yy, tx, ty);
  }
  Matrix(const base_type& m)
      : base_type(dim, dim, typeId) {
    init(m);
  }
  Matrix(const typed_type& m)
      : base_type(dim, dim, typeId) {
    init(m);
  }
  template<class OtherT>
  Matrix(const OtherT& m)
      : base_type(dim, dim, typeId) {
    init(m);
  }

  template<class R = std::array<T, dim>>
  Matrix(R row0, R row1, R row2 = {0, 0, 1})
      : base_type(dim, dim, typeId) {
    init(row0, row1, row2);
  }
  /**
   * @brief      { function_description }
   *
   * @param[in]  m     { parameter_description }
   * @param      pt    The point
   */
  template<class InputIterator, class OutputIterator>
  void transform_points(InputIterator from, InputIterator to, OutputIterator out) const {
    std::transform(from,
                   to,
                   out,
                   std::bind(&Matrix<T>::transform_point, this, std::placeholders::_1));
  }

  template<class InputIterator>
  void transform_points(InputIterator from, InputIterator to) const;

  Matrix<T>& operator=(const cv::MatExpr& expr) {
    init(cv::Mat(expr));
    return *this;
  }

  Matrix<T>& operator=(const Matrix<T>& other) {
    init(other);
    return *this;
  }

  cv::Point_<T> transform_point(const cv::Point_<T>& pt) const;
  ;

  cv::Point_<T>& convert_point(const cv::Point_<T>& in, cv::Point_<T>& out) const;
  ;

  // operator base_type() const { return *this; }

  /*static cv::Mat
  rotation(double angle) {
    return (typed_type(dim, dim) << std::cos(angle), std::sin(angle), 0,
  -std::sin(angle), std::cos(angle), 0, 0, 0, 1);
  }
*/
  static Matrix<T> rotation(double angle) {
    return Matrix<T>({std::cos(angle), std::sin(angle), 0},
                     {-std::sin(angle), std::cos(angle), 0});
  }

  static Matrix<T> scale(double scale) {
    return Matrix<T>({scale, 0, 0}, {0, scale, 0}, {0, 0, 1});
  }

  template<class OtherT> static Matrix<T> translation(OtherT x, OtherT y) {
    return Matrix<T>({1, 0, T(x)}, {0, 1, T(y)}, {0, 0, 1});
  }
  static Matrix<T> identity() { return Matrix<T>({1, 0, 0}, {0, 1, 0}, {0, 0, 1}); }

  cv::Affine3<T> affine() const { return cv::Affine3<T>(*this); }

  static Matrix<T> create(T xx, T xy, T yx, T yy, T tx, T ty) {
    return Matrix<T>({xx, xy, yx}, {yy, tx, ty});
  }

  template<class R = std::array<T, dim>>
  Matrix<T>& init(const R& row0, const R& row1, const R& row2);

  Matrix<T>& init(T xx, T xy, T yx, T yy, T tx, T ty) {
    set_row(0, {xx, xy, tx});
    set_row(1, {yx, yy, ty});
    set_row(2, {0, 0, 1});
    return *this;
  }

  Matrix<T>& init() {
    init({1, 0, 0}, {0, 1, 0}, {0, 0, 1});
    return *this;
  }
  Matrix<T>& init(T values[3][3]) {
    init(values[0], values[1], values[2]);
    return *this;
  }

  Matrix<T>& init(const cv::Mat& other) {
    if(other.rows > 2)
      init(other.ptr<T>(0, 0), other.ptr<T>(1, 0), other.ptr<T>(2, 0));
    else
      init(other.ptr<T>(0, 0), other.ptr<T>(1, 0), {0, 0, 1});
    return *this;
  }

  Matrix<T>& init(const Matrix<T>& other) {
    if(other.rows > 2)
      init(other[0], other[1], other[2]);
    else
      init(other[0], other[1], {0, 0, 1});
    return *this;
  }

  template<class OtherT = float>
  static Matrix<T> rotation(double angle, const cv::Point_<OtherT>& origin) {
    Matrix<T> ret = Matrix<T>::identity();
    const cv::Point_<OtherT> zero(0, 0);
    /*
     Matrix<T> a = Matrix<T>::identity();
     Matrix<T> b = Matrix<T>::identity();
     Matrix<T> c = Matrix<T>::identity();
     */
    if(origin != zero)
      ret.multiplicate(Matrix<T>(1, 0, -T(origin.x), 0, 1, -T(origin.y)));

    ret.multiplicate(Matrix<T>(
        T(std::cos(angle)), T(std::sin(angle)), 0, -T(std::sin(angle)), T(std::cos(angle)), 0));
    if(origin != zero)
      ret.multiplicate(Matrix<T>(1, 0, T(origin.x), 0, 1, T(origin.y)));

    return ret;
  }

  std::array<T, dim>& operator[](int row) {
    return *reinterpret_cast<std::array<T, dim>*>(ptr(row, 0));
  }
  std::array<T, dim> const& operator[](int row) const {
    return *reinterpret_cast<std::array<T, dim> const*>(ptr(row, 0));
  }

  Matrix<T>& multiplicate(const Matrix<T>& matrix2);

  Matrix<T> product(const Matrix<T>& other) const;

  Matrix<T> operator*(const Matrix<T>& other) const { return product(other); }
  Matrix<T>& operator*=(const Matrix<T>& other) { return multiplicate(other); }

protected:
  Matrix<T>& set(int row, int col, const T& value) {
    if(base_type::type() == CV_64F)
      *base_type::ptr<double>(row, col) = value;

    else if(base_type::type() == CV_32F)
      *base_type::ptr<float>(row, col) = value;
    else
      throw new std::runtime_error("set");
    return *this;
  }

  template<class R = std::array<T, Matrix<T>::dim>> Matrix<T>& set_row(int row, R arg);

  T get(int row, int col) const;

  const T& ref(int row, int col) const { return *ptr(row, col); }

  T& ref(int row, int col) { return *ptr(row, col); }

  const T* ptr(int row, int col) const;

  T* ptr(int row, int col);
};

std::string to_string(const cv::Mat& mat);

template<class Char, class Value>
inline std::ostream&
operator<<(std::ostream& os, const Matrix<Value>& m) {
  os << to_string(m) << std::endl;
  return os;
}

template<class T>
template<class InputIterator>
inline void
Matrix<T>::transform_points(InputIterator from, InputIterator to) const {
  std::for_each(
      from,
      to,
      std::bind(&Matrix<T>::convert_point, this, std::placeholders::_1, std::placeholders::_1));
}

template<class T>
inline cv::Point_<T>
Matrix<T>::transform_point(const cv::Point_<T>& pt) const {
  T x = at<T>(0, 0) * pt.x + at<T>(0, 1) * pt.y + at<T>(0, 2);
  T y = at<T>(1, 0) * pt.x + at<T>(1, 1) * pt.y + at<T>(1, 2);
  return cv::Point_<T>(x, y);
}

template<class T>
inline cv::Point_<T>&
Matrix<T>::convert_point(const cv::Point_<T>& in, cv::Point_<T>& out) const {
  out.x = at<T>(0, 0) * in.x + at<T>(0, 1) * in.y + at<T>(0, 2);
  out.y = at<T>(1, 0) * in.x + at<T>(1, 1) * in.y + at<T>(1, 2);
  return out;
}

template<class T>
template<class R>
inline Matrix<T>&
Matrix<T>::init(const R& row0, const R& row1, const R& row2) {
  set_row(0, row0);
  set_row(1, row1);
  set_row(2, row2);
  return *this;
}

template<class T>
inline Matrix<T>&
Matrix<T>::multiplicate(const Matrix<T>& matrix2) {
  const Matrix<T>& matrix1 = *this;
  Matrix<T> product;
  for(int x = 0; x < Matrix<T>::dim; ++x) {
    std::array<T, dim>& out = product[x];
    for(int y = 0; y < dim; ++y) {
      T sum = 0;
      for(int z = 0; z < dim; ++z)
        sum += matrix1[x][z] * matrix2[z][y];
      out[y] = sum;
    }
  }
  init(product[0], product[1], product[2]);
  return *this;
}

template<class T>
inline Matrix<T>
Matrix<T>::product(const Matrix<T>& other) const {
  T product;
  Matrix<T> ret;
  int i, j, k;
  for(i = 0; i < dim; i++) {
    std::array<T, dim>& row = ret[i];
    for(j = 0; j < dim; j++) {
      product = 0;
      for(k = 0; k < dim; k++)
        product += row[k] * other.get(k, j);
      row[j] = product;
    }
  }
  return ret;
}

template<class T>
template<class R /*= std::array<T, Matrix<T>::dim> */>
inline Matrix<T>&
Matrix<T>::set_row(int row, R arg) {
  T* arr = ptr(row, 0);
  std::copy(arg.cbegin(), arg.cend(), arr);
  return *this;
}

template<class T>
inline T
Matrix<T>::get(int row, int col) const {
  if(base_type::type() == CV_64F)
    return *base_type::ptr<double>(row, col);
  else if(base_type::type() == CV_32F)
    return *base_type::ptr<float>(row, col);
  else
    throw new std::runtime_error("get");

  return T();
}

template<class T>
inline const T*
Matrix<T>::ptr(int row, int col) const {
  const T* ptr = nullptr;
  if(base_type::type() == CV_64F)
    ptr = (T const*)base_type::ptr<double>(row, col);
  else if(base_type::type() == CV_32F)
    ptr = (T const*)base_type::ptr<float>(row, col);
  else
    throw std::runtime_error("ptr");

  return ptr;
}

template<class T>
inline T*
Matrix<T>::ptr(int row, int col) {
  T* ptr = nullptr;
  if(base_type::type() == CV_64F)
    ptr = (T*)base_type::ptr<double>(row, col);
  else if(base_type::type() == CV_32F)
    ptr = (T*)base_type::ptr<float>(row, col);
  else
    throw std::runtime_error("ptr");

  return ptr;
}

#endif // defined MATRIX_HPP
