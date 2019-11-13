#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/imgproc/types_c.h>

template <class T = double> class Matrix : public cv::Mat_<T> {
public:
  typedef cv::Mat_<T> base_type;

  static const int typeId = std::is_same<T, double>::value ? CV_64F : CV_32F;

  Matrix() : base_type(3,3, typeId) {}
  Matrix(cv::Mat m) : base_type(m) {}

  void
  transform_points(std::vector<cv::Point_<T>>& pt) {
    std::vector<cv::Point3_<T>> in = pt;
    std::vector<cv::Point3_<T>> out;
    out.resize(in.size());
    cv::transform(in, out, *this);
    std::transform(out.cbegin(), out.cend(), pt.begin(), [](const cv::Point3_<T>& pt3) -> cv::Point_<T> {  return cv::Point_<T>(pt3.x,pt3.y); });
  }

  void
  transform_point(cv::Point_<T>& pt) {
    std::vector<cv::Point3_<T>> v = {pt};
    transform_points(v);
    pt = v[0];
  }

  operator cv::Mat_<T>() const { return *this; }

  static  cv::Mat_<T>
  rotation(double angle) {
    cv::Mat_<T> ret(3, 3, typeId);
    ret =  cv::getRotationMatrix2D(cv::Point2f(0, 0), angle, 1);
    return ret;
  }
  static cv::Mat_<T>
  scale(double scale) {
    cv::Mat_<T> ret(3, 3, typeId);
    ret = cv::getRotationMatrix2D(cv::Point2f(0, 0), 0, scale);
    return ret;
  }

template<class OtherT>
    static Matrix<T>
  translation(OtherT x, OtherT y) {
    cv::Mat ret( 3, 2, typeId);

    ret[2] = T(x);
    ret[2+3] = T(y);
    return ret;
  }
};

template <class T>
inline std::string
to_string(const cv::Mat_<T>& mat) {
  std::ostringstream oss;
  oss << "rows: " << mat.rows;
  oss << " cols: " << mat.cols;

  for(int i = 0; i < mat.rows; ++i)  {
    const auto& row = mat.row(i);
    oss << "(";
    for(int j = 0; j < row.cols; ++j) {
      if(j) 
        oss << ", ";
      oss << *row[j];
    }
    oss << ")";
  }

  return oss.str();
}


template <class Char, class Value>
inline std::basic_ostream<Char>&
operator<<(std::basic_ostream<Char>& os, const Matrix<Value>& m) {

  os << '(' << m[0] << ", " << m[1] << ", " << m[2] << ')' << std::endl;
  os << '(' << m[3] << ", " << m[4] << ", " << m[5] << ')'<< std::endl;
}