#ifndef LINE_H
#define LINE_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include "psimpl.h"
#include <iostream>
#include <type_traits>
#include <exception>
#include <string>
template<class T> class LineEnd;

template<class T,
         typename std::enable_if<std::is_integral<T>::value ||
                                     std::is_floating_point<T>::value,
                                 T>::type* = nullptr>
inline std::string
to_string(const T& t, size_t n_pad = 3, char ch_pad = ' ') {
  std::ostringstream oss;
  oss << std::fixed << t;
  std::string ret(oss.str());

  if(ret.find('.') != std::string::npos) {
    while(ret.back() == '0') ret.pop_back();
    if(ret.back() == '.')
      ret.pop_back();
  }
  if(ret.length() < n_pad)
    ret.insert(ret.begin(), n_pad - ret.length(), ch_pad);
  else if(ret.length() > n_pad) {
    size_t i = ret.find('.');
    if(i != std::string::npos) {
      ret.resize(std::max(i, n_pad));
    }
  }

  return ret;
}

template<class T>
inline std::string
to_string(const cv::Point_<T>& pt, size_t n_pad = 3, char ch_pad = '0') {
  std::ostringstream oss;
  oss << to_string(pt.x) << ',' << to_string(pt.y);
  return oss.str();
}

template<class InputIterator>
inline typename std::iterator_traits<InputIterator>::value_type
segment_distance2(InputIterator s1, InputIterator s2, InputIterator p) {
  typedef typename std::iterator_traits<InputIterator>::value_type value_type;
  return psimpl::math::segment_distance2<2, InputIterator>(s1, s2, p);
}

template<class ValueT>
inline ValueT
point_distance(const cv::Point_<ValueT>& p1, const cv::Point_<ValueT>& p2) {
  return std::sqrt(psimpl::math::point_distance2<2>(&p1.x, &p2.x));
}

template<class ValueT>
void
moment_from_angle(double phi, cv::Point_<ValueT>& point) {
  point.x = std::sin(phi);
  point.y = std::cos(phi);
}
template<class ValueT>
double
angle_from_moment(const cv::Point_<ValueT>& point) {
  return std::atan2(point.x, point.y);
}

template<class ValueT> class Line {
public:
  typedef cv::Point_<ValueT> point_type;
  typedef Line<ValueT> line_type;
  typedef ValueT value_type;

  point_type a, b;

  Line(const point_type& p1, const point_type& p2) : a(p1), b(p2) {}

  Line(ValueT x1, ValueT y1, ValueT x2, ValueT y2) : a(x1, y1), b(x2, y2) {}

  ValueT
  length() const {
    point_type diff = a - b;
    return std::sqrt(diff.x * diff.x + diff.y * diff.y);
  }

  point_type
  at(double sigma) const {
    point_type ret;
    ret.x = (a.x + b.x) / 2;
    ret.y = (a.y + b.y) / 2;
    return ret;
  }

  point_type
  center() const {
    return point_type((a.x + b.x) / 2, (a.y + b.y) / 2);
  }
  point_type
  start() const {
    return a;
  }
  point_type
  end() const {
    return b;
  }

  point_type
  slope() const {
    point_type diff = b - a;
    return diff;
  }

  const point_type&
  pivot() const {
    return a;
  }

  const point_type&
  to() const {
    return b;
  }

  void
  swap() {
    point_type temp = a;
    a = b;
    b = temp;
  }

  point_type
  moment() const {
    point_type diff(slope());
    double len = length();
    return point_type(diff.x / len, diff.y / len);
  }

  double
  angle() const {
    point_type diff(slope());

    double phi = angle_from_moment(diff);
    // double len = length();
    // point_type mom, norm(moment());
    //  point_type mom;
    // moment_from_angle(phi, mom);
    // std::cout << "angle " << (phi *180/M_PI) << " x=" << norm.x << ",y=" <<
    // norm.y << " x=" << mom.x << ",y=" << mom.y << std::endl;

    return phi;
  }

  double
  distance(const point_type& p) const {
    return std::sqrt(segment_distance2(&a.x, &b.x, &p.x));
  }

  std::pair<ValueT, size_t>
  endpoint_distances(const Line<ValueT>& l) const {
    size_t offs1, offs2;
    std::pair<ValueT, ValueT> dist(endpoint_distance(l.a, &offs1),
                                   endpoint_distance(l.b, &offs2));

    size_t offs = dist.first < dist.second ? offs1 : offs2;
    return std::make_pair(dist.first < dist.second ? dist.first : dist.second,
                          offs);
  }

  std::pair<ValueT, ValueT>
  endpoint_distances(const cv::Point_<ValueT>& p) const {
    return std::make_pair<ValueT, ValueT>(point_distance<ValueT>(a, p),
                                          point_distance<ValueT>(b, p));
  }

  ValueT
  endpoint_distance(const cv::Point_<ValueT>& p,
                    size_t* point_index = nullptr) const {
    ValueT dist1 = point_distance<ValueT>(a, p),
           dist2 = point_distance<ValueT>(b, p);
    ValueT ret = std::min(dist1, dist2);
    if(point_index)
      *point_index = ret;
    return ret;
  }

  template<class OtherT>
  std::pair<line_type&, line_type&>
  nearest(const Line<OtherT>& l) const {
    return std::make_pair<ValueT, ValueT>(distance(l.a), distance(l.b));
  }

  template<class OtherT>
  bool
  operator==(const Line<OtherT>& other) const {
    return other.a == this->a && other.b == this->b;
  }

  ValueT
  min_distance(Line<ValueT>& l2, size_t* point_index = nullptr) const {
    std::pair<ValueT, size_t> dist = endpoint_distances(l2);
    /* if(intersect(l2))
       return 0;*/
    if(point_index)
      *point_index = dist.second;
    return dist.first;
  }

  ValueT
  nearest_end(Line<ValueT>& l2, LineEnd<ValueT>& end) const {
    size_t point_index;
    ValueT dist = min_distance(l2, &point_index);
    end = LineEnd<ValueT>(l2, point_index);
    return dist;
  }

  ValueT
  angle_diff(const Line<ValueT>& l) const {
    return l.angle() - angle();
  }

  std::vector<cv::Point_<ValueT>>
  points() const {
    std::vector<cv::Point_<ValueT>> ret = {a, b};
    return ret;
  }

  /**
   * Calculates intersect of two lines if exists.
   *
   * @param line1 First line.
   * @param line2 Second line.
   * @param intersect Result intersect.
   * @return True if there is intersect of two lines, otherwise false.
   */
  inline bool
  intersect(const Line<ValueT>& line2, cv::Point_<ValueT>* pt = nullptr) const {
    point_type x = line2.pivot() - pivot();
    point_type d1 = slope();
    point_type d2 = line2.slope();

    float inter = d1.x * d2.y - d1.y * d2.x;

    if(fabs(inter) < 1e-8) {
      return false;
    }

    double t1 = (x.x * d2.y - x.y * d2.x) / inter;
    if(pt)
      *pt = pivot() + d1 * t1;

    return true;
  }

  template<class OtherValueT>
  bool
  operator<(const Line<OtherValueT>& l2) const {
    cv::Point2f a, b;
    a = center();
    b = l2.center();
    return a.y < b.y ? true : a.x < b.x;
  }

  template<class Char = char>
  std::basic_string<Char>
  str(const std::string& comma = ",", const std::string& sep = "|") const {
    point_type p = pivot(), s = slope();

    std::basic_ostringstream<Char> os;
    // os << '[';
    os << to_string(a.x) << comma << to_string(a.y);
    os << sep << to_string(b.x);
    os << comma << to_string(b.y);
    os << '=' << to_string(length());
    // os << '@' << to_string(floor(angle()*180/ M_PI));
    // os << ']';
    return os.str();
  }

  /*template <class Char, class ValueT>
  inline std::basic_ostream<Char>&
  operator<<(std::basic_ostream<Char>& os, const Line<ValueT>& line) {
    os << line.a.x << ',' << line.a.y;
    os << " -> ";
    os << line.b.x << ',' << line.b.y;
  }
  */
};

template<class T, class Char = char>
inline std::basic_string<Char>
to_string(const Line<T>& line) {
  std::basic_string<Char> ret;
  ret = line.str(",", "|");

  return ret;
}

template<class ValueT,
         template<typename> typename Container = std::vector,
         class Char = char>
inline std::basic_string<Char>
to_string(const Container<Line<ValueT>>& lines) {
  typedef typename Container<Line<ValueT>>::const_iterator iterator_type;
  typedef Line<ValueT> value_type;
  std::basic_string<Char> ret;
  iterator_type end = lines.cend();
  for(iterator_type it = lines.cbegin(); it != end; ++it) {
    if(ret.length())
      ret += " ";
    ret += to_string<ValueT, Char>(*it);
  }
  return ret;
}

template<class Char, class Value>
inline std::basic_ostream<Char>&
operator<<(std::basic_ostream<Char>& os, const std::vector<Line<Value>>& c) {
  typedef typename std::vector<Line<Value>>::const_iterator iterator_type;
  iterator_type end = c.cend();
  int i = 0;
  for(iterator_type it = c.cbegin(); it != end; ++it) {
    if(i++ > 0)
      os << " ";
    os << to_string(*it);
  }
  return os;
}

template<class ContainerT>

typename ContainerT::iterator
find_nearest_line(typename ContainerT::value_type& line, ContainerT& lines) {
  typedef typename ContainerT::iterator iterator_type;
  typedef typename ContainerT::value_type line_type;
  typedef typename line_type::value_type value_type;
  value_type distance = 1e10;
  iterator_type end = lines.end();
  iterator_type ret = end;

  for(iterator_type it = lines.begin(); it != end; ++it) {
    value_type d = (*it).min_distance(line);
    if(*it == line)
      continue;
    if(d < distance) {
      distance = d;
      ret = it;
    }
  }
  return ret;
}
template<class ContainerT>
typename ContainerT::iterator
find_nearest_line(typename ContainerT::iterator& line, ContainerT& lines) {
  typedef typename ContainerT::iterator iterator_type;
  typedef typename ContainerT::value_type point_type;
  typedef typename point_type::value_type value_type;
  value_type distance = 1e10;
  iterator_type index = lines.end();
  iterator_type end = lines.end();

  for(iterator_type it = lines.begin(); it != end; ++it) {
    value_type d = (*it).min_distance(*line);
    if(std::distance(line, it) == 0)
      continue;
    if(d < distance) {
      distance = d;
      index = it;
    }
  }
  return index;
}
/*
template <class InputIterator>
InputIterator
find_nearest_line(const InputIterator& line, InputIterator from, InputIterator
to) { typedef InputIterator iterator_type; typedef typename
std::iterator_traits<InputIterator>::value_type point_type; typedef typename
point_type::value_type value_type; value_type distance = 1e10; iterator_type
index = to;

  for(iterator_type it = from; it != to; ++it) {
    value_type d = (*it).min_distance(*line);
    if(std::distance(line, it) == 0)
      continue;
    if(d < distance) {
      distance = d;
      index = it;
    }
  }
  return index;
}
*/

template<class T> class LineEnd {
  Line<T>* line;
  size_t point_index;

protected:
  cv::Point_<T>*
  ptr() {
    return line == nullptr ? nullptr : point_index > 0 ? &line->b : &line->a;
  }
  const cv::Point_<T>*
  const_ptr() const {
    return line == nullptr ? nullptr : point_index > 0 ? &line->b : &line->a;
  }

public:
  LineEnd() {}
  LineEnd(Line<T>& l, size_t pt_i) : line(&l), point_index(pt_i) {}
  ~LineEnd() {}

  cv::Point_<T>&
  point() {
    return *ptr();
  }
  cv::Point_<T> const&
  point() const {
    return *const_ptr();
  }

  operator cv::Point_<T>() const { return point(); }
};

struct LineHierarchy {
  int prevSibling;
  int nextSibling;
  int prevParallel;
  int nextParallel;
};

#endif // defined LINE_H