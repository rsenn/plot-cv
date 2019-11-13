#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include "psimpl.h"
#include <iostream>

template <class InputIterator>
inline typename std::iterator_traits<InputIterator>::value_type
segment_distance2(InputIterator s1, InputIterator s2, InputIterator p) {
  typedef typename std::iterator_traits<InputIterator>::value_type value_type;
  return psimpl::math::segment_distance2<2, InputIterator>(s1, s2, p);
}

template <class ValueT>
inline ValueT
point_distance(const cv::Point_<ValueT>& p1, const cv::Point_<ValueT>& p2) {
  return std::sqrt(psimpl::math::point_distance2<2>(&p1.x, &p2.x));
}

template <class ValueT> class Line {
public:
  typedef cv::Point_<ValueT> point_type;
  typedef ValueT value_type;
  Line(const point_type& p1, const point_type& p2) : a(p1.x, p1.y), b(p2.x, p2.y) {}

  Line(ValueT x1, ValueT y1, ValueT x2, ValueT y2) : a(x1, y1), b(x2, y2) {}

  ValueT
  length() const {
    return point_distance(a, b);
  }

  point_type
  slope() const {
    point_type diff = a - b;
    return diff;
  }

  void
  swap() {
    point_type temp = a;
    a = b;
    b = temp;
  }

  double
  angle(double factor = 180 / M_PI) const {
    point_type diff = a - b;
    return std::atan2(diff.x, diff.y);
  }

  ValueT
  distance(const point_type& p) const {
    point_type pt = p;
    std::vector<point_type> points;
    points.push_back(a);
    points.push_back(b);
    return segment_distance2((ValueT*)&points[0], ((ValueT*)&points[2]), ((ValueT*)&pt));
  }

  std::pair<ValueT, ValueT>
  distance(const Line<ValueT>& l) const {
    std::pair<ValueT, ValueT> ret = {distance(l.a), distance(l.b)};
    return ret;
  }

  bool
  operator==(const Line<ValueT>& other) const {
    return other.a == this->a && other.b == this->b;
  }

  ValueT
  min_distance(const Line<ValueT>& l) const {
    std::pair<ValueT, ValueT> dist = distance(l);
    return dist.first < dist.second ? dist.first : dist.second;
  }

  point_type a, b;
};

template <class Char, class ValueT>
inline std::basic_ostream<Char>&
operator<<(std::basic_ostream<Char>& os, const Line<ValueT>& line) {
  os << line.a.x << ',' << line.a.y;
  os << " -> ";
  os << line.b.x << ',' << line.b.y;
}

template <class ContainerT>
typename ContainerT::value_type&

findNearestLine(const typename ContainerT::value_type& line, ContainerT& lines) {
  typedef typename ContainerT::iterator iterator_type;
  typedef typename ContainerT::value_type line_type;
  typedef typename line_type::value_type value_type;
  value_type distance = 1e10;
  line_type* ret;
  iterator_type end = lines.end();

  for(iterator_type it = lines.begin(); it != end; ++it) {
    value_type d = (*it).min_distance(line);
    if(*it == line)
      continue;
    if(d < distance) {
      distance = d;
      ret = &(*it);
    }
  }
  return *ret;
}
template <class ContainerT>
typename ContainerT::iterator
findNearestLine(const typename ContainerT::iterator& line, ContainerT& lines) {
  typedef typename ContainerT::iterator iterator_type;
  typedef typename ContainerT::value_type point_type;
  typedef typename point_type::value_type value_type;
  value_type distance = 1e10;
  iterator_type index;
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