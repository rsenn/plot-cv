#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>
#include "line.h"

typedef std::vector<cv::Point> point_vector;
typedef std::vector<cv::Point2f> point2f_vector;


// Function that calculates the area given a
// std::vector of vertices in the XY plane.
template<class P>
double
polygon_area(std::vector<P> list) {

  if(list.size() < 3)
    return 0;
  double area = 0;                     // Total Area
  double diff = 0;                     // Difference Of Y{i + 1} - Y{i - 1}
  unsigned int last = list.size() - 1; // Size Of Vector - 1
  /* Given vertices from 1 to n, we first loop through
  the vertices 2 to n - 1. We will take into account
  vertex 1 and vertex n sepereately */
  for(size_t i = 1; i < last; i++) {
    diff = list[i + 1].y - list[i - 1].y;
    area += list[i].x * diff;
  }
  /* Now We Consider The Vertex 1 And The Vertex N */
  diff = list[1].y - list[last].y;
  area += list[0].x * diff; // Vertex 1
  diff = list[0].y - list[last - 1].y;
  area += list[last].x * diff; // Vertex N
  /* Calculate The Final Answer */
  area = 0.5 * fabs(area);
  return area; // Return The Area
}

std::vector<cv::Point2f>
get_mass_centers(std::vector<point_vector> contours) {
  std::vector<cv::Moments> mu(contours.size());
  std::vector<cv::Point2f> mc(contours.size());
  for(size_t i = 0; i < contours.size(); i++) mu[i] = cv::moments(contours[i], false);
  for(size_t i = 0; i < contours.size(); i++) mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);

  return mc;
}


template<class T, class Pred>
inline std::vector<int>
filter_lines(const std::vector<T>& c, bool (&pred)(const Line<T>&, size_t)) {
  return filter_lines<std::vector<Line<T>>::iterator, bool(Line<T>&, size_t)>(c.begin(), c.end(), pred);
}

template<class ValueT, class InputIterator>
inline std::vector<typename std::iterator_traits<InputIterator>::value_type::value_type>
angle_diffs(Line<ValueT>& line, InputIterator from, InputIterator to) {
  typedef InputIterator iterator_type;
  typedef typename std::iterator_traits<InputIterator>::value_type point_type;
  typedef typename point_type::value_type value_type;
  typedef std::vector<value_type> ret_type;

  ret_type ret;
  value_type distance = 1e10;
  iterator_type index = to;

  for(iterator_type it = from; it != to; ++it) {
    value_type d;

    ret.push_back((*it).angle_diff(line));
  }
  return ret;
}

template<class InputIterator>
inline std::vector<float>
line_distances(typename std::iterator_traits<InputIterator>::value_type& line, InputIterator from, InputIterator to) {
  typedef InputIterator iterator_type;
  typedef typename std::iterator_traits<InputIterator>::value_type line_type;
  typedef typename line_type::value_type value_type;
  typedef std::vector<float> ret_type;

  ret_type ret;
  value_type distance = 1e10;
  iterator_type index = to;

  for(InputIterator it = from; it != to; ++it) {
    /* if(line == *it)
       continue;*/
    ret.push_back(it->min_distance(line));
  }
  return ret;
}

template<class InputIterator, class Pred>
inline std::vector<int>
filter_lines(InputIterator from, InputIterator to, Pred predicate) {
  typedef InputIterator iterator_type;
  typedef typename std::iterator_traits<InputIterator>::value_type value_type;
  std::vector<int> ret;
  size_t index = 0;
  for(iterator_type it = from; it != to; ++it) {

    if(predicate(*it, index++)) {

      std::size_t index = std::distance(from, it);
      ret.push_back(index);
    }
  }
  return ret;
}

template<class T> class PredicateTraits {
public:
  typedef bool type(const Line<T>&, size_t);
  typedef std::function<bool(const Line<T>&, size_t)> function;
};

template<class Char, class Value>
inline std::basic_ostream<Char>&
operator<<(std::basic_ostream<Char>& os, const std::vector<Value>& c) {
  typedef typename std::vector<Value>::const_iterator iterator_type;
  iterator_type end = c.end();
  for(iterator_type it = c.begin(); it != end; ++it) {
    os << ' ';
    os << to_string(*it);
  }
  return os;
}

#endif
