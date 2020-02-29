#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>
#include "line.h"

template <class T, class Pred>
inline std::vector<int>
filter_lines(const std::vector<T>& c, bool (&pred)(const Line<T>&, size_t)) {
  return filter_lines<std::vector<Line<T>>::iterator, bool(Line<T>&, size_t)>(c.begin(), c.end(), pred);
}

template <class ValueT, class InputIterator>
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

template <class InputIterator>
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

#endif