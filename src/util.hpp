#ifndef UTIL_H
#define UTIL_H

#include <string>
#include <numeric>

template<class Iterator>
static inline std::string
join(const Iterator& start, const Iterator& end, const std::string& delim) {
  return std::accumulate(start,
                         end,
                         std::string(),
                         [&delim](const std::string& a, const std::string& b) -> std::string {
                           return a + (a.length() > 0 ? delim : "") + b;
                         });
}

extern "C" void* get_heap_base();

#endif // defined(UTIL_H)
