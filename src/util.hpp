#ifndef UTIL_HPP
#define UTIL_HPP

#include <string>
#include <numeric>

#define COLOR_BLACK "\x1b[30m"
#define COLOR_RED "\x1b[31m"
#define COLOR_GREEN "\x1b[32m"
#define COLOR_YELLOW "\x1b[33m"
#define COLOR_BLUE "\x1b[34m"
#define COLOR_MAGENTA "\x1b[35m"
#define COLOR_CYAN "\x1b[36m"
#define COLOR_WHITE "\x1b[37m"

#define COLOR_GRAY "\x1b[1;30m"
#define COLOR_LIGHTRED "\x1b[1;31m"
#define COLOR_LIGHTGREEN "\x1b[1;32m"
#define COLOR_LIGHTYELLOW "\x1b[1;33m"
#define COLOR_LIGHTBLUE "\x1b[1;34m"
#define COLOR_LIGHTMAGENTA "\x1b[1;35m"
#define COLOR_LIGHTCYAN "\x1b[1;36m"
#define COLOR_LIGHTWHITE "\x1b[1;37m"

#define COLOR_NONE "\x1b[m"

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
