#ifndef UTIL_HPP
#define UTIL_HPP

#include <string>
#include <numeric>
#include <ranges>

#include <opencv2/core.hpp>

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
  return std::accumulate(start, end, std::string(), [&delim](const std::string& a, const std::string& b) -> std::string { return a + (a.length() > 0 ? delim : "") + b; });
}

extern "C" void* get_heap_base();

typedef struct JSMatDimensions {
  uint32_t rows, cols;
} JSMatDimensions;

template<class T>
static inline JSMatDimensions
mat_dimensions(const T& mat) {
  JSMatDimensions ret;
  ret.rows = mat.rows;
  ret.cols = mat.cols;
  return ret;
}

static inline uint8_t*
mat_ptr(cv::Mat& mat) {
  return reinterpret_cast<uint8_t*>(mat.ptr());
}

static inline uint8_t*
mat_ptr(cv::UMat& mat) {
  cv::UMatData* u;

  if((u = mat.u))
    return reinterpret_cast<uint8_t*>(u->data);

  return nullptr;
}

static inline size_t
mat_offset(const cv::Mat& mat, uint32_t row, uint32_t col) {
  const uchar *base, *ptr;

  base = mat.ptr<uchar>();
  ptr = mat.ptr<uchar>(row, col);

  return ptr - base;
}

static inline size_t
mat_offset(const cv::UMat& mat, uint32_t row, uint32_t col) {
  return (size_t(mat.cols) * row + col) * mat.elemSize();
}

template<class T>
static inline T&
mat_at(cv::Mat& mat, uint32_t row, uint32_t col) {
  return *mat.ptr<T>(row, col);
}

template<class T>
static inline T&
mat_at(cv::UMat& mat, uint32_t row, uint32_t col) {
  size_t offs = mat_offset(mat, row, col);
  return *reinterpret_cast<T*>(mat_ptr(mat) + offs);
}

template<class T>
static inline size_t
mat_size(const T& mat) {
  return mat.elemSize() * mat.total();
}

template<class T>
static inline size_t
mat_channels(const T& mat) {
  return mat.elemSize() / mat.elemSize1();
}

template<class T>
static inline bool
mat_signed(const T& mat) {
  switch(mat.type()) {
    case CV_8S:
    case CV_16S:
    case CV_32S: return true;
    default: return false;
  }
}

template<class T>
static inline bool
mat_floating(const T& mat) {
  switch(mat.type()) {
    case CV_32F:
    case CV_64F: return true;
    default: return false;
  }
}

template<class T>
static inline std::ranges::subrange<T>
sized_range(T ptr, size_t len) {
  return std::ranges::subrange<T>(ptr, ptr + len);
}

template<class T>
static inline std::ranges::subrange<T*>
argument_range(int argc, T* argv) {
  return std::ranges::subrange<T*>(argv, argv + argc);
}

template<class T, int N>
static inline T*
begin(cv::Vec<T, N>& v) {
  return &v[0];
}
template<class T, int N>
static inline T*
end(cv::Vec<T, N>& v) {
  return &v[N];
}

template<class T, int N>
static inline T const*
begin(cv::Vec<T, N> const& v) {
  return &v[0];
}
template<class T, int N>
static inline T const*
end(cv::Vec<T, N> const& v) {
  return &v[N];
}

#endif // defined(UTIL_H)
