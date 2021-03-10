#ifndef JSBINDINGS_HPP
#define JSBINDINGS_HPP

#include "js.hpp"
#include <quickjs/cutils.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <iomanip>
#include <map>
#include <iterator>
#include <ranges>
#include <array>

typedef struct {
  BOOL done;
  JSValue value;
} IteratorValue;

typedef cv::Rect2d JSRectDataD;
typedef cv::Mat JSMatData;
typedef cv::Size2d JSSizeDataD;

template<class T> using JSPointData = cv::Point_<T>;
template<class T> using JSSizeData = cv::Size_<T>;
template<class T> using JSRectData = cv::Rect_<T>;
template<class T> using JSContourData = std::vector<JSPointData<T>>;
template<class T> using JSContoursData = std::vector<JSContourData<T>>;

/*typedef JSPointData<int> JSPointData<int>;
typedef JSPointData<float> JSPointDataF;
typedef JSPointData<double> JSPointDataD;*/

typedef cv::VideoCapture JSVideoCaptureData;
typedef cv::TickMeter JSTickMeterData;
typedef cv::Ptr<cv::CLAHE> JSCLAHEData;

template<class T> union JSLineData {
  std::array<T, 4> array;
  cv::Vec<T, 4> vec;
  cv::Scalar_<T> scalar;
  std::array<JSPointData<T>, 2> points;
  std::pair<JSPointData<T>, JSPointData<T>> pt;
  struct {
    T x1, y1, x2, y2;
  };
};

template<class T> struct JSLineTraits {
  typedef std::array<T, 4> array_type;
  typedef cv::Vec<T, 4> vector_type;
  typedef cv::Scalar_<T> scalar_type;
};

template<class T> union JSColorData {
  std::array<T, 4> arr;
  struct {
    T r, g, b, a;
  };
};

template<> union JSColorData<uint8_t> {
  std::array<uint8_t, 4> arr;
  struct {
    uint8_t r, g, b, a;
  };
  uint32_t u32;
};

struct JSPointIteratorData : public std::pair<JSPointData<double>*, JSPointData<double>*> {
  int magic;
};

#if defined(_WIN32) || defined(__MINGW32__)
#define VISIBLE __declspec(dllexport)
#define HIDDEN
#else
#define VISIBLE __attribute__((visibility("default")))
#define HIDDEN __attribute__((visibility("hidden")))
#endif

#define JS_CGETSET_ENUMERABLE_DEF(prop_name, fgetter, fsetter, magic_num)                                                                  \
  {                                                                                                                                        \
    .name = prop_name, .prop_flags = JS_PROP_ENUMERABLE | JS_PROP_CONFIGURABLE, .def_type = JS_DEF_CGETSET_MAGIC, .magic = magic_num,      \
    .u = {                                                                                                                                 \
      .getset = {.get = {.getter_magic = fgetter}, .set = {.setter_magic = fsetter}}                                                       \
    }                                                                                                                                      \
  }

extern "C" {

int js_draw_functions(JSContext* ctx, JSValue parent);
int js_draw_init(JSContext*, JSModuleDef*);

VISIBLE JSValue js_point_new(JSContext*, double x, double y);
VISIBLE JSValue js_point_wrap(JSContext*, const JSPointData<double>&);
VISIBLE JSPointData<double>* js_point_data(JSContext*, JSValueConst val);

int js_point_init(JSContext*, JSModuleDef* m);
void js_point_constructor(JSContext* ctx, JSValue parent, const char* name);

JSModuleDef* js_init_point_module(JSContext*, const char* module_name);

VISIBLE JSValue js_size_new(JSContext* ctx, double w, double h);
VISIBLE JSValue js_size_wrap(JSContext* ctx, const JSSizeData<double>& size);
VISIBLE JSSizeData<double>* js_size_data(JSContext*, JSValueConst val);

int js_size_init(JSContext*, JSModuleDef* m);
JSModuleDef* js_init_size_module(JSContext*, const char* module_name);
void js_size_constructor(JSContext* ctx, JSValue parent, const char* name);

VISIBLE JSRectData<double>* js_rect_data(JSContext*, JSValueConst val);
VISIBLE JSValue js_rect_wrap(JSContext*, const JSRectData<double>&);
int js_rect_init(JSContext*, JSModuleDef*);
JSModuleDef* js_init_rect_module(JSContext*, const char* module_name);

void js_rect_constructor(JSContext* ctx, JSValue parent, const char* name);

VISIBLE JSValue js_line_new(JSContext* ctx, double x1, double y1, double x2, double y2);

int js_point_iterator_init(JSContext*, JSModuleDef* m);
JSModuleDef* js_init_point_iterator_module(JSContext*, const char* module_name);
void js_point_iterator_constructor(JSContext* ctx, JSValue parent, const char* name);

VISIBLE JSContourData<double>* js_contour_data(JSContext*, JSValueConst val);
void js_contour_finalizer(JSRuntime* rt, JSValue val);

JSValue js_contour_to_string(JSContext*, JSValueConst this_val, int argc, JSValueConst* argv);
int js_contour_init(JSContext*, JSModuleDef*);
JSModuleDef* js_init_contour_module(JSContext* ctx, const char* module_name);
void js_contour_constructor(JSContext* ctx, JSValue parent, const char* name);

VISIBLE JSValue js_mat_new(JSContext*, int, int, int);
int js_mat_init(JSContext*, JSModuleDef*);
JSModuleDef* js_init_mat_module(JSContext* ctx, const char* module_name);
void js_mat_constructor(JSContext* ctx, JSValue parent, const char* name);

VISIBLE JSMatData* js_mat_data(JSContext* ctx, JSValueConst val);

JSModuleDef* js_init_module(JSContext* ctx, const char* module_name);
JSModuleDef* js_init_module_point(JSContext*, const char*);
JSModuleDef* js_init_module_point_iterator(JSContext*, const char*);
JSModuleDef* js_init_module_size(JSContext*, const char*);
JSModuleDef* js_init_module_rect(JSContext*, const char*);
JSModuleDef* js_init_module_mat(JSContext*, const char*);
JSModuleDef* js_init_module_contour(JSContext*, const char*);
JSModuleDef* js_init_module_line(JSContext*, const char*);
JSModuleDef* js_init_module_draw(JSContext*, const char*);
JSModuleDef* js_init_module_cv(JSContext*, const char*);
JSModuleDef* js_init_module_video_capture(JSContext*, const char*);

int js_video_capture_init(JSContext*, JSModuleDef*);

VISIBLE JSValue js_video_capture_wrap(JSContext*, cv::VideoCapture* cap);

extern "C" JSValue int32array_ctor, int32array_proto, mat_class, mat_proto, mat_iterator_proto, point_class, line_class,
    point_iterator_class, draw_class, point_iterator_proto, point_proto, rect_class, rect_proto, size_class, size_proto, line_proto,
    draw_proto;

VISIBLE JSValue js_point_iterator_new(JSContext* ctx, const std::pair<JSPointData<double>*, JSPointData<double>*>& range, int magic);
VISIBLE JSValue js_mat_wrap(JSContext*, const cv::Mat& mat);
}

extern "C" JSClassDef js_size_class, js_point_class, js_mat_class, js_rect_class;
extern "C" JSClassID js_point_iterator_class_id, js_line_class_id, js_draw_class_id;

extern "C" const JSCFunctionListEntry js_rect_proto_funcs[];

extern "C" JSClassID js_point_class_id, js_size_class_id, js_rect_class_id, js_mat_class_id, js_mat_iterator_class_id;

extern "C" {
static inline JSValue js_global_get(JSContext* ctx, const char* prop);
static inline BOOL js_is_iterable(JSContext* ctx, JSValueConst obj);
static inline JSValue js_iterator_method(JSContext* ctx, JSValueConst obj);
static inline JSValue js_iterator_new(JSContext* ctx, JSValueConst obj);
static inline IteratorValue js_iterator_next(JSContext* ctx, JSValueConst obj);
static inline JSAtom js_symbol_atom(JSContext* ctx, const char* name);
static inline JSValue js_symbol_ctor(JSContext* ctx);
static inline JSValue js_symbol_get_static(JSContext* ctx, const char* name);
}
/*
template<class Type> JSValue js_contour_new(JSContext* ctx, const JSContourData<Type>& points);

template<> JSValue js_contour_new<double>(JSContext* ctx, const JSContourData<double>& points);

template<> JSValue js_contour_new<float>(JSContext* ctx, const JSContourData<float>& points);

template<> JSValue js_contour_new<int>(JSContext* ctx, const JSContourData<int>& points);
*/
#define countof(x) (sizeof(x) / sizeof((x)[0]))

JSValue js_vector_vec4i_to_array(JSContext*, const std::vector<cv::Vec4i>& vec);

inline JSValueConst
js_ctor(JSContext* ctx, const char* name) {
  JSValue global = JS_GetGlobalObject(ctx);
  JSValueConst ctor = JS_GetPropertyStr(ctx, global, name);
  return ctor;
}

inline JSValueConst
js_proto(JSContext* ctx, const char* name) {
  return JS_GetPrototype(ctx, js_ctor(ctx, name));
}

inline JSValue
js_new(JSContext* ctx, const char* name) {
  return JS_NewObjectProto(ctx, js_proto(ctx, name));
}

int js_color_read(JSContext* ctx, JSValueConst color, JSColorData<double>* out);
int js_color_read(JSContext* ctx, JSValueConst value, JSColorData<uint8_t>* out);

static inline int
js_color_read(JSContext* ctx, JSValueConst value, cv::Scalar* out) {
  JSColorData<double> color;
  int ret;
  if((ret = js_color_read(ctx, value, &color))) {
    (*out)[0] = color.arr[0];
    (*out)[1] = color.arr[1];
    (*out)[2] = color.arr[2];
    (*out)[3] = color.arr[3];
  }
  return ret;
}

template<class T>
static inline int
js_number_read(JSContext* ctx, JSValueConst num, T* out) {
  double d;
  int ret;
  if((ret = !JS_ToFloat64(ctx, &d, num)))
    *out = d;
  return ret;
}

template<>
inline int
js_number_read<int32_t>(JSContext* ctx, JSValueConst num, int32_t* out) {
  return !JS_ToInt32(ctx, out, num);
}

template<>
inline int
js_number_read<uint32_t>(JSContext* ctx, JSValueConst num, uint32_t* out) {
  return !JS_ToUint32(ctx, out, num);
}

template<>
inline int
js_number_read<int64_t>(JSContext* ctx, JSValueConst num, int64_t* out) {
  return !JS_ToInt64(ctx, out, num);
}

template<class T>
static inline JSValue
js_number_new(JSContext* ctx, T num) {
  return JS_NewFloat64(ctx, num);
}

template<>
inline JSValue
js_number_new<int32_t>(JSContext* ctx, int32_t num) {
  return JS_NewInt32(ctx, num);
}
template<>
inline JSValue
js_number_new<uint32_t>(JSContext* ctx, uint32_t num) {
  return JS_NewUint32(ctx, num);
}

template<>
inline JSValue
js_number_new<int64_t>(JSContext* ctx, int64_t num) {
  return JS_NewInt64(ctx, num);
}

int js_ref(JSContext* ctx, const char* name, JSValueConst arg, JSValue value);

static inline std::ranges::subrange<uint8_t*>
js_arraybuffer_range(JSContext* ctx, JSValueConst buffer) {
  size_t size;
  uint8_t* ptr;
  ptr = JS_GetArrayBuffer(ctx, &size, buffer);
  return std::ranges::subrange<uint8_t*>(ptr, ptr + size);
}

static inline size_t
round_to(size_t num, size_t x) {
  num /= x;
  num *= x;
  return num;
}

template<class T>
static inline std::ranges::subrange<T>
js_arraybuffer_range(JSContext* ctx, JSValueConst buffer) {
  typedef typename std::remove_pointer<T>::type value_type;
  size_t size;
  uint8_t* byte_ptr;
  byte_ptr = JS_GetArrayBuffer(ctx, &size, buffer);
  size = round_to(size, sizeof(value_type));
  return std::ranges::subrange<T>(reinterpret_cast<T>(byte_ptr), reinterpret_cast<T>(byte_ptr + size));
}

template<class Ptr>
static inline JSValue
js_arraybuffer_from(JSContext* ctx, const Ptr& begin, const Ptr& end) {
  const uint8_t* ptr;
  size_t len;
  ptr = reinterpret_cast<const uint8_t*>(begin);
  len = reinterpret_cast<const uint8_t*>(end) - ptr;
  return JS_NewArrayBufferCopy(ctx, ptr, len);
}

template<class Ptr>
static inline JSValue
js_arraybuffer_from(JSContext* ctx,
                    const Ptr& begin,
                    const Ptr& end,
                    JSFreeArrayBufferDataFunc& free_func,
                    void* opaque = nullptr,
                    bool is_shared = false) {
  const uint8_t* ptr;
  size_t len;
  ptr = reinterpret_cast<const uint8_t*>(begin);
  len = reinterpret_cast<const uint8_t*>(end) - ptr;
  return JS_NewArrayBuffer(ctx, ptr, len, &free_func, opaque, is_shared);
}

static inline JSValue
js_global_get(JSContext* ctx, const char* prop) {
  JSValue global_obj, ret;

  global_obj = JS_GetGlobalObject(ctx);
  ret = JS_GetPropertyStr(ctx, global_obj, prop);
  JS_FreeValue(ctx, global_obj);
  return ret;
}

static inline JSValue
js_symbol_ctor(JSContext* ctx) {
  return js_global_get(ctx, "Symbol");
}

static inline JSValue
js_symbol_get_static(JSContext* ctx, const char* name) {
  JSValue symbol_ctor, ret;
  symbol_ctor = js_symbol_ctor(ctx);
  ret = JS_GetPropertyStr(ctx, symbol_ctor, name);
  JS_FreeValue(ctx, symbol_ctor);
  return ret;
}

static inline JSAtom
js_symbol_atom(JSContext* ctx, const char* name) {
  JSValue sym = js_symbol_get_static(ctx, name);
  JSAtom ret = JS_ValueToAtom(ctx, sym);
  JS_FreeValue(ctx, sym);
  return ret;
}

static inline JSValue
js_iterator_method(JSContext* ctx, JSValueConst obj) {
  JSAtom atom;
  JSValue ret = JS_UNDEFINED;
  atom = js_symbol_atom(ctx, "iterator");
  if(JS_HasProperty(ctx, obj, atom))
    ret = JS_GetProperty(ctx, obj, atom);
  JS_FreeAtom(ctx, atom);
  if(!JS_IsFunction(ctx, ret)) {
    atom = js_symbol_atom(ctx, "asyncIterator");
    if(JS_HasProperty(ctx, obj, atom))
      ret = JS_GetProperty(ctx, obj, atom);
    JS_FreeAtom(ctx, atom);
  }
  return ret;
}

static inline BOOL
js_is_iterable(JSContext* ctx, JSValueConst obj) {
  JSAtom atom;
  BOOL ret = FALSE;
  atom = js_symbol_atom(ctx, "iterator");
  if(JS_HasProperty(ctx, obj, atom))
    ret = TRUE;
  JS_FreeAtom(ctx, atom);
  if(!ret) {
    atom = js_symbol_atom(ctx, "asyncIterator");
    if(JS_HasProperty(ctx, obj, atom))
      ret = TRUE;
    JS_FreeAtom(ctx, atom);
  }
  return ret;
}

static inline JSValue
js_iterator_new(JSContext* ctx, JSValueConst obj) {
  JSValue fn, ret;
  fn = js_iterator_method(ctx, obj);

  ret = JS_Call(ctx, fn, obj, 0, 0);
  JS_FreeValue(ctx, fn);
  return ret;
}

static inline IteratorValue
js_iterator_next(JSContext* ctx, JSValueConst obj) {
  JSValue fn, result, done;
  IteratorValue ret;

  fn = JS_GetPropertyStr(ctx, obj, "next");

  result = JS_Call(ctx, fn, obj, 0, 0);
  JS_FreeValue(ctx, fn);

  done = JS_GetPropertyStr(ctx, result, "done");
  ret.value = JS_GetPropertyStr(ctx, result, "value");
  JS_FreeValue(ctx, result);

  ret.done = JS_ToBool(ctx, done);
  JS_FreeValue(ctx, done);

  return ret;
}

template<class T, typename std::enable_if<std::is_integral<T>::value || std::is_floating_point<T>::value, T>::type* = nullptr>
static inline int
js_value_to(JSContext* ctx, JSValueConst value, T& out) {
  return js_number_read(ctx, value, &out);
}

static inline int
js_value_to(JSContext* ctx, JSValueConst value, bool& out) {
  out = JS_ToBool(ctx, value);
  return 1;
}

static inline int
js_value_to(JSContext* ctx, JSValueConst value, std::string& out) {
  const char* str;
  size_t len;
  str = JS_ToCStringLen(ctx, &len, value);
  out.clear();
  out.assign(str, len);
  JS_FreeCString(ctx, str);
  return 1;
}

template<class T> class js_iterable {
public:
  static int64_t
  to_vector(JSContext* ctx, JSValueConst arg, std::vector<T>& out) {
    IteratorValue item;
    JSValueConst iter = js_iterator_new(ctx, arg);
    out.clear();
    for(;;) {
      T value;
      item = js_iterator_next(ctx, iter);
      if(item.done)
        break;
      js_value_to(ctx, item.value, value);
      out.push_back(value);
      JS_FreeValue(ctx, item.value);
    }
    JS_FreeValue(ctx, iter);
    return out.size();
  }

  template<size_t N>
  static int64_t
  to_array(JSContext* ctx, JSValueConst arg, std::array<T, N>& out) {
    int64_t i = 0;
    IteratorValue item;
    JSValue iter = js_iterator_new(ctx, arg);
    for(i = 0; i < N; i++) {
      T value;
      item = js_iterator_next(ctx, iter);
      if(item.done)
        break;
      js_value_to(ctx, item.value, value);
      out[i] = value;
      JS_FreeValue(ctx, item.value);
    }
    JS_FreeValue(ctx, iter);
    return i;
  }

  static int64_t
  to_scalar(JSContext* ctx, JSValueConst arg, cv::Scalar_<T>& out) {
    return to_array(ctx, arg, *reinterpret_cast<std::array<T, 4>*>(&out));
  }
};

template<class T>
static inline int64_t
js_iterable_to(JSContext* ctx, JSValueConst arr, std::vector<T>& out) {
  return js_iterable<T>::to_vector(ctx, arr, out);
}

template<class T, size_t N>
static inline int64_t
js_iterable_to(JSContext* ctx, JSValueConst arr, std::array<T, N>& out) {
  typedef js_iterable<T> array_type;
  return array_type::to_array(ctx, arr, out);
}

template<class T>
static inline int64_t
js_iterable_to(JSContext* ctx, JSValueConst arr, cv::Scalar_<T>& out) {
  return js_iterable<T>::to_scalar(ctx, arr, out);
}

#endif
