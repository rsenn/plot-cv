#ifndef JSBINDINGS_H
#define JSBINDINGS_H

#include "js.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <iomanip>
#include <map>
#include <iterator>

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

template<class T>

union JSLineData {
  std::array<T, 4> arr;
  cv::Vec<T, 4> vec;
  cv::Scalar_<T> scalar;
  std::array<JSPointData<T>, 2> points;
  std::pair<JSPointData<T>, JSPointData<T>> pt;
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

#define JS_CGETSET_ENUMERABLE_DEF(prop_name, fgetter, fsetter, magic_num)                                  \
  {                                                                                                        \
    .name = prop_name, .prop_flags = JS_PROP_ENUMERABLE | JS_PROP_CONFIGURABLE,                            \
    .def_type = JS_DEF_CGETSET_MAGIC, .magic = magic_num, .u = {                                           \
      .getset = {.get = {.getter_magic = fgetter}, .set = {.setter_magic = fsetter}}                       \
    }                                                                                                      \
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

extern "C" JSValue int32array_ctor, int32array_proto, mat_class, mat_proto, mat_iterator_proto, point_class,
    line_class, point_iterator_class, draw_class, point_iterator_proto, point_proto, rect_class, rect_proto,
    size_class, size_proto, line_proto, draw_proto;

VISIBLE JSValue js_point_iterator_new(JSContext* ctx,
                                      const std::pair<JSPointData<double>*, JSPointData<double>*>& range,
                                      int magic);
VISIBLE JSValue js_mat_wrap(JSContext*, const cv::Mat& mat);
}

extern "C" JSClassDef js_size_class, js_point_class, js_mat_class, js_rect_class;
extern "C" JSClassID js_point_iterator_class_id, js_line_class_id, js_draw_class_id;

extern "C" const JSCFunctionListEntry js_rect_proto_funcs[];

extern "C" JSClassID js_point_class_id, js_size_class_id, js_rect_class_id, js_mat_class_id,
    js_mat_iterator_class_id;
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

#endif
