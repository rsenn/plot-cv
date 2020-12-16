#ifndef JSBINDINGS_H
#define JSBINDINGS_H
#include "js.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <iomanip>
#include <map>
#include <iterator>

typedef cv::Rect2d JSRectData;
typedef cv::Mat JSMatData;
typedef cv::Size2d JSSizeData;
typedef cv::Point2d JSPointData;
typedef cv::Point2i JSPointDataI;
typedef cv::VideoCapture JSVideoCaptureData;
typedef cv::Ptr<cv::CLAHE> JSCLAHEData;

typedef union {
  std::array<double, 4> arr;
  cv::Vec4d vec;
  cv::Scalar_<double> scalar;
  std::array<JSPointData, 2> points;
  std::pair<JSPointData, JSPointData> pt;
} JSLineData;

typedef union {
  std::array<int, 4> arr;
  cv::Vec4i vec;
  cv::Scalar_<int> scalar;
  std::array<JSPointDataI, 2> points;
  std::pair<JSPointDataI, JSPointDataI> pt;
} JSLineDataI;

typedef union {
  std::array<double, 4> arr;
  // cv::Vec4d vec;
  // cv::Scalar scalar;
  struct {
    double r, g, b, a;
  } rgb;

} JSColorData;

typedef std::vector<JSPointData> JSContourData;
struct JSPointIteratorData : public std::pair<JSPointData*, JSPointData*> {
  int magic;
};

#if defined(_WIN32) || defined(__MINGW32__)
#define VISIBLE __declspec(dllexport)
#define HIDDEN
#else
#define VISIBLE __attribute__((visibility("default")))
#define HIDDEN __attribute__((visibility("hidden")))
#endif

#define JS_CGETSET_ENUMERABLE_DEF(prop_name, fgetter, fsetter, magic_num)                                              \
  {                                                                                                                    \
    .name = prop_name, .prop_flags = JS_PROP_ENUMERABLE | JS_PROP_CONFIGURABLE, .def_type = JS_DEF_CGETSET_MAGIC,      \
    .magic = magic_num, .u = {                                                                                         \
      .getset = {.get = {.getter_magic = fgetter}, .set = {.setter_magic = fsetter}}                                   \
    }                                                                                                                  \
  }

extern "C" {

int js_draw_functions(JSContext* ctx, JSValue parent);
int js_draw_init(JSContext*, JSModuleDef*);

VISIBLE JSValue js_point_new(JSContext*, double x, double y);
VISIBLE JSValue js_point_wrap(JSContext*, const JSPointData&);
VISIBLE JSPointData* js_point_data(JSContext*, JSValueConst val);

int js_point_init(JSContext*, JSModuleDef* m);
void js_point_constructor(JSContext* ctx, JSValue parent, const char* name);

JSModuleDef* js_init_point_module(JSContext*, const char* module_name);

VISIBLE JSValue js_size_new(JSContext* ctx, double w, double h);
VISIBLE JSValue js_size_wrap(JSContext* ctx, const JSSizeData& size);
VISIBLE JSSizeData* js_size_data(JSContext*, JSValueConst val);

int js_size_init(JSContext*, JSModuleDef* m);
JSModuleDef* js_init_size_module(JSContext*, const char* module_name);
void js_size_constructor(JSContext* ctx, JSValue parent, const char* name);

VISIBLE JSRectData* js_rect_data(JSContext*, JSValueConst val);
VISIBLE JSValue js_rect_wrap(JSContext*, const JSRectData&);
int js_rect_init(JSContext*, JSModuleDef*);
JSModuleDef* js_init_rect_module(JSContext*, const char* module_name);

void js_rect_constructor(JSContext* ctx, JSValue parent, const char* name);

VISIBLE JSValue js_line_new(JSContext* ctx, double x1, double y1, double x2, double y2);

int js_point_iterator_init(JSContext*, JSModuleDef* m);
JSModuleDef* js_init_point_iterator_module(JSContext*, const char* module_name);
void js_point_iterator_constructor(JSContext* ctx, JSValue parent, const char* name);

VISIBLE JSContourData* js_contour_data(JSContext*, JSValueConst val);
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

extern "C" JSValue contour_class, contour_proto, int32array_ctor, int32array_proto, mat_class, mat_proto,
    mat_iterator_proto, point_class, line_class, point_iterator_class, draw_class, point_iterator_proto, point_proto,
    rect_class, rect_proto, size_class, size_proto, line_proto, draw_proto;

VISIBLE JSValue js_point_iterator_new(JSContext* ctx, const std::pair<JSPointData*, JSPointData*>& range, int magic);
VISIBLE JSValue js_mat_wrap(JSContext*, const cv::Mat& mat);
VISIBLE JSValue js_contour2d_new(JSContext*, const std::vector<cv::Point_<double>>& points);
VISIBLE JSValue js_contour2f_new(JSContext*, const std::vector<cv::Point_<float>>& points);
VISIBLE JSValue js_contour2i_new(JSContext*, const std::vector<cv::Point_<int>>& points);
}

extern "C" JSValue contour_proto;
extern "C" JSClassDef js_contour_class, js_size_class, js_point_class, js_mat_class, js_rect_class;
extern "C" JSClassID js_contour_class_id;
extern "C" JSClassID js_point_iterator_class_id, js_line_class_id, js_draw_class_id;

extern "C" const JSCFunctionListEntry js_rect_proto_funcs[];

extern "C" JSClassID js_point_class_id, js_size_class_id, js_rect_class_id, js_mat_class_id, js_mat_iterator_class_id;

template<class Type> JSValue js_contour_new(JSContext* ctx, const std::vector<cv::Point_<Type>>& points);

template<> JSValue js_contour_new<double>(JSContext* ctx, const std::vector<cv::Point_<double>>& points);

template<> JSValue js_contour_new<float>(JSContext* ctx, const std::vector<cv::Point_<float>>& points);

template<> JSValue js_contour_new<int>(JSContext* ctx, const std::vector<cv::Point_<int>>& points);

#define countof(x) (sizeof(x) / sizeof((x)[0]))

static inline int32_t
js_array_length(JSContext* ctx, const JSValueConst& arr) {
  int32_t ret = -1;
  if(JS_IsArray(ctx, arr)) {
    JSValue v = JS_GetPropertyStr(ctx, arr, "length");
    JS_ToInt32(ctx, &ret, v);
    JS_FreeValue(ctx, v);
  }
  return ret;
}

class js_array_iterator : public std::iterator<std::input_iterator_tag, JSValue> {
public:
  js_array_iterator(JSContext* c, const JSValueConst& a, const size_t i = 0) : ctx(c), array(&a), pos(i) {}
  value_type
  operator*() const {
    return JS_GetPropertyUint32(ctx, *array, pos);
  }
  js_array_iterator&
  operator++() {
    ++this->pos;
    return *this;
  }
  js_array_iterator
  operator++(int) {
    js_array_iterator temp(*this);
    ++(*this);
    return temp;
  }
  bool
  operator==(const js_array_iterator& rhs) {
    return array == rhs.array && pos == rhs.pos;
  }
  bool
  operator!=(const js_array_iterator& rhs) {
    return !operator==(rhs);
  }

private:
  JSContext* ctx;
  const JSValueConst* array;
  difference_type pos;
};

static inline js_array_iterator
js_begin(JSContext* c, const JSValueConst& a) {
  return js_array_iterator(c, a, 0);
}
static inline js_array_iterator
js_end(JSContext* c, const JSValueConst& a) {
  return js_array_iterator(c, a, js_array_length(c, a));
}

template<class T>
JSValue
js_contours_new(JSContext* ctx, const std::vector<std::vector<cv::Point_<T>>>& contours) {

  JSValue ret = JS_NewArray(ctx);
  uint32_t i, size = contours.size();

  for(i = 0; i < size; i++) {
    JSValue contour = js_contour_new(ctx, contours[i]);
    JS_SetPropertyUint32(ctx, ret, i, contour);
  }

  return ret;
}

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

/*static inline JSValue
js_point_create(JSContext* ctx, double x, double y) {

  JSValue point = js_new(ctx, "Point");

  JS_SetPropertyStr(ctx, point, "x", JS_NewFloat64(ctx, x));
  JS_SetPropertyStr(ctx, point, "y", JS_NewFloat64(ctx, y));
  return point;
}*/

extern "C" JSValue js_point_clone(JSContext* ctx, const JSPointData& point);

static inline int
js_rect_read(JSContext* ctx, JSValueConst rect, JSRectData* out) {
  int ret = 1;
  JSValue x, y, w, h;
  if(JS_IsArray(ctx, rect)) {
    x = JS_GetPropertyUint32(ctx, rect, 0);
    y = JS_GetPropertyUint32(ctx, rect, 1);
    w = JS_GetPropertyUint32(ctx, rect, 2);
    h = JS_GetPropertyUint32(ctx, rect, 3);

  } else {
    x = JS_GetPropertyStr(ctx, rect, "x");
    y = JS_GetPropertyStr(ctx, rect, "y");
    w = JS_GetPropertyStr(ctx, rect, "width");
    h = JS_GetPropertyStr(ctx, rect, "height");
  }
  if(JS_IsNumber(x) && JS_IsNumber(y) && JS_IsNumber(w) && JS_IsNumber(h)) {
    ret &= !JS_ToFloat64(ctx, &out->x, x);
    ret &= !JS_ToFloat64(ctx, &out->y, y);
    ret &= !JS_ToFloat64(ctx, &out->width, w);
    ret &= !JS_ToFloat64(ctx, &out->height, h);
  }
  JS_FreeValue(ctx, x);
  JS_FreeValue(ctx, y);
  JS_FreeValue(ctx, w);
  JS_FreeValue(ctx, h);
  return ret;
}

static JSRectData
js_rect_get(JSContext* ctx, JSValueConst rect) {
  JSRectData r = {0, 0, 0, 0};
  js_rect_read(ctx, rect, &r);
  return r;
}

static inline int
js_rect_write(JSContext* ctx, JSValue out, JSRectData rect) {
  int ret = 0;
  ret += JS_SetPropertyStr(ctx, out, "x", JS_NewFloat64(ctx, rect.x));
  ret += JS_SetPropertyStr(ctx, out, "y", JS_NewFloat64(ctx, rect.y));
  ret += JS_SetPropertyStr(ctx, out, "width", JS_NewFloat64(ctx, rect.width));
  ret += JS_SetPropertyStr(ctx, out, "height", JS_NewFloat64(ctx, rect.height));
  return ret;
}

static JSRectData
js_rect_set(JSContext* ctx, JSValue out, double x, double y, double w, double h) {
  const JSRectData r = {x, y, w, h};
  js_rect_write(ctx, out, r);
  return r;
}

static inline int
js_size_read(JSContext* ctx, JSValueConst size, JSSizeData* out) {
  int ret = 1;
  JSValue w, h;

  if(JS_IsArray(ctx, size)) {
    w = JS_GetPropertyUint32(ctx, size, 0);
    h = JS_GetPropertyUint32(ctx, size, 1);
  } else if(JS_IsObject(size)) {
    w = JS_GetPropertyStr(ctx, size, "width");
    h = JS_GetPropertyStr(ctx, size, "height");
  }
  if(JS_IsNumber(w) && JS_IsNumber(h)) {
    ret &= !JS_ToFloat64(ctx, &out->width, w);
    ret &= !JS_ToFloat64(ctx, &out->height, h);
  } else {
    ret = 0;
  }
  JS_FreeValue(ctx, w);
  JS_FreeValue(ctx, h);
  return ret;
}

static inline JSSizeData
js_size_get(JSContext* ctx, JSValueConst size) {
  JSSizeData r = {0, 0};
  js_size_read(ctx, size, &r);
  return r;
}

static inline int
js_point_read(JSContext* ctx, JSValueConst point, JSPointData* out) {
  int ret = 1;
  JSValue x, y;
  if(JS_IsArray(ctx, point)) {
    x = JS_GetPropertyUint32(ctx, point, 0);
    y = JS_GetPropertyUint32(ctx, point, 1);
  } else if(JS_IsObject(point)) {
    x = JS_GetPropertyStr(ctx, point, "x");
    y = JS_GetPropertyStr(ctx, point, "y");
  }
  if(JS_IsNumber(x) && JS_IsNumber(y)) {
    ret &= !JS_ToFloat64(ctx, &out->x, x);
    ret &= !JS_ToFloat64(ctx, &out->y, y);
  } else {
    ret = 0;
  }
  JS_FreeValue(ctx, x);
  JS_FreeValue(ctx, y);
  return ret;
}

static JSPointData
js_point_get(JSContext* ctx, JSValueConst point) {
  JSPointData r; /*, *ptr;
   if((ptr = js_point_data(ctx, point)) != nullptr)
     r = *ptr;
   else*/
  js_point_read(ctx, point, &r);
  return r;
}

static inline bool
js_is_point(JSContext* ctx, JSValueConst point) {
  JSPointData r;

  if(js_point_data(ctx, point))
    return true;

  if(js_point_read(ctx, point, &r))
    return true;

  return false;
}

extern "C" int js_color_read(JSContext* ctx, JSValueConst color, JSColorData* out);

static inline int
js_contour_read(JSContext* ctx, JSValueConst contour, JSContourData* out) {
  int ret = 0;
  return ret;
}

static JSContourData
js_contour_get(JSContext* ctx, JSValueConst contour) {
  JSContourData r = {};
  js_contour_read(ctx, contour, &r);
  return r;
}

template<class T> class js_array {
public:
  static int64_t
  to_vector(JSContext* ctx, JSValueConst arr, std::vector<T>& out) {
    int64_t i, n;
    JSValue len;
    if(!JS_IsArray(ctx, arr))
      return -1;
    len = JS_GetPropertyStr(ctx, arr, "length");
    JS_ToInt64(ctx, &n, len);
    out.reserve(out.size() + n);
    for(i = 0; i < n; i++) {
      double value;
      JSValue item = JS_GetPropertyUint32(ctx, arr, (uint32_t)i);
      if(JS_ToFloat64(ctx, &value, item) == -1) {
        JS_FreeValue(ctx, item);
        out.clear();
        return -1;
      }
      out.push_back(value);
      JS_FreeValue(ctx, item);
    }
    return n;
  }

  template<class Container>

  static JSValue
  from(JSContext* ctx, const Container& in) {
    return from_sequence<typename Container::const_iterator>(ctx, in.cbegin(), in.cend());
  }

  static JSValue
  from_vector(JSContext* ctx, const std::vector<T>& in) {
    return from_sequence(ctx, in.cbegin(), in.cend());
  }

  template<class Iterator>
  static JSValue
  from_sequence(JSContext* ctx, const Iterator& start, const Iterator& end) {
    JSValue arr = JS_NewArray(ctx);
    size_t i = 0;
    for(Iterator it = start; it != end; ++it) {
      JSValue item = JS_NewFloat64(ctx, *it);
      JS_SetPropertyUint32(ctx, arr, i, item);
      ++i;
    }
    return arr;
  }

  template<size_t N> static int64_t to_array(JSContext* ctx, JSValueConst arr, std::array<T, N>& out);
  static int64_t to_scalar(JSContext* ctx, JSValueConst arr, cv::Scalar_<T>& out);
};

template<class T>
template<size_t N>
int64_t
js_array<T>::to_array(JSContext* ctx, JSValueConst arr, std::array<T, N>& out) {
  std::vector<T> tmp;
  to_vector(ctx, arr, tmp);
  if(tmp.size() < N)
    return -1;
  for(size_t i = 0; i < N; i++) out[i] = tmp[i];
  return N;
}

template<class T>
int64_t
js_array<T>::to_scalar(JSContext* ctx, JSValueConst arr, cv::Scalar_<T>& out) {
  size_t n;
  std::vector<T> tmp;
  to_vector(ctx, arr, tmp);
  if((n = tmp.size()) < 4)
    tmp.resize(4);
  for(size_t i = 0; i < 4; i++) out[i] = tmp[i];
  return n;
}

template<class T> class js_array<cv::Point_<T>> {
public:
  static int64_t
  to_vector(JSContext* ctx, JSValueConst arr, std::vector<cv::Point_<T>>& out) {
    int64_t i, n;
    JSValue len;
    if(!JS_IsArray(ctx, arr))
      return -1;
    len = JS_GetPropertyStr(ctx, arr, "length");
    JS_ToInt64(ctx, &n, len);
    out.reserve(out.size() + n);
    for(i = 0; i < n; i++) {
      JSPointData value;
      JSValue item = JS_GetPropertyUint32(ctx, arr, (uint32_t)i);
      if(!js_point_read(ctx, item, &value)) {
        JS_FreeValue(ctx, item);
        out.clear();
        return -1;
      }
      out.push_back(value);
      JS_FreeValue(ctx, item);
    }
    return n;
  }
  template<size_t N> static int64_t to_array(JSContext* ctx, JSValueConst arr, std::array<cv::Mat, N>& out);
};

template<> class js_array<cv::Mat> {
public:
  static int64_t
  to_vector(JSContext* ctx, JSValueConst arr, std::vector<cv::Mat>& out) {
    int64_t i, n;
    JSValue len;
    if(!JS_IsArray(ctx, arr))
      return -1;
    len = JS_GetPropertyStr(ctx, arr, "length");
    JS_ToInt64(ctx, &n, len);
    out.reserve(out.size() + n);
    for(i = 0; i < n; i++) {
      JSMatData* value;
      JSValue item = JS_GetPropertyUint32(ctx, arr, (uint32_t)i);
      value = js_mat_data(ctx, item);
      if(value == nullptr) {
        JS_FreeValue(ctx, item);
        out.clear();
        return -1;
      }
      out.push_back(*value);
      JS_FreeValue(ctx, item);
    }
    return n;
  }
  template<size_t N> static int64_t to_array(JSContext* ctx, JSValueConst arr, std::array<cv::Mat, N>& out);
};

template<class T> class js_array<std::vector<T>> {
public:
  static int64_t
  to_vector(JSContext* ctx, JSValueConst arr, std::vector<std::vector<T>>& out) {
    int64_t i, n;
    JSValue len;
    if(!JS_IsArray(ctx, arr))
      return -1;
    len = JS_GetPropertyStr(ctx, arr, "length");
    JS_ToInt64(ctx, &n, len);
    out.reserve(out.size() + n);
    for(i = 0; i < n; i++) {
      std::vector<T> value;
      JSValue item = JS_GetPropertyUint32(ctx, arr, (uint32_t)i);
      if(js_array<T>::to_vector(ctx, arr, value) == -1) {
        JS_FreeValue(ctx, item);
        out.clear();
        return -1;
      }
      out.push_back(value);
      JS_FreeValue(ctx, item);
    }
    return n;
  }

  template<size_t N> static int64_t to_array(JSContext* ctx, JSValueConst arr, std::array<std::vector<T>, N>& out);
};

template<class T>
inline int64_t
js_array_to_vector(JSContext* ctx, JSValueConst arr, std::vector<T>& out) {
  return js_array<T>::to_vector(ctx, arr, out);
}

template<class T, size_t N>
inline int64_t
js_array_to_array(JSContext* ctx, JSValueConst arr, std::array<T, N>& out) {
  return js_array<T>::to_array<N>(ctx, arr, out);
}

template<class T>
inline int64_t
js_array_to_scalar(JSContext* ctx, JSValueConst arr, cv::Scalar_<T>& out) {
  return js_array<T>::to_scalar(ctx, arr, out);
}

template<class Iterator>
inline JSValue
js_array_from(JSContext* ctx, const Iterator& start, const Iterator& end) {
  return js_array<typename Iterator::value_type>::from_sequence(ctx, start, end);
}

template<class Container>
inline JSValue
js_array_from(JSContext* ctx, const Container& v) {
  return js_array<typename Container::value_type>::from_sequence(ctx, v.cbegin(), v.cend());
}

class js_object {
public:
  template<class T>
  static int64_t
  to_map(JSContext* ctx, JSValueConst obj, std::map<std::string, T>& out) {
    int64_t i = 0;
    jsrt js(ctx);
    auto names = js.property_names(obj);
    for(auto name : names) {
      T prop = js.to<T>(js.get_property(obj, name));
      out[name] = prop;
      ++i;
    }
    return i;
  }

  template<class T>
  static JSValue
  from_map(JSContext* ctx, const std::map<std::string, T>& in) {
    typedef std::pair<std::string, T> entry_type;
    jsrt js(ctx);
    JSValue obj = JS_NewObject(ctx);
    ;

    for(entry_type entry : in) js.set_property(obj, entry.first, js.create<T>(entry.second), JS_PROP_C_W_E);

    return obj;
  }
};

#endif
