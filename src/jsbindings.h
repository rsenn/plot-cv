#ifndef JSBINDINGS_H
#define JSBINDINGS_H
#include "js.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iomanip>
#include <map>

typedef cv::Rect2d JSRectData;
typedef cv::Mat JSMatData;
typedef cv::Size2d JSSizeData;
typedef cv::Point2d JSPointData;
typedef cv::Vec4d JSLineData;
typedef std::vector<JSPointData> JSContourData;
typedef std::pair<JSPointData*, JSPointData*> JSPointIteratorData;

#define VISIBLE
#define HIDDEN __attribute__((visibility("hidden")))

extern "C" {

int js_init(int argc, char*[]);

int js_draw_functions(JSContext* ctx, JSValue parent);
int js_draw_init(JSContext*, JSModuleDef*);

JSValue js_point_new(JSContext*, double x, double y);
JSPointData* js_point_data(JSContext*, JSValue val);

int js_point_init(JSContext*, JSModuleDef* m);
void js_point_constructor(JSContext* ctx, JSValue parent, const char* name);

JSModuleDef* js_init_point_module(JSContext*, const char* module_name);

JSSizeData* js_size_data(JSContext*, JSValue val);

int js_size_init(JSContext*, JSModuleDef* m);
JSModuleDef* js_init_size_module(JSContext*, const char* module_name);
void js_size_constructor(JSContext* ctx, JSValue parent, const char* name);

JSRectData* js_rect_data(JSContext*, JSValue val);

int js_rect_init(JSContext*, JSModuleDef*);
JSModuleDef* js_init_rect_module(JSContext*, const char* module_name);

void js_rect_constructor(JSContext* ctx, JSValue parent, const char* name);

int js_point_iterator_init(JSContext*, JSModuleDef* m);
JSModuleDef* js_init_point_iterator_module(JSContext*, const char* module_name);
void js_point_iterator_constructor(JSContext* ctx, JSValue parent, const char* name);

JSContourData* js_contour_data(JSContext*, JSValue val);
void js_contour_finalizer(JSRuntime* rt, JSValue val);

JSValue js_contour_to_string(JSContext*, JSValueConst this_val, int argc, JSValueConst* argv);
int js_contour_init(JSContext*, JSModuleDef*);
JSModuleDef* js_init_contour_module(JSContext* ctx, const char* module_name);
void js_contour_constructor(JSContext* ctx, JSValue parent, const char* name);

JSValue js_mat_wrap(JSContext*, const cv::Mat& mat);

int js_mat_init(JSContext*, JSModuleDef*);
JSModuleDef* js_init_mat_module(JSContext* ctx, const char* module_name);
void js_mat_constructor(JSContext* ctx, JSValue parent, const char* name);

JSValue js_mat_wrap(JSContext* ctx, const cv::Mat& mat);
JSMatData* js_mat_data(JSContext* ctx, JSValue val);

JSModuleDef* js_init_module(JSContext* ctx, const char* module_name);
JSModuleDef* js_init_module_point(JSContext*, const char*);
JSModuleDef* js_init_module_point_iterator(JSContext*, const char*);
JSModuleDef* js_init_module_size(JSContext*, const char*);
JSModuleDef* js_init_module_rect(JSContext*, const char*);
JSModuleDef* js_init_module_mat(JSContext*, const char*);
JSModuleDef* js_init_module_contour(JSContext*, const char*);
JSModuleDef* js_init_module_line(JSContext*, const char*);
JSModuleDef* js_init_module_draw(JSContext*, const char*);

extern JSValue contour_class, contour_proto, int32array_ctor, int32array_proto, mat_class, mat_proto, point_class, line_class, point_iterator_class, draw_class, point_iterator_proto, point_proto,
    rect_class, rect_proto, size_class, size_proto, line_proto, draw_proto;

JSValue js_create_point_iterator(JSContext*, JSValueConst this_val, int argc, JSValueConst* argv, int magic);

extern cv::Mat* dptr;
}
extern "C" JSValue contour_proto;
extern "C" JSClassDef js_contour_class, js_size_class, js_point_class, js_mat_class, js_rect_class;
extern "C" JSClassID js_contour_class_id;
extern "C" JSClassID js_point_iterator_class_id, js_line_class_id, js_draw_class_id;

extern "C" const JSCFunctionListEntry js_rect_proto_funcs[];

extern "C" JSClassID js_point_class_id, js_size_class_id, js_rect_class_id, js_mat_class_id;

extern "C" JSValue js_contour2d_new(JSContext*, const std::vector<cv::Point_<double>>& points);
extern "C" JSValue js_contour2f_new(JSContext*, const std::vector<cv::Point_<float>>& points);
extern "C" JSValue js_contour2i_new(JSContext*, const std::vector<cv::Point_<int>>& points);

template<class Type> JSValue js_contour_new(JSContext* ctx, const std::vector<cv::Point_<Type>>& points);

template<> JSValue js_contour_new<double>(JSContext* ctx, const std::vector<cv::Point_<double>>& points);

template<> JSValue js_contour_new<float>(JSContext* ctx, const std::vector<cv::Point_<float>>& points);

template<> JSValue js_contour_new<int>(JSContext* ctx, const std::vector<cv::Point_<int>>& points);

#define countof(x) (sizeof(x) / sizeof((x)[0]))

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

template<class Value> int64_t js_array_to_vector(JSContext*, JSValue arr, std::vector<Value>& out);

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

static inline JSValue
js_point_create(JSContext* ctx, double x, double y) {

  JSValue point = js_new(ctx, "Point");

  JS_SetPropertyStr(ctx, point, "x", JS_NewFloat64(ctx, x));
  JS_SetPropertyStr(ctx, point, "y", JS_NewFloat64(ctx, y));
  return point;
}

static inline int
js_rect_read(JSContext* ctx, JSValueConst rect, JSRectData* out) {
  int ret = 0;
  ret += JS_ToFloat64(ctx, &out->x, JS_GetPropertyStr(ctx, rect, "x"));
  ret += JS_ToFloat64(ctx, &out->y, JS_GetPropertyStr(ctx, rect, "y"));
  ret += JS_ToFloat64(ctx, &out->width, JS_GetPropertyStr(ctx, rect, "width"));
  ret += JS_ToFloat64(ctx, &out->height, JS_GetPropertyStr(ctx, rect, "height"));
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
  int ret = 0;
  ret += JS_ToFloat64(ctx, &out->width, JS_GetPropertyStr(ctx, size, "width"));
  ret += JS_ToFloat64(ctx, &out->height, JS_GetPropertyStr(ctx, size, "height"));
  return ret;
}

static JSSizeData
js_size_get(JSContext* ctx, JSValueConst size) {
  JSSizeData r = {0, 0};
  js_size_read(ctx, size, &r);
  return r;
}

static inline int
js_point_read(JSContext* ctx, JSValueConst point, JSPointData* out) {
  int ret = 0;
  ret += JS_ToFloat64(ctx, &out->x, JS_GetPropertyStr(ctx, point, "x"));
  ret += JS_ToFloat64(ctx, &out->y, JS_GetPropertyStr(ctx, point, "y"));
  return ret;
}

static JSPointData
js_point_get(JSContext* ctx, JSValueConst point) {
  JSPointData r = {0, 0};
  js_point_read(ctx, point, &r);
  return r;
}

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
#endif
