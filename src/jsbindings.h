#ifndef JSBINDINGS_H
#define JSBINDINGS_H
#include "js.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iomanip>

typedef cv::Rect2d JSRectData;
typedef cv::Mat JSMatData;
typedef cv::Size2d JSSizeData;

typedef cv::Point2d JSPointData;

typedef std::vector<JSPointData> JSContourData;

struct JSPointIteratorData {
  cv::Point2d *begin, *end;
};

extern "C" {

int js_init(int argc, char*[]);

JSValue js_draw_line(JSContext*, jsrt::const_value, int, jsrt::const_value*);
JSValue js_draw_rect(JSContext*, jsrt::const_value, int, jsrt::const_value*);
JSValue js_draw_contour(JSContext*, jsrt::const_value, int, jsrt::const_value*);
JSValue js_draw_polygon(JSContext*, jsrt::const_value, int, jsrt::const_value*);
JSValue js_draw_circle(JSContext*, jsrt::const_value, int, jsrt::const_value*);

void js_point_finalizer(JSRuntime* rt, JSValue val);
JSPointData* js_point_data(JSContext* ctx, JSValue val);
JSValue js_point_new(JSContext* ctx, double x, double y);
JSValue js_point_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv);
JSValue js_point_get_xy(JSContext* ctx, JSValueConst this_val, int magic);
JSValue js_point_set_xy(JSContext* ctx, JSValueConst this_val, JSValue val, int magic);
JSValue js_point_norm(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv);
JSValue js_point_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv);
int js_point_init(JSContext* ctx, void* m, const char* name, bool exp = true);

void js_size_finalizer(JSRuntime* rt, JSValue val);
JSValue js_size_new(JSContext* ctx, double w, double h);
JSSizeData* js_size_data(JSContext* ctx, JSValue val);
JSValue js_size_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv);
JSValue js_size_get_wh(JSContext* ctx, JSValueConst this_val, int magic);
JSValue js_size_set_wh(JSContext* ctx, JSValueConst this_val, JSValue val, int magic);
JSValue js_size_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv);
int js_size_init(JSContext* ctx, void* m, const char* name, bool exp);

void js_mat_finalizer(JSRuntime* rt, JSValue val);
JSValue js_mat_new(JSContext* ctx, int cols = 0, int rows = 0, int type = CV_32FC1);
JSValue js_mat_wrap(JSContext* ctx, const cv::Mat& mat);
JSMatData* js_mat_data(JSContext* ctx, JSValue val);
JSValue js_mat_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv);
JSValue js_mat_get_props(JSContext* ctx, JSValueConst this_val, int magic);
JSValue
js_mat_funcs(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic);
JSValue js_mat_findcontours(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv);
JSValue js_mat_tostring(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv);
int js_mat_init(JSContext* ctx, void* m, const char* name, bool exp);

JSRectData* js_rect_data(JSContext* ctx, JSValue val);
void js_rect_finalizer(JSRuntime* rt, JSValue val);
JSValue js_rect_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv);
JSValue js_rect_get_xy(JSContext* ctx, JSValueConst this_val, int magic);
JSValue js_rect_set_xy(JSContext* ctx, JSValueConst this_val, JSValue val, int magic);
JSValue js_rect_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv);
int js_rect_init(JSContext* ctx, void* m, const char* name, bool exp = true);

JSModuleDef* js_init_module(JSContext* ctx, const char* module_name);
void js_point_iterator_finalizer(JSRuntime* rt, JSValue val);
JSValue
js_point_iterator_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv);
JSValue
js_point_iterator_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv);
int js_point_iterator_init(JSContext* ctx, void* m, const char* name, bool exp = true);
void js_contour_finalizer(JSRuntime* rt, JSValue val);
JSValue js_contour_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv);
JSValue js_contour_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv);
int js_contour_init(JSContext* ctx, void* m, const char* name, bool exp = true);

JSValue js_mat_wrap(JSContext* ctx, const cv::Mat& mat);
void js_mat_finalizer(JSRuntime* rt, JSValue val);
JSValue js_mat_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv);
JSValue js_mat_get_wh(JSContext* ctx, JSValueConst this_val, int magic);
int js_mat_init(JSContext* ctx, void* m, const char* name, bool exp);

JSContourData* js_contour_data(JSContext* ctx, JSValue val);

extern JSValue contour_class, contour_proto, int32array_ctor, int32array_proto, mat_class,
    mat_proto, point_class, point_iterator_class, point_iterator_proto, point_proto, rect_class,
    rect_proto, size_class, size_proto;

JSValue js_create_point_iterator(
    JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic);
}
extern "C" JSValue contour_proto;
extern "C" JSClassDef js_contour_class, js_size_class, js_point_class, js_mat_class, js_rect_class;
extern "C" JSClassID js_contour_class_id;
extern "C" JSClassID js_point_iterator_class_id;

extern "C" const JSCFunctionListEntry js_rect_proto_funcs[];

// extern "C" JSCFunctionListEntry js_contour_proto_funcs[];
extern "C" JSClassID js_point_class_id, js_size_class_id, js_rect_class_id, js_mat_class_id;

JSValue js_contour2i_new(JSContext* ctx, const std::vector<cv::Point_<int>>& points);
JSValue js_contour_new(JSContext* ctx, const std::vector<cv::Point_<float>>& points);
extern "C" JSValue js_contour2d_new(JSContext* ctx, const std::vector<cv::Point_<double>>& points);

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

JSValue js_vector_vec4i_to_array(JSContext* ctx, const std::vector<cv::Vec4i>& vec);

template<class Value>
int64_t js_array_to_vector(JSContext* ctx, JSValue arr, std::vector<Value>& out);

#endif