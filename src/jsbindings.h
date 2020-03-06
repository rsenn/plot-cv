#ifndef JSBINDINGS_H
#define JSBINDINGS_H
#include "js.h"

extern "C" {
JSValue js_draw_line(JSContext*, jsrt::const_value, int, jsrt::const_value*);
JSValue js_draw_rect(JSContext*, jsrt::const_value, int, jsrt::const_value*);
JSValue js_draw_contour(JSContext*, jsrt::const_value, int, jsrt::const_value*);
JSValue js_draw_polygon(JSContext*, jsrt::const_value, int, jsrt::const_value*);
JSValue js_draw_circle(JSContext*, jsrt::const_value, int, jsrt::const_value*);

void js_point_finalizer(JSRuntime* rt, JSValue val);
JSValue js_point_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv);
JSValue js_point_get_xy(JSContext* ctx, JSValueConst this_val, int magic);
JSValue js_point_set_xy(JSContext* ctx, JSValueConst this_val, JSValue val, int magic);
JSValue js_point_norm(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv);
JSValue js_point_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv);
int js_point_init(JSContext* ctx, void* m, const char* name, bool exp = true);

int js_size_init(JSContext* ctx, void* m, const char* name, bool exp = true);

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
}
extern "C" JSValue contour_proto;
extern "C" JSClassID js_contour_class_id;

typedef cv::Point2d JSPointData;

typedef std::vector<JSPointData> JSContourData;

template<class T>
inline JSValue
js_contour_new(JSContext* ctx, const std::vector<cv::Point_<T>>& points) {
  JSValue ret;
  JSContourData* contour;

  ret = JS_NewObjectProtoClass(ctx, contour_proto, js_contour_class_id);

  contour = static_cast<JSContourData*>(js_mallocz(ctx, sizeof(JSContourData)));

  std::transform(points.cbegin(),
                 points.cend(),
                 std::back_inserter(*contour),
                 [](const cv::Point_<T>& pt) -> cv::Point2d { return cv::Point2d(pt.x, pt.y); });

  JS_SetOpaque(ret, contour);
  return ret;
};

#endif