#include "color.h"
#include "geometry.h"
#include "jsbindings.h"
#include "plot-cv.h"
#include "psimpl.h"
#include "geometry.h"
#include "quickjs/cutils.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

extern jsrt js;

extern "C" {
cv::Mat* dptr = nullptr;

JSValue
js_draw_line(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {
  int i = 0, ret = -1;
  point2f_type points[2];
  color_type color;
  int thickness = 1;
  bool antialias = true;

  if(argc > i && js.is_point(argv[i]))
    js.get_point(argv[i++], points[0]);

  if(argc > i && js.is_point(argv[1]))
    js.get_point(argv[i++], points[1]);

  if(argc > i && js.is_color(argv[i]))
    js.get_color(argv[i++], color);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  if(argc > i && js.is_boolean(argv[i]))
    js.get_boolean(argv[i++], antialias);

  cv::line(*dptr, points[0], points[1], color, thickness, antialias ? cv::LINE_AA : cv::LINE_8);
  return js._true;
}

JSValue
js_draw_rect(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {
  int i = 0, ret = -1;
  cv::Rect2f rect;
  point2f_type points[2];
  color_type color;
  int thickness = 1;
  bool antialias = true;

  if(argc > i && js.is_rect(argv[i]))
    js.get_rect(argv[i++], rect);

  if(argc > i && js.is_color(argv[i]))
    js.get_color(argv[i++], color);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  if(argc > i && js.is_boolean(argv[i]))
    js.get_boolean(argv[i++], antialias);

  points[0].x = rect.x;
  points[0].y = rect.y;
  points[1].x = rect.x + rect.width;
  points[1].y = rect.y + rect.height;

  cv::rectangle(
      *dptr, points[0], points[1], color, thickness, antialias ? cv::LINE_AA : cv::LINE_8);

  return js._true;
}

JSValue
js_draw_contour(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {

  int i = 0, ret = -1;
  contour2i_vector points;
  color_type color;
  int thickness = 1;
  bool antialias = true;

  points.resize(1);
  if(argc > i && js.is_array(argv[i]))
    js.get_point_array(argv[i++], points[0]);

  if(argc > i && js.is_color(argv[i]))
    js.get_color(argv[i++], color);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  if(argc > i && js.is_boolean(argv[i]))
    js.get_boolean(argv[i++], antialias);

  if(dptr != nullptr)
    cv::drawContours(*dptr, points, -1, color, thickness, antialias ? cv::LINE_AA : cv::LINE_8);

  logfile << "draw_contour() ret:" << ret << " color: " << color << std::endl;
  return js._true;
}

JSValue
js_draw_polygon(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {

  int i = 0, ret = -1;
  point2i_vector points;
  cv::Scalar color;
  bool antialias = true;
  int thickness = -1;

  if(argc > i && js.is_array_like(argv[i]))
    js.get_point_array(argv[i++], points);

  if(argc > i && js.is_color(argv[i]))
    js.get_color(argv[i++], color);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  if(argc > i && js.is_boolean(argv[i]))
    js.get_boolean(argv[i++], antialias);

  if(dptr != nullptr) {
    const int size = points.size();
    int lineType = antialias ? cv::LINE_AA : cv::LINE_8;
    const point2i_type* pts = points.data();

    std::cerr << "drawPolygon() points: " << (points) << " color: " << to_string(color)
              << std::endl;

    // cv::fillPoly(*dptr, points, color, antialias ? cv::LINE_AA : cv::LINE_8);
    (thickness <= 0 ? cv::fillPoly(*dptr, &pts, &size, 1, color, lineType)
                    : cv::polylines(*dptr, &pts, &size, 1, true, color, thickness, lineType));
  }
  return js._true;
}

JSValue
js_draw_circle(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {

  int i = 0, ret = -1;
  point2i_type point;
  int radius = 0;
  cv::Scalar color;

  bool antialias = true;
  int thickness = -1;

  if(argc > i && js.is_point(argv[i]))
    js.get_point(argv[i++], point);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], radius);

  if(argc > i && js.is_color(argv[i]))
    js.get_color(argv[i++], color);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  if(argc > i && js.is_boolean(argv[i]))
    js.get_boolean(argv[i++], antialias);

  if(dptr != nullptr) {
    int lineType = antialias ? cv::LINE_AA : cv::LINE_8;

    logfile << "drawCircle() center: " << (point) << " radius: " << radius
            << " color: " << to_string(color) << std::endl;

    // cv::fillPoly(*dptr, points, color, antialias ? cv::LINE_AA : cv::LINE_8);
    cv::circle(*dptr, point, radius, color, thickness < 0 ? cv::FILLED : thickness, lineType);
  }
  return js._true;
}
}

template<class Value>
int64_t js_array_to_vector(JSContext* ctx, JSValue arr, std::vector<Value>& out);

template<>
int64_t
js_array_to_vector(JSContext* ctx, JSValue arr, std::vector<int>& out) {
  int64_t i, n;
  JSValue len = JS_GetPropertyStr(ctx, arr, "length");
  JS_ToInt64(ctx, &n, len);
  out.resize(n);
  for(i = 0; i < n; i++) {
    int32_t value;
    JSValue item = JS_GetPropertyUint32(ctx, arr, (uint32_t)i);
    JS_ToInt32(ctx, &value, item);
    out[i] = value;
  }
  return n;
}

template<class Value> JSValue js_vector_to_array(JSContext* ctx, const std::vector<Value>& vec);

template<>
JSValue
js_vector_to_array(JSContext* ctx, const std::vector<int>& vec) {
  JSValue ret = JS_NewArray(ctx);
  uint32_t i, n = vec.size();
  for(i = 0; i < n; i++) {
    JS_SetPropertyUint32(ctx, ret, i, JS_NewInt32(ctx, vec[i]));
  }
  return ret;
}

template<class Vector>
JSValue
js_vector_to_array(std::enable_if_t<std::is_same<Vector, cv::Vec4i>::value, JSContext*> ctx,
                   const std::vector<Vector>& vec) {
  JSValue ret = JS_NewArray(ctx);
  uint32_t i, j, n = vec.size();
  for(i = 0; i < n; i++) {
    JSValue item = JS_NewArray(ctx);
    for(j = 0; j < 4; j++) {
      JS_SetPropertyUint32(ctx, item, j, JS_NewInt32(ctx, vec[i][j]));
    }
    JS_SetPropertyUint32(ctx, ret, i, item);
  }
  return ret;
}

JSValue
js_vector_to_array(JSContext* ctx, const std::vector<cv::Vec4i>& vec) {
  JSValue ret = JS_NewArray(ctx);
  uint32_t i, j, n = vec.size();
  for(i = 0; i < n; i++) {
    JSValue item = JS_NewArray(ctx);
    for(j = 0; j < 4; j++) {
      JS_SetPropertyUint32(ctx, item, j, JS_NewInt32(ctx, vec[i][j]));
    }
    JS_SetPropertyUint32(ctx, ret, i, item);
  }
  return ret;
}

JSValue
js_vector_to_array(JSContext* ctx, const std::vector<cv::Point_<float>>& vec) {
  JSValue ret = JS_NewArray(ctx);
  uint32_t i, n = vec.size();
  for(i = 0; i < n; i++) {
    JSValue item = js_point_new(ctx, vec[i].x, vec[i].y);

    JS_SetPropertyUint32(ctx, ret, i, item);
  }
  return ret;
}

JSValue
js_vector_to_array(JSContext* ctx, const std::vector<std::vector<cv::Point2d>>& contours) {
  JSValue ret = JS_NewArray(ctx);
  uint32_t i, size = contours.size();

  for(i = 0; i < size; i++) {
    JS_SetPropertyUint32(ctx, ret, i, js_contour_new(ctx, contours[i]));
  }
  return ret;
}

extern "C" {
#define countof(x) (sizeof(x) / sizeof((x)[0]))

/* Point Class */

typedef cv::Rect2d JSRectData;
typedef cv::Mat JSMatData;
typedef cv::Size2d JSSizeData;

JSClassID js_point_class_id, js_size_class_id, js_rect_class_id, js_mat_class_id;

JSValue point_proto, point_class;
JSValue rect_proto, rect_class;
JSValue size_proto, size_class;
JSValue point_iterator_proto, point_iterator_class;
JSValue contour_proto, contour_class;
JSValue mat_proto, mat_class;

JSRectData* js_rect_data(JSContext* ctx, JSValue val);

JSValue js_mat_wrap(JSContext* ctx, const cv::Mat& mat);

extern "C++" template<class Type>
JSValue js_contour_new(JSContext* ctx, const std::vector<Type>& points);

void
js_point_finalizer(JSRuntime* rt, JSValue val) {
  JSPointData* s = static_cast<JSPointData*>(JS_GetOpaque(val, js_point_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */
  js_free_rt(rt, s);
}

JSValue
js_point_new(JSContext* ctx, double x, double y) {
  JSValue ret;
  JSPointData* s;

  ret = JS_NewObjectProtoClass(ctx, point_proto, js_point_class_id);

  s = static_cast<JSPointData*>(js_mallocz(ctx, sizeof(JSPointData)));
  s->x = x;
  s->y = y;

  JS_SetOpaque(ret, s);
  return ret;
}

JSPointData*
js_point_data(JSContext* ctx, JSValue val) {
  return static_cast<JSPointData*>(JS_GetOpaque2(ctx, val, js_point_class_id));
}

JSValue
js_point_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSPointData* s;
  JSValue obj = JS_UNDEFINED;
  JSValue proto;

  s = static_cast<JSPointData*>(js_mallocz(ctx, sizeof(*s)));
  if(!s)
    return JS_EXCEPTION;
  if(JS_ToFloat64(ctx, &s->x, argv[0]))
    goto fail;
  if(JS_ToFloat64(ctx, &s->y, argv[1]))
    goto fail;
  /* using new_target to get the prototype is necessary when the
     class is extended. */
  proto = JS_GetPropertyStr(ctx, new_target, "prototype");
  if(JS_IsException(proto))
    goto fail;
  obj = JS_NewObjectProtoClass(ctx, proto, js_point_class_id);
  JS_FreeValue(ctx, proto);
  if(JS_IsException(obj))
    goto fail;
  JS_SetOpaque(obj, s);
  return obj;
fail:
  js_free(ctx, s);
  JS_FreeValue(ctx, obj);
  return JS_EXCEPTION;
}

JSValue
js_point_get_xy(JSContext* ctx, JSValueConst this_val, int magic) {
  JSPointData* s = js_point_data(ctx, this_val);
  if(!s)
    return JS_EXCEPTION;
  if(magic == 0)
    return JS_NewFloat64(ctx, s->x);
  else if(magic == 1)
    return JS_NewFloat64(ctx, s->y);
  return JS_UNDEFINED;
}

JSValue
js_point_set_xy(JSContext* ctx, JSValueConst this_val, JSValue val, int magic) {
  JSPointData* s = js_point_data(ctx, this_val);
  double v;
  if(!s)
    return JS_EXCEPTION;
  if(JS_ToFloat64(ctx, &v, val))
    return JS_EXCEPTION;
  if(magic == 0)
    s->x = v;
  else
    s->y = v;
  return JS_UNDEFINED;
}

JSValue
js_point_norm(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = js_point_data(ctx, this_val);
  if(!s)
    return JS_EXCEPTION;
  return JS_NewFloat64(ctx, sqrt((double)s->x * s->x + (double)s->y * s->y));
}

JSValue
js_point_cross(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = js_point_data(ctx, this_val);
  JSPointData* other = js_point_data(ctx, argv[0]);
  double retval;
  if(!s || !other)
    return JS_EXCEPTION;
  retval = s->cross(*other);
  return JS_NewFloat64(ctx, retval);
}

JSValue
js_point_ddot(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = js_point_data(ctx, this_val);
  JSPointData* other = js_point_data(ctx, argv[0]);
  double retval;
  if(!s || !other)
    return JS_EXCEPTION;
  retval = s->ddot(*other);
  return JS_NewFloat64(ctx, retval);
}

JSValue
js_point_inside(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = js_point_data(ctx, this_val);
  JSRectData* r = js_rect_data(ctx, argv[0]);
  bool retval;
  if(!s || !r)
    return JS_EXCEPTION;

  retval = s->inside(*r);

  return JS_NewBool(ctx, retval);
}

JSValue
js_point_diff(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = js_point_data(ctx, this_val);
  JSPointData* other = js_point_data(ctx, argv[0]);

  JSValue ret;
  if(!s || !other)
    return JS_EXCEPTION;

  ret = js_point_new(ctx, s->x - other->x, s->y - other->y);
  return ret;
}

JSValue
js_point_prod(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = js_point_data(ctx, this_val);
  double factor = 1.0;
  JS_ToFloat64(ctx, &factor, argv[0]);
  JSValue ret;
  if(!s || argc < 1)
    return JS_EXCEPTION;

  ret = js_point_new(ctx, s->x * factor, s->y * factor);
  return ret;
}

JSValue
js_point_quot(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = js_point_data(ctx, this_val);
  double divisor = 1.0;
  JS_ToFloat64(ctx, &divisor, argv[0]);
  JSValue ret;
  if(!s || argc < 1)
    return JS_EXCEPTION;

  ret = js_point_new(ctx, s->x / divisor, s->y / divisor);
  return ret;
}

JSValue
js_point_sum(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = js_point_data(ctx, this_val);
  JSPointData* other = js_point_data(ctx, argv[0]);

  JSValue ret;
  if(!s || !other)
    return JS_EXCEPTION;

  ret = js_point_new(ctx, s->x + other->x, s->y + other->y);
  return ret;
}

JSValue
js_point_getrotationmatrix2d(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = js_point_data(ctx, this_val);

  double angle = 0, scale = 1;
  JSMatData m;

  JSValue ret;
  if(!s)
    return JS_EXCEPTION;
  if(argc > 0) {
    JS_ToFloat64(ctx, &angle, argv[0]);
    if(argc > 1) {
      JS_ToFloat64(ctx, &scale, argv[1]);
    }
  }

  m = cv::getRotationMatrix2D(*s, angle, scale);

  ret = js_mat_wrap(ctx, m);
  return ret;
}

JSValue
js_point_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = js_point_data(ctx, this_val);
  std::ostringstream os;
  JSValue xv, yv;
  double x = -1, y = -1;
  /* if(!s)
     return JS_EXCEPTION;*/

  xv = JS_GetPropertyStr(ctx, this_val, "x");
  yv = JS_GetPropertyStr(ctx, this_val, "y");

  if(JS_IsNumber(xv) && JS_IsNumber(yv)) {
    JS_ToFloat64(ctx, &x, xv);
    JS_ToFloat64(ctx, &y, yv);
  } else if(s) {
    x = s->x;
    y = s->y;
  }

  os << "{x:" << x << ",y:" << y << "}" << std::endl;

  return JS_NewString(ctx, os.str().c_str());
}

JSClassDef js_point_class = {
    "Point",
    .finalizer = js_point_finalizer,
};

const JSCFunctionListEntry js_point_proto_funcs[] = {
    JS_CGETSET_MAGIC_DEF("x", js_point_get_xy, js_point_set_xy, 0),
    JS_CGETSET_MAGIC_DEF("y", js_point_get_xy, js_point_set_xy, 1),
    JS_CFUNC_DEF("cross", 1, js_point_cross),
    JS_CFUNC_DEF("dot", 1, js_point_ddot),
    JS_CFUNC_DEF("inside", 1, js_point_inside),
    JS_CFUNC_DEF("diff", 1, js_point_diff),
    JS_CFUNC_DEF("sum", 1, js_point_sum),
    JS_CFUNC_DEF("prod", 1, js_point_prod),
    JS_CFUNC_DEF("quot", 1, js_point_quot),
    JS_CFUNC_DEF("norm", 0, js_point_norm),
    JS_CFUNC_DEF("getRotationMatrix2D", 0, js_point_getrotationmatrix2d),
    JS_CFUNC_DEF("toString", 0, js_point_to_string),
};

int
js_point_init(JSContext* ctx, void* m, const char* name, bool exp) {

  /* create the Point class */
  JS_NewClassID(&js_point_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_point_class_id, &js_point_class);

  point_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx, point_proto, js_point_proto_funcs, countof(js_point_proto_funcs));
  JS_SetClassProto(ctx, js_point_class_id, point_proto);

  point_class = JS_NewCFunction2(ctx, js_point_ctor, name, 2, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, point_class, point_proto);

  if(exp)
    JS_SetModuleExport(ctx, static_cast<JSModuleDef*>(m), name, point_class);
  else
    JS_SetPropertyStr(ctx, *static_cast<JSValue*>(m), name, point_class);
  return 0;
}

void
js_size_finalizer(JSRuntime* rt, JSValue val) {
  JSSizeData* s = static_cast<JSSizeData*>(JS_GetOpaque(val, js_size_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */
  js_free_rt(rt, s);
}

JSValue
js_size_new(JSContext* ctx, double w, double h) {
  JSValue ret;
  JSSizeData* s;

  ret = JS_NewObjectProtoClass(ctx, size_proto, js_size_class_id);

  s = static_cast<JSSizeData*>(js_mallocz(ctx, sizeof(JSSizeData)));
  s->width = w;
  s->height = h;

  JS_SetOpaque(ret, s);
  return ret;
}

JSSizeData*
js_size_data(JSContext* ctx, JSValue val) {
  return static_cast<JSSizeData*>(JS_GetOpaque2(ctx, val, js_size_class_id));
}

JSValue
js_size_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSSizeData* s;
  JSValue obj = JS_UNDEFINED;
  JSValue proto;

  s = static_cast<JSSizeData*>(js_mallocz(ctx, sizeof(*s)));
  if(!s)
    return JS_EXCEPTION;
  if(JS_ToFloat64(ctx, &s->width, argv[0]))
    goto fail;
  if(JS_ToFloat64(ctx, &s->height, argv[1]))
    goto fail;
  /* using new_target to get the prototype is necessary when the
     class is extended. */
  proto = JS_GetPropertyStr(ctx, new_target, "prototype");
  if(JS_IsException(proto))
    goto fail;
  obj = JS_NewObjectProtoClass(ctx, proto, js_size_class_id);
  JS_FreeValue(ctx, proto);
  if(JS_IsException(obj))
    goto fail;
  JS_SetOpaque(obj, s);
  return obj;
fail:
  js_free(ctx, s);
  JS_FreeValue(ctx, obj);
  return JS_EXCEPTION;
}

JSValue
js_size_get_wh(JSContext* ctx, JSValueConst this_val, int magic) {
  JSSizeData* s = js_size_data(ctx, this_val);
  if(!s)
    return JS_EXCEPTION;
  if(magic == 0)
    return JS_NewFloat64(ctx, s->width);
  else if(magic == 1)
    return JS_NewFloat64(ctx, s->height);
  return JS_UNDEFINED;
}

JSValue
js_size_set_wh(JSContext* ctx, JSValueConst this_val, JSValue val, int magic) {
  JSSizeData* s = js_size_data(ctx, this_val);
  double v;
  if(!s)
    return JS_EXCEPTION;
  if(JS_ToFloat64(ctx, &v, val))
    return JS_EXCEPTION;
  if(magic == 0)
    s->width = v;
  else
    s->height = v;
  return JS_UNDEFINED;
}

JSValue
js_size_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSSizeData* s = js_size_data(ctx, this_val);
  std::ostringstream os;
  JSValue wv, hv;
  double width = -1, height = -1;
  /* if(!s)
     return JS_EXCEPTION;*/

  wv = JS_GetPropertyStr(ctx, this_val, "width");
  hv = JS_GetPropertyStr(ctx, this_val, "height");

  if(JS_IsNumber(wv) && JS_IsNumber(hv)) {
    JS_ToFloat64(ctx, &width, wv);
    JS_ToFloat64(ctx, &height, hv);
  } else if(s) {
    width = s->width;
    height = s->height;
  }

  os << "{width:" << width << ",height:" << height << "}" << std::endl;

  return JS_NewString(ctx, os.str().c_str());
}

JSClassDef js_size_class = {
    "Size",
    .finalizer = js_size_finalizer,
};

const JSCFunctionListEntry js_size_proto_funcs[] = {
    JS_CGETSET_MAGIC_DEF("width", js_size_get_wh, js_size_set_wh, 0),
    JS_CGETSET_MAGIC_DEF("height", js_size_get_wh, js_size_set_wh, 1),
    JS_CFUNC_DEF("toString", 0, js_size_to_string),
};

int
js_size_init(JSContext* ctx, void* m, const char* name, bool exp) {

  /* create the Size class */
  JS_NewClassID(&js_size_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_size_class_id, &js_size_class);

  size_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx, size_proto, js_size_proto_funcs, countof(js_size_proto_funcs));
  JS_SetClassProto(ctx, js_size_class_id, size_proto);

  size_class = JS_NewCFunction2(ctx, js_size_ctor, name, 2, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, size_class, size_proto);

  if(exp)
    JS_SetModuleExport(ctx, static_cast<JSModuleDef*>(m), name, size_class);
  else
    JS_SetPropertyStr(ctx, *static_cast<JSValue*>(m), name, size_class);
  return 0;
}

/*
JSModuleDef*
js_init_module(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, js_point_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "Point");
  return m;
}*/
}

JSRectData*
js_rect_data(JSContext* ctx, JSValue val) {
  return static_cast<JSRectData*>(JS_GetOpaque2(ctx, val, js_rect_class_id));
}

void
js_rect_finalizer(JSRuntime* rt, JSValue val) {
  JSRectData* s = static_cast<JSRectData*>(JS_GetOpaque(val, js_rect_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */
  js_free_rt(rt, s);
}

JSValue
js_rect_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSRectData* s;
  JSValue obj = JS_UNDEFINED;
  JSValue proto;

  s = static_cast<JSRectData*>(js_mallocz(ctx, sizeof(*s)));
  if(!s)
    return JS_EXCEPTION;
  if(JS_ToFloat64(ctx, &s->x, argv[0]))
    goto fail;
  if(JS_ToFloat64(ctx, &s->y, argv[1]))
    goto fail;
  if(JS_ToFloat64(ctx, &s->width, argv[2]))
    goto fail;
  if(JS_ToFloat64(ctx, &s->height, argv[3]))
    goto fail;
  /* using new_target to get the prototype is necessary when the
     class is extended. */
  proto = JS_GetPropertyStr(ctx, new_target, "prototype");
  if(JS_IsException(proto))
    goto fail;
  obj = JS_NewObjectProtoClass(ctx, proto, js_rect_class_id);
  JS_FreeValue(ctx, proto);
  if(JS_IsException(obj))
    goto fail;
  JS_SetOpaque(obj, s);
  return obj;
fail:
  js_free(ctx, s);
  JS_FreeValue(ctx, obj);
  return JS_EXCEPTION;
}

JSValue
js_rect_get_xywh(JSContext* ctx, JSValueConst this_val, int magic) {
  JSValue ret = JS_UNDEFINED;
  JSRectData* s = static_cast<JSRectData*>(JS_GetOpaque2(ctx, this_val, js_rect_class_id));
  if(!s)
    ret = JS_EXCEPTION;
  else if(magic == 0)
    ret = JS_NewFloat64(ctx, s->x);
  else if(magic == 1)
    ret = JS_NewFloat64(ctx, s->y);
  else if(magic == 2)
    ret = JS_NewFloat64(ctx, s->width);
  else if(magic == 3)
    ret = JS_NewFloat64(ctx, s->height);
  else if(magic == 4)
    ret = JS_NewFloat64(ctx, s->x + s->width);
  else if(magic == 5)
    ret = JS_NewFloat64(ctx, s->y + s->height);
  return ret;
}

JSValue
js_rect_set_xywh(JSContext* ctx, JSValueConst this_val, JSValue val, int magic) {
  JSRectData* s = static_cast<JSRectData*>(JS_GetOpaque2(ctx, this_val, js_rect_class_id));
  double v;
  if(!s)
    return JS_EXCEPTION;
  if(JS_ToFloat64(ctx, &v, val))
    return JS_EXCEPTION;
  if(magic == 0)
    s->x = v;
  else if(magic == 1)
    s->y = v;
  else if(magic == 2)
    s->width = v;
  else if(magic == 3)
    s->height = v;
  else if(magic == 4)
    s->width = v - s->x;
  else if(magic == 5)
    s->height = v - s->y;

  return JS_UNDEFINED;
}

JSValue
js_rect_points(JSContext* ctx, JSValueConst this_val) {
  JSValue ret = JS_UNDEFINED;
  std::vector<cv::Point2d> points;
  JSRectData* s = static_cast<JSRectData*>(JS_GetOpaque2(ctx, this_val, js_rect_class_id));
  if(!s)
    ret = JS_EXCEPTION;

  points.push_back(cv::Point2d(s->x, s->y));
  points.push_back(cv::Point2d(s->x + s->width, s->y));
  points.push_back(cv::Point2d(s->x + s->width, s->y + s->height));
  points.push_back(cv::Point2d(s->x, s->y + s->height));
  points.push_back(cv::Point2d(s->x, s->y));

  ret = js_contour_new(ctx, points);
  return ret;
}

JSValue
js_rect_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSRectData* s = static_cast<JSRectData*>(JS_GetOpaque2(ctx, this_val, js_rect_class_id));
  std::ostringstream os;
  JSValue xv, yv, wv, hv;
  double x = -1, y = -1, w = 0, h = 0;
  /* if(!s)
     return JS_EXCEPTION;*/

  if(s) {
    x = s->x;
    y = s->y;
    w = s->width;
    h = s->height;
  } else {
    xv = JS_GetPropertyStr(ctx, this_val, "x");
    yv = JS_GetPropertyStr(ctx, this_val, "y");
    wv = JS_GetPropertyStr(ctx, this_val, "width");
    hv = JS_GetPropertyStr(ctx, this_val, "height");

    if(JS_IsNumber(xv) && JS_IsNumber(yv) && JS_IsNumber(wv) && JS_IsNumber(hv)) {
      JS_ToFloat64(ctx, &x, xv);
      JS_ToFloat64(ctx, &y, yv);
      JS_ToFloat64(ctx, &w, wv);
      JS_ToFloat64(ctx, &h, hv);
    }
  }

  os << "{x:" << x << ",y:" << y << ",width:" << w << ",height:" << h << "}" << std::endl;

  return JS_NewString(ctx, os.str().c_str());
}

JSClassDef js_rect_class = {
    "Rect",
    .finalizer = js_rect_finalizer,
};

const JSCFunctionListEntry js_rect_proto_funcs[] = {
    JS_CGETSET_MAGIC_DEF("x", js_rect_get_xywh, js_rect_set_xywh, 0),
    JS_CGETSET_MAGIC_DEF("y", js_rect_get_xywh, js_rect_set_xywh, 1),
    JS_CGETSET_MAGIC_DEF("width", js_rect_get_xywh, js_rect_set_xywh, 2),
    JS_CGETSET_MAGIC_DEF("height", js_rect_get_xywh, js_rect_set_xywh, 3),
    JS_CGETSET_MAGIC_DEF("x2", js_rect_get_xywh, js_rect_set_xywh, 4),
    JS_CGETSET_MAGIC_DEF("y2", js_rect_get_xywh, js_rect_set_xywh, 5),
    JS_CGETSET_DEF("points", js_rect_points, NULL),
    JS_ALIAS_DEF("x1", "x"),
    JS_ALIAS_DEF("y1", "y"),
    JS_CFUNC_DEF("toString", 0, js_rect_to_string),
};

int
js_rect_init(JSContext* ctx, void* m, const char* name, bool exp) {

  /* create the Rect class */
  JS_NewClassID(&js_rect_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_rect_class_id, &js_rect_class);

  rect_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx, rect_proto, js_rect_proto_funcs, countof(js_rect_proto_funcs));
  JS_SetClassProto(ctx, js_rect_class_id, rect_proto);

  rect_class = JS_NewCFunction2(ctx, js_rect_ctor, name, 2, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, rect_class, rect_proto);

  if(exp)
    JS_SetModuleExport(ctx, static_cast<JSModuleDef*>(m), name, rect_class);
  else
    JS_SetPropertyStr(ctx, *static_cast<JSValue*>(m), name, rect_class);
  return 0;
}

/* PointIterator Class */

JSClassID js_contour_class_id;

struct JSPointIteratorData {
  cv::Point2d *begin, *end;
};

JSClassID js_point_iterator_class_id;

void
js_point_iterator_finalizer(JSRuntime* rt, JSValue val) {
  JSPointIteratorData* s =
      static_cast<JSPointIteratorData*>(JS_GetOpaque(val, js_point_iterator_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */
  js_free_rt(rt, s);
}

JSValue
js_point_iterator_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSPointIteratorData* s;
  JSContourData* v;
  JSValue obj = JS_UNDEFINED;
  JSValue proto;

  s = static_cast<JSPointIteratorData*>(js_mallocz(ctx, sizeof(*s)));
  if(!s)
    return JS_EXCEPTION;

  v = static_cast<JSContourData*>(JS_GetOpaque(argv[0], js_contour_class_id));

  s->begin = &(*v)[0];
  s->end = s->begin + v->size();

  /*  if(JS_ToFloat64(ctx, &s->x, argv[0]))
      goto fail;
    if(JS_ToFloat64(ctx, &s->y, argv[1]))
      goto fail;*/
  /* using new_target to get the prototype is necessary when the
     class is extended. */
  proto = JS_GetPropertyStr(ctx, new_target, "prototype");
  if(JS_IsException(proto))
    goto fail;
  obj = JS_NewObjectProtoClass(ctx, proto, js_point_iterator_class_id);
  JS_FreeValue(ctx, proto);
  if(JS_IsException(obj))
    goto fail;
  JS_SetOpaque(obj, s);
  return obj;
fail:
  js_free(ctx, s);
  JS_FreeValue(ctx, obj);
  return JS_EXCEPTION;
}

JSValue
js_point_iterator_next(
    JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, BOOL* pdone, int magic) {
  JSPointIteratorData* it =
      static_cast<JSPointIteratorData*>(JS_GetOpaque(this_val, js_point_iterator_class_id));
  JSPointData* ptr;
  JSValue point;

  point = JS_NewObjectClass(ctx, js_point_iterator_class_id);
  if(JS_IsException(point))
    goto fail;
  ptr = static_cast<JSPointData*>(js_malloc(ctx, sizeof(*ptr)));
  if(!ptr)
    goto fail1;
  ptr = it->begin;

  *pdone = (it->begin == it->end);
  it->begin++;
  JS_SetOpaque(point, ptr);
  return point;
fail1:
  JS_FreeValue(ctx, point);
fail:
  return JS_EXCEPTION;
}

JSValue
js_point_iterator_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointIteratorData* s =
      static_cast<JSPointIteratorData*>(JS_GetOpaque2(ctx, this_val, js_point_iterator_class_id));
  std::ostringstream os;
  if(!s)
    return JS_EXCEPTION;

  // os << "{x:" << s->x << ",y:" << s->y << "}" << std::endl;

  return JS_NewString(ctx, os.str().c_str());
}

JSClassDef js_point_iterator_class = {
    "PointIterator",
    .finalizer = js_point_iterator_finalizer,
};

const JSCFunctionListEntry js_point_iterator_proto_funcs[] = {
    JS_ITERATOR_NEXT_DEF("next", 0, js_point_iterator_next, 0),
    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "PointIterator", JS_PROP_CONFIGURABLE),
};

int
js_point_iterator_init(JSContext* ctx, void* m, const char* name, bool exp) {

  /* create the PointIterator class */
  JS_NewClassID(&js_point_iterator_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_point_iterator_class_id, &js_point_iterator_class);

  point_iterator_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx,
                             point_iterator_proto,
                             js_point_iterator_proto_funcs,
                             countof(js_point_iterator_proto_funcs));
  JS_SetClassProto(ctx, js_point_iterator_class_id, point_iterator_proto);

  point_iterator_class =
      JS_NewCFunction2(ctx, js_point_iterator_ctor, name, 2, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, point_iterator_class, point_iterator_proto);

  if(exp)
    JS_SetModuleExport(ctx, static_cast<JSModuleDef*>(m), name, point_iterator_class);
  else
    JS_SetPropertyStr(ctx, *static_cast<JSValue*>(m), name, point_iterator_class);
  return 0;
}

/* Contour Class */

JSContourData*
js_contour_data(JSContext* ctx, JSValue val) {
  return static_cast<JSContourData*>(JS_GetOpaque2(ctx, val, js_contour_class_id));
}

JSValue
js_create_point_iterator(
    JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSContourData* s = js_contour_data(ctx, this_val);
  JSPointIteratorData* it;
  JSValue iterator;
  int class_id;

  iterator = JS_NewObjectClass(ctx, js_point_iterator_class_id);
  if(JS_IsException(iterator))
    goto fail;
  it = static_cast<JSPointIteratorData*>(js_malloc(ctx, sizeof(*it)));
  if(!it)
    goto fail1;
  it->begin = &(*s)[0];
  it->end = it->begin + s->size();
  JS_SetOpaque(iterator, it);
  return iterator;
fail1:
  JS_FreeValue(ctx, iterator);
fail:
  return JS_EXCEPTION;
}

void
js_contour_finalizer(JSRuntime* rt, JSValue val) {
  JSContourData* s = static_cast<JSContourData*>(JS_GetOpaque(val, js_contour_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */
  js_free_rt(rt, s);
}

JSValue
js_contour_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSContourData* v;
  JSValue obj = JS_UNDEFINED;
  JSValue proto;

  v = static_cast<JSContourData*>(js_mallocz(ctx, sizeof(*v)));
  if(!v)
    return JS_EXCEPTION;

  /* using new_target to get the prototype is necessary when the
     class is extended. */
  proto = JS_GetPropertyStr(ctx, new_target, "prototype");
  if(JS_IsException(proto))
    goto fail;
  obj = JS_NewObjectProtoClass(ctx, proto, js_contour_class_id);
  JS_FreeValue(ctx, proto);
  if(JS_IsException(obj))
    goto fail;
  JS_SetOpaque(obj, v);
  return obj;
fail:
  js_free(ctx, v);
  JS_FreeValue(ctx, obj);
  return JS_EXCEPTION;
}

JSValue
js_contour2i_new(JSContext* ctx, const std::vector<cv::Point_<int>>& points) {
  JSValue ret;
  JSContourData* contour;

  ret = JS_NewObjectProtoClass(ctx, contour_proto, js_contour_class_id);

  contour = static_cast<JSContourData*>(js_mallocz(ctx, sizeof(JSContourData)));

  std::transform(points.cbegin(),
                 points.cend(),
                 std::back_inserter(*contour),
                 [](const cv::Point& pt) -> cv::Point2d { return cv::Point2d(pt.x, pt.y); });

  JS_SetOpaque(ret, contour);
  return ret;
};

JSValue
js_contour2f_new(JSContext* ctx, const std::vector<cv::Point_<float>>& points) {
  JSValue ret;
  JSContourData* contour;

  ret = JS_NewObjectProtoClass(ctx, contour_proto, js_contour_class_id);

  contour = static_cast<JSContourData*>(js_mallocz(ctx, sizeof(JSContourData)));

  std::transform(points.cbegin(),
                 points.cend(),
                 std::back_inserter(*contour),
                 [](const cv::Point2f& pt) -> cv::Point2d { return cv::Point2d(pt.x, pt.y); });

  JS_SetOpaque(ret, contour);
  return ret;
};

JSValue
js_contour_new(JSContext* ctx, const std::vector<cv::Point_<float>>& points) {
  return js_contour2f_new(ctx, points);
}

JSValue
js_contour_new(JSContext* ctx, const std::vector<cv::Point_<double>>& points) {
  JSValue ret;
  JSContourData* contour;

  ret = JS_NewObjectProtoClass(ctx, contour_proto, js_contour_class_id);

  contour = static_cast<JSContourData*>(js_mallocz(ctx, sizeof(JSContourData)));

  std::copy(points.begin(), points.end(), std::back_inserter(*contour));

  JS_SetOpaque(ret, contour);
  return ret;
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

JSValue
js_contour_push(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* v;
  int i;
  double x, y;
  JSValue xv, yv;
  JSPointData point;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  for(i = 0; i < argc; i++) {
    if(JS_IsObject(argv[i])) {
      xv = JS_GetPropertyStr(ctx, argv[i], "x");
      yv = JS_GetPropertyStr(ctx, argv[i], "y");
    } else if(JS_IsArray(ctx, argv[i])) {

      xv = JS_GetPropertyUint32(ctx, argv[i], 0);
      yv = JS_GetPropertyUint32(ctx, argv[i], 1);
    } else if(i + 1 < argc) {
      xv = argv[i++];
      yv = argv[i];
    }
    JS_ToFloat64(ctx, &point.x, xv);
    JS_ToFloat64(ctx, &point.y, yv);

    v->push_back(point);
  }
  return JS_UNDEFINED;
}

JSValue
js_contour_get(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* v;
  JSValue ret;
  JSValue x, y;
  int64_t i;
  JSPointData* point;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  JS_ToInt64(ctx, &i, argv[0]);

  ret = js_point_new(ctx, (*v)[i].x, (*v)[i].y);
  return ret;
}

JSValue
js_contour_pop(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* v;
  JSValue ret;
  JSValue x, y;
  JSPointData *ptr, point;
  int64_t n = 0;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;
  n = v->size();
  if(n > 0) {
    point = (*v)[n - 1];
    v->pop_back();
  } else {
    return JS_EXCEPTION;
  }

  ret = js_point_new(ctx, point.x, point.y);

  return ret;
}

JSValue
js_contour_length(JSContext* ctx, JSValueConst this_val) {
  JSContourData* v;
  JSValue ret;
  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;
  ret = JS_NewInt64(ctx, v->size());
  return ret;
}

JSValue
js_contour_area(JSContext* ctx, JSValueConst this_val) {
  JSContourData* v;
  JSValue ret = JS_UNDEFINED;
  double area;
  std::vector<cv::Point2f> contour;
  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;
  std::transform(v->begin(),
                 v->end(),
                 std::back_inserter(contour),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });
  area = cv::contourArea(contour);
  ret = JS_NewFloat64(ctx, area);
  return ret;
}

JSValue
js_contour_center(JSContext* ctx, JSValueConst this_val) {
  JSContourData* v;
  JSValue ret = JS_UNDEFINED;
  double area;
  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;
  {
    std::vector<cv::Point> points;
    points.resize(v->size());
    std::copy(v->begin(), v->end(), points.begin());
    cv::Moments mu = cv::moments(points);
    cv::Point centroid = cv::Point(mu.m10 / mu.m00, mu.m01 / mu.m00);

    ret = js_point_new(ctx, centroid.x, centroid.y);
  }

  return ret;
}

JSValue
js_contour_fitellipse(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *ptr;
  cv::RotatedRect rr;
  JSValue ret = JS_UNDEFINED;
  double area;
  std::vector<cv::Point2f> contour, ellipse;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  std::transform(v->begin(),
                 v->end(),
                 std::back_inserter(contour),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });

  rr = cv::fitEllipse(contour);

  ellipse.resize(5);
  rr.points(ellipse.data());

  ret = js_contour2f_new(ctx, ellipse);

  return ret;
}

JSValue
js_contour_fitline(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *ptr;
  JSValue ret = JS_UNDEFINED;
  int64_t distType = cv::DIST_FAIR;
  double param = 0, reps = 0.01, aeps = 0.01;
  std::vector<cv::Point> contour;
  std::vector<cv::Point2f> points;
  cv::Vec4f line;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  if(argc > 0) {

    JS_ToInt64(ctx, &distType, argv[0]);

    if(argc > 1) {
      JS_ToFloat64(ctx, &param, argv[1]);
      if(argc > 2) {
        JS_ToFloat64(ctx, &reps, argv[2]);
        if(argc > 3) {
          JS_ToFloat64(ctx, &aeps, argv[3]);
        }
      }
    }
  }

  std::transform(v->begin(),
                 v->end(),
                 std::back_inserter(contour),
                 [](const cv::Point2d& pt) -> cv::Point { return cv::Point(pt.x, pt.y); });

  cv::fitLine(contour, line, distType, param, reps, aeps);

  points.push_back(cv::Point2f(line[0], line[1]));
  points.push_back(cv::Point2f(line[2], line[3]));

  ret = js_contour2f_new(ctx, points);

  return ret;
}

JSValue
js_contour_approxpolydp(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSValue ret = JS_UNDEFINED;
  double epsilon;
  bool closed = false;
  std::vector<cv::Point> curve;
  std::vector<cv::Point2f> approxCurve;
  JSContourData *out, *v;

  v = js_contour_data(ctx, this_val);

  if(!v)
    return JS_EXCEPTION;
  out = static_cast<JSContourData*>(JS_GetOpaque2(ctx, argv[0], js_contour_class_id));

  if(argc > 1) {
    JS_ToFloat64(ctx, &epsilon, argv[1]);

    if(argc > 2) {
      closed = !!JS_ToBool(ctx, argv[2]);
    }
  }

  std::transform(v->begin(),
                 v->end(),
                 std::back_inserter(curve),
                 [](const cv::Point2d& pt) -> cv::Point { return cv::Point(pt.x, pt.y); });

  cv::approxPolyDP(curve, approxCurve, epsilon, closed);

  std::transform(approxCurve.begin(),
                 approxCurve.end(),
                 std::back_inserter(*out),
                 [](const cv::Point2f& pt) -> cv::Point2d { return cv::Point2d(pt.x, pt.y); });

  return JS_UNDEFINED;
}

JSValue
js_contour_convexhull(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSValue ret = JS_UNDEFINED;
  bool clockwise = false, returnPoints = true;
  std::vector<cv::Point2f> curve, hull;
  std::vector<int> hullIndices;
  JSContourData *out, *v;

  v = js_contour_data(ctx, this_val);

  if(!v)
    return JS_EXCEPTION;

  if(argc > 0) {
    clockwise = !!JS_ToBool(ctx, argv[0]);

    if(argc > 1) {
      returnPoints = !!JS_ToBool(ctx, argv[1]);
    }
  }

  std::transform(v->begin(),
                 v->end(),
                 std::back_inserter(curve),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });

  if(returnPoints)
    cv::convexHull(curve, hull, clockwise, true);
  else
    cv::convexHull(curve, hullIndices, clockwise, false);

  if(returnPoints) {
    ret = js_contour2f_new(ctx, hull);
  } else {
    uint32_t i, size = hullIndices.size();

    ret = JS_NewArray(ctx);

    for(i = 0; i < size; i++) {
      JS_SetPropertyUint32(ctx, ret, i, JS_NewInt32(ctx, hullIndices[i]));
    }
  }

  return ret;
}

JSValue
js_contour_boundingrect(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSValue ret = JS_UNDEFINED;
  cv::Rect2f rect;
  std::vector<cv::Point2f> curve;
  JSContourData* v;
  JSRectData* r;

  v = js_contour_data(ctx, this_val);

  if(!v)
    return JS_EXCEPTION;

  std::transform(v->begin(),
                 v->end(),
                 std::back_inserter(curve),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });

  rect = cv::boundingRect(curve);

  ret = JS_NewObjectProtoClass(ctx, rect_proto, js_rect_class_id);

  r = static_cast<JSRectData*>(js_mallocz(ctx, sizeof(*r)));

  *r = rect;

  JS_SetOpaque(ret, r);

  return ret;
}

JSValue
js_contour_intersectconvex(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  bool handleNested = true;
  std::vector<cv::Point2f> a, b, intersection;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  if(argc > 0) {
    other = static_cast<JSContourData*>(JS_GetOpaque2(ctx, argv[0], js_contour_class_id));

    if(argc > 1) {
      handleNested = !!JS_ToBool(ctx, argv[1]);
    }
  }

  std::transform(v->begin(),
                 v->end(),
                 std::back_inserter(a),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });

  std::transform(other->begin(),
                 other->end(),
                 std::back_inserter(b),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });

  cv::intersectConvexConvex(a, b, intersection, handleNested);

  ret = js_contour2f_new(ctx, intersection);
  return ret;
}

JSValue
js_contour_isconvex(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  bool isConvex;
  std::vector<cv::Point2f> contour;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  std::transform(v->begin(),
                 v->end(),
                 std::back_inserter(contour),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });

  isConvex = cv::isContourConvex(contour);

  ret = JS_NewBool(ctx, isConvex);

  return ret;
}

JSValue
js_contour_minarearect(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  cv::RotatedRect rr;

  std::vector<cv::Point2f> contour, minarea;
  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;
  std::transform(v->begin(),
                 v->end(),
                 std::back_inserter(contour),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });
  rr = cv::minAreaRect(contour);
  minarea.resize(5);
  rr.points(minarea.data());

  ret = js_contour2f_new(ctx, minarea);
  return ret;
}

JSValue
js_contour_minenclosingcircle(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  cv::RotatedRect rr;

  std::vector<cv::Point2f> contour;
  cv::Point2f center;
  float radius;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  std::transform(v->begin(),
                 v->end(),
                 std::back_inserter(contour),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });

  cv::minEnclosingCircle(contour, center, radius);

  ret = JS_NewObject(ctx);

  JS_SetPropertyStr(ctx, ret, "center", js_point_new(ctx, center.x, center.y));
  JS_SetPropertyStr(ctx, ret, "radius", JS_NewFloat64(ctx, radius));

  return ret;
}

JSValue
js_contour_minenclosingtriangle(JSContext* ctx,
                                JSValueConst this_val,
                                int argc,
                                JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  cv::RotatedRect rr;

  std::vector<cv::Point2f> contour, triangle;
  cv::Point2f center;
  float radius;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  std::transform(v->begin(),
                 v->end(),
                 std::back_inserter(contour),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });

  cv::minEnclosingTriangle(contour, triangle);

  ret = js_contour2f_new(ctx, triangle);

  return ret;
}

JSValue
js_contour_pointpolygontest(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  cv::RotatedRect rr;
  cv::Point2f pt;
  bool measureDist = false;
  std::vector<cv::Point2f> contour, triangle;
  JSPointData* point;
  double retval;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  if(argc > 0) {
    point = js_point_data(ctx, argv[0]);

    if(point) {
      pt.x = point->x;
      pt.y = point->y;
    }
    if(argc > 1) {
      measureDist = !!JS_ToBool(ctx, argv[1]);
    }
  }

  std::transform(v->begin(),
                 v->end(),
                 std::back_inserter(contour),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });

  retval = cv::pointPolygonTest(contour, pt, measureDist);

  ret = JS_NewFloat64(ctx, retval);

  return ret;
}

JSValue
js_contour_rotatedrectangleintersection(JSContext* ctx,
                                        JSValueConst this_val,
                                        int argc,
                                        JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  bool handleNested = true;
  std::vector<cv::Point2f> a, b, intersection;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  if(argc > 0) {
    other = static_cast<JSContourData*>(JS_GetOpaque2(ctx, argv[0], js_contour_class_id));
  }

  std::transform(v->begin(),
                 v->end(),
                 std::back_inserter(a),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });

  std::transform(other->begin(),
                 other->end(),
                 std::back_inserter(b),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });
  {
    cv::RotatedRect rra(a[0], a[1], a[2]);
    cv::RotatedRect rrb(b[0], b[1], b[2]);

    cv::rotatedRectangleIntersection(rra, rrb, intersection);

    ret = js_contour2f_new(ctx, intersection);
  }
  return ret;
}

JSValue
js_contour_arclength(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  cv::Point2f pt;
  bool closed = false;
  std::vector<cv::Point2f> contour;
  JSPointData* point;
  double retval;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  if(argc > 0) {
    closed = !!JS_ToBool(ctx, argv[0]);
  }

  std::transform(v->begin(),
                 v->end(),
                 std::back_inserter(contour),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });

  retval = cv::arcLength(contour, closed);

  ret = JS_NewFloat64(ctx, retval);

  return ret;
}

JSValue
js_contour_getaffinetransform(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  bool handleNested = true;
  std::vector<cv::Point2f> a, b;
  cv::Mat matrix;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  if(argc > 0) {
    other = static_cast<JSContourData*>(JS_GetOpaque2(ctx, argv[0], js_contour_class_id));
  }

  std::transform(v->begin(),
                 v->end(),
                 std::back_inserter(a),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });

  std::transform(other->begin(),
                 other->end(),
                 std::back_inserter(b),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });

  matrix = cv::getAffineTransform(a, b);

  ret = js_mat_wrap(ctx, matrix);
  return ret;
}

JSValue
js_contour_getperspectivetransform(JSContext* ctx,
                                   JSValueConst this_val,
                                   int argc,
                                   JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  bool handleNested = true;
  std::vector<cv::Point2f> a, b;
  cv::Mat matrix;
  int32_t solveMethod = cv::DECOMP_LU;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  if(argc > 0) {
    other = static_cast<JSContourData*>(JS_GetOpaque2(ctx, argv[0], js_contour_class_id));

    if(argc > 1) {
      JS_ToInt32(ctx, &solveMethod, argv[1]);
    }
  }

  std::transform(v->begin(),
                 v->end(),
                 std::back_inserter(a),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });

  std::transform(other->begin(),
                 other->end(),
                 std::back_inserter(b),
                 [](const cv::Point2d& pt) -> cv::Point2f { return cv::Point2f(pt.x, pt.y); });

  matrix = cv::getPerspectiveTransform(a, b, solveMethod);

  ret = js_mat_wrap(ctx, matrix);
  return ret;
}

JSValue
js_contour_rotatepoints(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* s = js_contour_data(ctx, this_val);
  int32_t shift = 1;
  uint32_t size = s->size();
  if(!s)
    return JS_EXCEPTION;
  if(argc > 0)
    JS_ToInt32(ctx, &shift, argv[0]);

  shift %= size;

  if(shift > 0) {
    std::rotate(s->begin(), s->begin() + shift, s->end());

  } else if(shift < 0) {
    std::rotate(s->rbegin(), s->rbegin() + (-shift), s->rend());
  }
  return this_val;
}
JSValue
js_contour_psimpl(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSContourData* s = js_contour_data(ctx, this_val);
  int32_t shift = 1;
  uint32_t size = s->size();
  JSValue ret = JS_UNDEFINED;
  JSContourData r;
  double arg1 = 0, arg2 = 0;
  double* it;
  cv::Point2d* start = &(*s)[0];
  cv::Point2d* end = start + size;
  r.resize(size);
  it = (double*)&r[0];

  if(!s)
    return JS_EXCEPTION;

  if(argc > 0) {
    JS_ToFloat64(ctx, &arg1, argv[0]);
    if(argc > 1) {
      JS_ToFloat64(ctx, &arg2, argv[1]);
    }
  }

  if(magic == 0) {
    if(arg1 == 0)
      arg1 == 2;
    it = psimpl::simplify_reumann_witkam<2>((double*)start, (double*)end, arg1, it);
  } else if(magic == 1) {
    if(arg1 == 0)
      arg1 == 2;
    if(arg2 == 0)
      arg2 == 10;
    it = psimpl::simplify_opheim<2>((double*)start, (double*)end, arg1, arg2, it);
  } else if(magic == 2) {
    if(arg1 == 0)
      arg1 == 2;
    if(arg2 == 0)
      arg2 == 10;
    it = psimpl::simplify_lang<2>((double*)start, (double*)end, arg1, arg2, it);
  } else if(magic == 3) {
    if(arg1 == 0)
      arg1 == 2;
    it = psimpl::simplify_douglas_peucker<2>((double*)start, (double*)end, arg1, it);
  } else if(magic == 4) {
    if(arg1 == 0)
      arg1 == 2;
    it = psimpl::simplify_nth_point<2>((double*)start, (double*)end, arg1, it);
  } else if(magic == 5) {
    if(arg1 == 0)
      arg1 == 2;
    it = psimpl::simplify_radial_distance<2>((double*)start, (double*)end, arg1, it);
  } else if(magic == 6) {
    if(arg1 == 0)
      arg1 == 2;
    if(arg2 == 0)
      arg2 == 1;
    it = psimpl::simplify_perpendicular_distance<2>((double*)start, (double*)end, arg1, arg2, it);
  }
  size = it - (double*)&r[0];
  r.resize(size / 2);
  ret = js_contour_new(ctx, r);
  return ret;
}

JSValue
js_contour_convexitydefects(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* s = js_contour_data(ctx, this_val);
  JSValue ret = JS_UNDEFINED;

  std::vector<int> hullIndices;
  std::vector<cv::Vec4i> defects;

  if(argc > 0) {
    int64_t n = js_array_to_vector(ctx, argv[0], hullIndices);
    if(n == 0)
      return JS_EXCEPTION;
  }

  if(s->size() == 0 || hullIndices.size() == 0)
    return JS_EXCEPTION;

  defects.resize(hullIndices.size());
  cv::convexityDefects(*s, hullIndices, defects);

  ret = js_vector_to_array(ctx, defects);

  return ret;
}

JSValue
js_contour_toarray(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* s = js_contour_data(ctx, this_val);
  uint32_t i, size = s->size();
  JSValue ret = JS_UNDEFINED;

  if(!s)
    return JS_EXCEPTION;

  ret = JS_NewArray(ctx);

  for(i = 0; i < size; i++) {
    JS_SetPropertyUint32(ctx, ret, i, js_point_new(ctx, (*s)[i].x, (*s)[i].y));
  }

  return ret;
}

JSValue
js_contour_tostring(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* s = js_contour_data(ctx, this_val);
  std::ostringstream os;
  int i = 0;
  if(!s)
    return JS_EXCEPTION;
  os << "Contour[";
  std::for_each(s->begin(), s->end(), [&i, &os](const JSPointData& point) {
    if(i > 0)
      os << ',';
    os << "{x:" << point.x << ",y:" << point.y << "}";
    i++;
  });
  os << ']' << std::endl;

  return JS_NewString(ctx, os.str().c_str());
}

JSClassDef js_contour_class = {
    "Contour",
    .finalizer = js_contour_finalizer,
};

const JSCFunctionListEntry js_contour_proto_funcs[] = {
    JS_CFUNC_DEF("push", 1, js_contour_push),
    JS_CFUNC_DEF("pop", 0, js_contour_pop),
    JS_CFUNC_DEF("get", 1, js_contour_get),
    JS_CGETSET_DEF("length", js_contour_length, NULL),
    JS_CGETSET_DEF("area", js_contour_area, NULL),
    JS_CGETSET_DEF("center", js_contour_center, NULL),
    JS_CFUNC_DEF("approxPolyDP", 1, js_contour_approxpolydp),
    JS_CFUNC_DEF("convexHull", 1, js_contour_convexhull),
    JS_CFUNC_DEF("boundingRect", 0, js_contour_boundingrect),
    JS_CFUNC_DEF("fitEllipse", 0, js_contour_fitellipse),
    JS_CFUNC_DEF("fitLine", 0, js_contour_fitline),
    JS_CFUNC_DEF("intersectConvex", 0, js_contour_intersectconvex),
    JS_CFUNC_DEF("isConvex", 0, js_contour_isconvex),
    JS_CFUNC_DEF("minAreaRect", 0, js_contour_minarearect),
    JS_CFUNC_DEF("minEnclosingCircle", 0, js_contour_minenclosingcircle),
    JS_CFUNC_DEF("minEnclosingTriangle", 0, js_contour_minenclosingtriangle),
    JS_CFUNC_DEF("pointPolygonTest", 0, js_contour_pointpolygontest),
    JS_CFUNC_DEF("rotatedRectangleIntersection", 0, js_contour_rotatedrectangleintersection),
    JS_CFUNC_DEF("arcLength", 0, js_contour_arclength),
    JS_CFUNC_DEF("getAffineTransform", 1, js_contour_getaffinetransform),
    JS_CFUNC_DEF("getPerspectiveTransform", 1, js_contour_getperspectivetransform),
    JS_CFUNC_DEF("rotatePoints", 1, js_contour_rotatepoints),
    JS_CFUNC_DEF("convexityDefects", 1, js_contour_convexitydefects),
    JS_CFUNC_MAGIC_DEF("simplifyReumannWitkam", 0, js_contour_psimpl, 0),
    JS_CFUNC_MAGIC_DEF("simplifyOpheim", 0, js_contour_psimpl, 1),
    JS_CFUNC_MAGIC_DEF("simplifyLang", 0, js_contour_psimpl, 2),
    JS_CFUNC_MAGIC_DEF("simplifyDouglasPeucker", 0, js_contour_psimpl, 3),
    JS_CFUNC_MAGIC_DEF("simplifyNthPoint", 0, js_contour_psimpl, 4),
    JS_CFUNC_MAGIC_DEF("simplifyRadialDistance", 0, js_contour_psimpl, 5),
    JS_CFUNC_MAGIC_DEF("simplifyPerpendicularDistance", 0, js_contour_psimpl, 6),
    JS_CFUNC_DEF("toArray", 0, js_contour_toarray),
    JS_CFUNC_DEF("toString", 0, js_contour_tostring),

    JS_CFUNC_MAGIC_DEF("entries", 0, js_create_point_iterator, 0),
    JS_ALIAS_DEF("[Symbol.iterator]", "entries"),

    //  JS_PROP_STRING_DEF("[Symbol.toStringTag]", "Contour", JS_PROP_CONFIGURABLE),

};

int
js_contour_init(JSContext* ctx, void* m, const char* name, bool exp) {

  /* create the Contour class */
  JS_NewClassID(&js_contour_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_contour_class_id, &js_contour_class);

  contour_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx,
                             contour_proto,
                             js_contour_proto_funcs,
                             countof(js_contour_proto_funcs));
  JS_SetClassProto(ctx, js_contour_class_id, contour_proto);

  contour_class = JS_NewCFunction2(ctx, js_contour_ctor, name, 2, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, contour_class, contour_proto);

  if(exp)
    JS_SetModuleExport(ctx, static_cast<JSModuleDef*>(m), name, contour_class);
  else
    JS_SetPropertyStr(ctx, *static_cast<JSValue*>(m), name, contour_class);
  return 0;
}

void
js_mat_finalizer(JSRuntime* rt, JSValue val) {
  JSMatData* s = static_cast<JSMatData*>(JS_GetOpaque(val, js_mat_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */
  js_free_rt(rt, s);
}

JSValue
js_mat_new(JSContext* ctx, int cols = 0, int rows = 0, int type = CV_32FC1) {
  JSValue ret;
  JSMatData* s;
  ret = JS_NewObjectProtoClass(ctx, mat_proto, js_mat_class_id);
  s = new cv::Mat(cv::Size(cols, rows), type);
  *s = cv::Mat::zeros(cv::Size(cols, rows), type);
  JS_SetOpaque(ret, s);
  return ret;
}

JSValue
js_mat_wrap(JSContext* ctx, const cv::Mat& mat) {
  JSValue ret;
  JSMatData* s;
  ret = JS_NewObjectProtoClass(ctx, mat_proto, js_mat_class_id);
  s = new cv::Mat(mat);
  JS_SetOpaque(ret, s);
  return ret;
}

JSMatData*
js_mat_data(JSContext* ctx, JSValue val) {
  return static_cast<JSMatData*>(JS_GetOpaque2(ctx, val, js_mat_class_id));
}

JSValue
js_mat_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSMatData* s;
  JSValue obj = JS_UNDEFINED;
  JSValue proto;
  JSSizeData* size = nullptr;

  int64_t cols = 0, rows = 0;
  int32_t type = CV_32FC1;

  if(argc > 0) {
    JS_ToInt64(ctx, &rows, argv[0]);
    size = js_size_data(ctx, argv[0]);
    if(size != nullptr) {
      cols = size->width;
      rows = size->height;
    } else {
      JS_ToInt64(ctx, &cols, argv[1]);
      argc--;
      argv++;
    }
    if(argc > 1) {
      type = JS_ToInt32(ctx, &type, argv[1]);
    }
  }

  obj = js_mat_new(ctx, cols, rows, type);

  return obj;
fail:
  s->release();

  JS_FreeValue(ctx, obj);
  return JS_EXCEPTION;
}

JSValue
js_mat_get_props(JSContext* ctx, JSValueConst this_val, int magic) {
  JSMatData* s = js_mat_data(ctx, this_val);
  if(!s)
    return JS_EXCEPTION;
  if(magic == 0)
    return JS_NewFloat64(ctx, s->cols);
  else if(magic == 1)
    return JS_NewFloat64(ctx, s->rows);
  else if(magic == 2)
    return JS_NewFloat64(ctx, s->channels());
  else if(magic == 3)
    return JS_NewFloat64(ctx, s->type());
  else if(magic == 4)
    return JS_NewFloat64(ctx, s->depth());
  else if(magic == 5)
    return JS_NewBool(ctx, s->empty());
  return JS_UNDEFINED;
}

JSValue
js_mat_funcs(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValue ret = JS_UNDEFINED;
  int64_t i = -1, i2 = -1;
  JSPointData* pt = nullptr;
  JSMatData* m = js_mat_data(ctx, this_val);

  if(argc > 0) {
    JS_ToInt64(ctx, &i, argv[0]);
    pt = js_point_data(ctx, argv[0]);
    if(argc > 2) {
      JS_ToInt64(ctx, &i2, argv[2]);
    }
  }

  if(magic == 0)
    ret = js_mat_wrap(ctx, m->col(i));
  else if(magic == 1)
    ret = js_mat_wrap(ctx, m->row(i));
  else if(magic == 2)
    ret = js_mat_wrap(ctx, m->colRange(i, i2));
  else if(magic == 3)
    ret = js_mat_wrap(ctx, m->rowRange(i, i2));
  else if(magic == 4) {
    cv::Point p;

    if(pt != nullptr) {
      p = *pt;
    } else {
      p.x = i;
      p.y = i2;
    }
    if(m->type() == CV_32FC1)
      ret = JS_NewFloat64(ctx, (*m).at<float>(p.y, p.x));
    else
      ret = JS_NewInt64(ctx, (*m).at<uint32_t>(p.y, p.x));

  } else if(magic == 5) {
    ret = js_mat_wrap(ctx, m->clone());
  } else if(magic == 6) {
    JSRectData* rect = argc > 0 ? js_rect_data(ctx, argv[0]) : nullptr;
    ret = js_mat_wrap(ctx, (*m)(*rect));
  }

  return JS_EXCEPTION;
}

JSValue
js_mat_findcontours(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSMatData* m = js_mat_data(ctx, this_val);
  JSValue ret = JS_UNDEFINED;
  int mode = cv::RETR_TREE;
  int approx = cv::CHAIN_APPROX_SIMPLE;
  cv::Point offset(0, 0);

  contour2i_vector contours;
  vec4i_vector hier;
  contour2f_vector poly;

  cv::findContours(*m, contours, hier, mode, approx, offset);

  poly.resize(contours.size());

  transform_contours(contours.cbegin(), contours.cend(), poly.begin());

  {
    JSValue hier_arr = js_vector_to_array(ctx, hier);
    JSValue contours_obj = js_contours_new(ctx, poly);

    ret = JS_NewObject(ctx);

    JS_SetPropertyStr(ctx, ret, "hier", hier_arr);
    JS_SetPropertyStr(ctx, ret, "contours", contours_obj);
  }
  return ret;
}

JSValue
js_mat_tostring(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSMatData* m = js_mat_data(ctx, this_val);
  int x, y;

  std::ostringstream os;
  int i = 0;
  if(!m)
    return JS_EXCEPTION;
  os << "Mat[";
  for(y = 0; y < m->rows; y++) {
    os << "\n  ";

    for(x = 0; x < m->cols; x++) {
      if(x > 0)
        os << ',';
      if(m->type() == CV_32FC1)
        os << m->at<float>(y, x);
      else
        os << m->at<int>(y, x);
    }
  }

  os << ']' << std::endl;

  return JS_NewString(ctx, os.str().c_str());
}

JSClassDef js_mat_class = {
    "Mat",
    .finalizer = js_mat_finalizer,
};

const JSCFunctionListEntry js_mat_proto_funcs[] = {
    JS_CGETSET_MAGIC_DEF("cols", js_mat_get_props, NULL, 0),
    JS_CGETSET_MAGIC_DEF("rows", js_mat_get_props, NULL, 1),
    JS_CGETSET_MAGIC_DEF("channels", js_mat_get_props, NULL, 2),
    JS_CGETSET_MAGIC_DEF("type", js_mat_get_props, NULL, 3),
    JS_CGETSET_MAGIC_DEF("depth", js_mat_get_props, NULL, 4),
    JS_CGETSET_MAGIC_DEF("empty", js_mat_get_props, NULL, 5),
    JS_CFUNC_MAGIC_DEF("col", 1, js_mat_funcs, 0),
    JS_CFUNC_MAGIC_DEF("row", 1, js_mat_funcs, 1),
    JS_CFUNC_MAGIC_DEF("colRange", 2, js_mat_funcs, 2),
    JS_CFUNC_MAGIC_DEF("rowRange", 2, js_mat_funcs, 3),
    JS_CFUNC_MAGIC_DEF("at", 1, js_mat_funcs, 4),
    JS_CFUNC_MAGIC_DEF("clone", 0, js_mat_funcs, 5),
    JS_CFUNC_MAGIC_DEF("roi", 0, js_mat_funcs, 6),
    JS_CFUNC_DEF("findContours", 0, js_mat_findcontours),
    JS_CFUNC_DEF("toString", 0, js_mat_tostring),
    //    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "cv::Mat", JS_PROP_CONFIGURABLE)

};

int
js_mat_init(JSContext* ctx, void* m, const char* name, bool exp) {
  /* create the Mat class */
  JS_NewClassID(&js_mat_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_mat_class_id, &js_mat_class);

  mat_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx, mat_proto, js_mat_proto_funcs, countof(js_mat_proto_funcs));
  JS_SetClassProto(ctx, js_mat_class_id, mat_proto);

  mat_class = JS_NewCFunction2(ctx, js_mat_ctor, name, 2, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, mat_class, mat_proto);

  JS_SetPropertyStr(ctx, mat_class, "CV_8UC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_8U, 1)));
  JS_SetPropertyStr(ctx, mat_class, "CV_8UC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_8U, 2)));
  JS_SetPropertyStr(ctx, mat_class, "CV_8UC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_8U, 3)));
  JS_SetPropertyStr(ctx, mat_class, "CV_8UC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_8U, 4)));
  JS_SetPropertyStr(ctx, mat_class, "CV_8SC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_8S, 1)));
  JS_SetPropertyStr(ctx, mat_class, "CV_8SC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_8S, 2)));
  JS_SetPropertyStr(ctx, mat_class, "CV_8SC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_8S, 3)));
  JS_SetPropertyStr(ctx, mat_class, "CV_8SC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_8S, 4)));
  JS_SetPropertyStr(ctx, mat_class, "CV_16UC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_16U, 1)));
  JS_SetPropertyStr(ctx, mat_class, "CV_16UC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_16U, 2)));
  JS_SetPropertyStr(ctx, mat_class, "CV_16UC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_16U, 3)));
  JS_SetPropertyStr(ctx, mat_class, "CV_16UC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_16U, 4)));
  JS_SetPropertyStr(ctx, mat_class, "CV_16SC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_16S, 1)));
  JS_SetPropertyStr(ctx, mat_class, "CV_16SC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_16S, 2)));
  JS_SetPropertyStr(ctx, mat_class, "CV_16SC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_16S, 3)));
  JS_SetPropertyStr(ctx, mat_class, "CV_16SC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_16S, 4)));
  JS_SetPropertyStr(ctx, mat_class, "CV_32SC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_32S, 1)));
  JS_SetPropertyStr(ctx, mat_class, "CV_32SC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_32S, 2)));
  JS_SetPropertyStr(ctx, mat_class, "CV_32SC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_32S, 3)));
  JS_SetPropertyStr(ctx, mat_class, "CV_32SC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_32S, 4)));
  JS_SetPropertyStr(ctx, mat_class, "CV_32FC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_32F, 1)));
  JS_SetPropertyStr(ctx, mat_class, "CV_32FC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_32F, 2)));
  JS_SetPropertyStr(ctx, mat_class, "CV_32FC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_32F, 3)));
  JS_SetPropertyStr(ctx, mat_class, "CV_32FC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_32F, 4)));
  JS_SetPropertyStr(ctx, mat_class, "CV_64FC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_64F, 1)));
  JS_SetPropertyStr(ctx, mat_class, "CV_64FC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_64F, 2)));
  JS_SetPropertyStr(ctx, mat_class, "CV_64FC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_64F, 3)));
  JS_SetPropertyStr(ctx, mat_class, "CV_64FC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_64F, 4)));

  if(exp)
    JS_SetModuleExport(ctx, static_cast<JSModuleDef*>(m), name, mat_class);
  else
    JS_SetPropertyStr(ctx, *static_cast<JSValue*>(m), name, mat_class);
  return 0;
}
