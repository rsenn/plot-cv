#include "color.h"
#include "geometry.h"
#include "jsbindings.h"
#include "plot-cv.h"
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

/*
 * QuickJS: Example of C module with a class
 *
 * Copyright (c) 2019 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
/*#include "../quickjs.h"
#include <math.h>
*/
#define countof(x) (sizeof(x) / sizeof((x)[0]))

/* Point Class */

typedef cv::Point2d JSPointData;
typedef cv::Rect2d JSRectData;

JSClassID js_point_class_id;
JSClassID js_rect_class_id;

JSValue point_proto, point_class;
JSValue rect_proto, rect_class;
JSValue point_iterator_proto, point_iterator_class;
JSValue contour_proto, contour_class;

void
js_point_finalizer(JSRuntime* rt, JSValue val) {
  JSPointData* s = static_cast<JSPointData*>(JS_GetOpaque(val, js_point_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */
  js_free_rt(rt, s);
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
  JSPointData* s = static_cast<JSPointData*>(JS_GetOpaque2(ctx, this_val, js_point_class_id));
  if(!s)
    return JS_EXCEPTION;
  if(magic == 0)
    return JS_NewFloat64(ctx, s->x);
  else
    return JS_NewFloat64(ctx, s->y);
}

JSValue
js_point_set_xy(JSContext* ctx, JSValueConst this_val, JSValue val, int magic) {
  JSPointData* s = static_cast<JSPointData*>(JS_GetOpaque2(ctx, this_val, js_point_class_id));
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
  JSPointData* s = static_cast<JSPointData*>(JS_GetOpaque2(ctx, this_val, js_point_class_id));
  if(!s)
    return JS_EXCEPTION;
  return JS_NewFloat64(ctx, sqrt((double)s->x * s->x + (double)s->y * s->y));
}

JSValue
js_point_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = static_cast<JSPointData*>(JS_GetOpaque2(ctx, this_val, js_point_class_id));
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
    JS_CFUNC_DEF("norm", 0, js_point_norm),
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

  return JS_UNDEFINED;
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
typedef std::vector<JSPointData> JSContourData;

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

static JSValue
js_create_point_iterator(
    JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSContourData* s = static_cast<JSContourData*>(JS_GetOpaque2(ctx, this_val, js_contour_class_id));
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
js_contour_push(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* v;
  int i;
  double x, y;
  JSValue xv, yv;
  JSPointData point;

  v = static_cast<JSContourData*>(JS_GetOpaque2(ctx, this_val, js_contour_class_id));
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

  v = static_cast<JSContourData*>(JS_GetOpaque2(ctx, this_val, js_contour_class_id));
  if(!v)
    return JS_EXCEPTION;

  JS_ToInt64(ctx, &i, argv[0]);

  ret = JS_NewObjectProtoClass(ctx, point_proto, js_point_class_id);

  point = static_cast<JSPointData*>(js_mallocz(ctx, sizeof(*point)));

  *point = (*v)[i];
  JS_SetOpaque(ret, point);

  return ret;
}

JSValue
js_contour_pop(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* v;
  JSValue ret;
  JSValue x, y;
  JSPointData *ptr, point;
  int64_t n = 0;

  v = static_cast<JSContourData*>(JS_GetOpaque2(ctx, this_val, js_contour_class_id));
  if(!v)
    return JS_EXCEPTION;
  n = v->size();
  if(n > 0) {
    point = (*v)[n - 1];
    v->pop_back();
  } else {
    return JS_EXCEPTION;
  }

  ret = JS_NewObjectProtoClass(ctx, point_proto, js_point_class_id);
  ptr = static_cast<JSPointData*>(js_mallocz(ctx, sizeof(point)));
  *ptr = point;
  JS_SetOpaque(ret, ptr);

  return ret;
}

JSValue
js_contour_length(JSContext* ctx, JSValueConst this_val) {
  JSContourData* v;
  JSValue ret;
  v = static_cast<JSContourData*>(JS_GetOpaque2(ctx, this_val, js_contour_class_id));
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

  v = static_cast<JSContourData*>(JS_GetOpaque2(ctx, this_val, js_contour_class_id));
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
js_contour_approxpolydp(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSValue ret = JS_UNDEFINED;
  double epsilon;
  bool closed = false;
  std::vector<cv::Point> curve;
  std::vector<cv::Point2f> approxCurve;
  JSContourData *out, *v;

  v = static_cast<JSContourData*>(JS_GetOpaque2(ctx, this_val, js_contour_class_id));

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
js_contour_boundingrect(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSValue ret = JS_UNDEFINED;
  cv::Rect2f rect;
  std::vector<cv::Point2f> curve;
  JSContourData* v;
  JSRectData* r;

  v = static_cast<JSContourData*>(JS_GetOpaque2(ctx, this_val, js_contour_class_id));

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
js_contour_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* s = static_cast<JSContourData*>(JS_GetOpaque2(ctx, this_val, js_contour_class_id));
  std::ostringstream os;
  if(!s)
    return JS_EXCEPTION;

  // os << "{x:" << s->x << ",y:" << s->y << "}" << std::endl;

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
    JS_CFUNC_DEF("approxPolyDP", 1, js_contour_approxpolydp),
    JS_CFUNC_DEF("boundingRect", 0, js_contour_boundingrect),

    JS_CFUNC_MAGIC_DEF("entries", 0, js_create_point_iterator, 0),
    JS_ALIAS_DEF("[Symbol.iterator]", "entries"),

    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "Contour", JS_PROP_CONFIGURABLE),

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
