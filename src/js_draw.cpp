#include "./jsbindings.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "plot-cv.h"
#include "color.h"
#include "geometry.h"

#if defined(JS_DRAW_MODULE) || defined(quickjs_draw_EXPORTS)
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module
#else
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module_draw
#endif

extern "C" {

cv::Mat* dptr = 0;

static JSValue
js_draw_circle(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {
  cv::Mat* dst;
  int i = 0, ret = -1;
  point2i_type point;
  int radius = 0;
  cv::Scalar color;
  bool antialias = true;
  int thickness = -1;
  int lineType = cv::LINE_AA;

  if(argc > i && (dst = js_mat_data(ctx, argv[i])))
    i++;
  else
    dst = dptr;

  if(dst == nullptr)
    return JS_EXCEPTION;

  if(argc > i && js.is_point(argv[i]))
    js.get_point(argv[i++], point);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], radius);

  if(argc > i && js.is_color(argv[i]))
    js.get_color(argv[i++], color);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);
  /**/
  if(argc > i) {
    if(js.is_bool(argv[i])) {
      js.get_boolean(argv[i++], antialias);
      lineType = antialias ? cv::LINE_AA : cv::LINE_8;

    } else if(js.is_number(argv[i])) {
      js.get_number(argv[i++], lineType);
    }
  }

  cv::circle(*dst, point, radius, color, thickness < 0 ? cv::FILLED : thickness, lineType);
  return js._undefined;
}

static JSValue
js_draw_contour(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {
  cv::Mat* dst;
  int i = 0, ret = -1;
  contour2i_vector points;
  color_type color;
  int thickness = 1;
  bool antialias = true;

  if(argc > i && (dst = js_mat_data(ctx, argv[i])))
    i++;
  else
    dst = dptr;

  if(dst == nullptr)
    return JS_EXCEPTION;

  points.resize(1);
  if(argc > i && js.is_array(argv[i]))
    js.get_point_array(argv[i++], points[0]);

  if(argc > i && js.is_color(argv[i]))
    js.get_color(argv[i++], color);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  if(argc > i && js.is_bool(argv[i]))
    js.get_boolean(argv[i++], antialias);

  cv::drawContours(*dptr, points, -1, color, thickness, antialias ? cv::LINE_AA : cv::LINE_8);

  std::cerr << "draw_contour() ret:" << ret << " color: " << color << std::endl;
  return js._undefined;
}

static JSValue
js_draw_line(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {
  cv::Mat* dst;
  int i = 0, ret = -1;
  point2f_type points[2];
  color_type color;
  int thickness = 1;
  bool antialias = true;

  if(argc > i && (dst = js_mat_data(ctx, argv[i])))
    i++;
  else
    dst = dptr;

  if(dst == nullptr)
    return JS_EXCEPTION;

  if(argc > i && js.is_point(argv[i]))
    js.get_point(argv[i++], points[0]);

  if(argc > i && js.is_point(argv[1]))
    js.get_point(argv[i++], points[1]);

  if(argc > i && js.is_color(argv[i]))
    js.get_color(argv[i++], color);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  if(argc > i && js.is_bool(argv[i]))
    js.get_boolean(argv[i++], antialias);

  cv::line(*dptr, points[0], points[1], color, thickness, antialias ? cv::LINE_AA : cv::LINE_8);
  return js._undefined;
}

static JSValue
js_draw_polygon(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {
  cv::Mat* dst;
  int i = 0, ret = -1;
  point2i_vector points;
  cv::Scalar color;
  bool antialias = true;
  int thickness = -1;

  if(argc > i && (dst = js_mat_data(ctx, argv[i])))
    i++;
  else
    dst = dptr;

  if(argc > i && js.is_array_like(argv[i]))
    js.get_point_array(argv[i++], points);
  if(argc > i && js.is_color(argv[i]))
    js.get_color(argv[i++], color);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  if(argc > i && js.is_bool(argv[i]))
    js.get_boolean(argv[i++], antialias);

  if(dptr != nullptr) {
    const int size = points.size();
    int lineType = antialias ? cv::LINE_AA : cv::LINE_8;
    const point2i_type* pts = points.data();

    std::cerr << "drawPolygon() points: " << (points) << " color: " << to_string(color) << std::endl;

    // cv::fillPoly(*dptr, points, color, antialias ? cv::LINE_AA : cv::LINE_8);
    (thickness <= 0 ? cv::fillPoly(*dptr, &pts, &size, 1, color, lineType) : cv::polylines(*dptr, &pts, &size, 1, true, color, thickness, lineType));

    return js._undefined;
  }
  return JS_EXCEPTION;
}

static JSValue
js_draw_rect(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {
  cv::Mat* dst;
  int i = 0, ret = -1;
  cv::Rect2f rect;
  point2f_type points[2];
  color_type color;
  int thickness = 1;
  bool antialias = true;

  if(argc > i && (dst = js_mat_data(ctx, argv[i])))
    i++;
  else
    dst = dptr;

  if(dst == nullptr)
    return JS_EXCEPTION;

  if(argc > i && js.is_rect(argv[i]))
    js.get_rect(argv[i++], rect);

  if(argc > i && js.is_color(argv[i]))
    js.get_color(argv[i++], color);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  if(argc > i && js.is_bool(argv[i]))
    js.get_boolean(argv[i++], antialias);

  points[0].x = rect.x;
  points[0].y = rect.y;
  points[1].x = rect.x + rect.width;
  points[1].y = rect.y + rect.height;

  cv::rectangle(*dptr, points[0], points[1], color, thickness, antialias ? cv::LINE_AA : cv::LINE_8);

  return js._undefined;
}

JSValue draw_proto, draw_class;
JSClassID js_draw_class_id;

JSClassDef js_draw_class = {
    .class_name = "Draw",
    .finalizer = 0,
};

const JSCFunctionListEntry js_draw_proto_funcs[] = {
    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "Draw", JS_PROP_CONFIGURABLE),
};

const JSCFunctionListEntry js_draw_static_funcs[] = {
    JS_CFUNC_DEF("circle", 1, &js_draw_circle),
    JS_CFUNC_DEF("contour", 1, &js_draw_contour),
    JS_CFUNC_DEF("line", 1, &js_draw_line),
    JS_CFUNC_DEF("polygon", 1, &js_draw_polygon),
    JS_CFUNC_DEF("rect", 1, &js_draw_rect),
};

const JSCFunctionListEntry js_draw_global_funcs[] = {
    JS_CFUNC_DEF("drawCircle", 1, &js_draw_circle),
    JS_CFUNC_DEF("drawContour", 1, &js_draw_contour),
    JS_CFUNC_DEF("drawLine", 1, &js_draw_line),
    JS_CFUNC_DEF("drawPolygon", 1, &js_draw_polygon),
    JS_CFUNC_DEF("drawRect", 1, &js_draw_rect),
};

static JSValue
js_draw_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSValue obj = JS_UNDEFINED;
  JSValue proto;

  /* using new_target to get the prototype is necessary when the
     class is extended. */
  proto = JS_GetPropertyStr(ctx, new_target, "prototype");
  if(JS_IsException(proto))
    goto fail;
  obj = JS_NewObjectProtoClass(ctx, proto, js_draw_class_id);
  JS_FreeValue(ctx, proto);
  if(JS_IsException(obj))
    goto fail;
  return obj;
fail:
  JS_FreeValue(ctx, obj);
  return JS_EXCEPTION;
}

int
js_draw_init(JSContext* ctx, JSModuleDef* m) {

  /* create the Draw class */
  JS_NewClassID(&js_draw_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_draw_class_id, &js_draw_class);

  draw_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx, draw_proto, js_draw_proto_funcs, countof(js_draw_proto_funcs));
  JS_SetClassProto(ctx, js_draw_class_id, draw_proto);

  draw_class = JS_NewCFunction2(ctx, js_draw_ctor, "Draw", 2, JS_CFUNC_constructor, 0);

  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, draw_class, draw_proto);
  JS_SetPropertyFunctionList(ctx, draw_class, js_draw_static_funcs, countof(js_draw_static_funcs));

  if(m) {
    JS_SetModuleExportList(ctx, m, js_draw_global_funcs, countof(js_draw_global_funcs));
    JS_SetModuleExport(ctx, m, "Draw", draw_class);
  }
  /*  else
      JS_SetPropertyStr(ctx, *static_cast<JSValue*>(m), "Draw", draw_class);*/
  return 0;
}

JSModuleDef*
JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_draw_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "Draw");
  JS_AddModuleExportList(ctx, m, js_draw_global_funcs, countof(js_draw_global_funcs));
  return m;
}

void
js_draw_constructor(JSContext* ctx, JSValue parent, const char* name) {
  if(JS_IsUndefined(draw_class))
    js_draw_init(ctx, 0);

  JS_SetPropertyStr(ctx, parent, name ? name : "Draw", draw_class);
}
}
