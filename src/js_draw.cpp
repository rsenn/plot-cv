#include "jsbindings.hpp"
#include "js_point.hpp"
#include "js_array.hpp"
#include "js_rect.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "plot-cv.hpp"
#include "color.hpp"
#include "geometry.hpp"

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
  JSPointData<double> point;
  int32_t x, y;
  int radius = 0;
  JSColorData<double> color;
  bool antialias = true;
  int thickness = -1;
  int lineType = cv::LINE_AA;

  if(argc > i && (dst = js_mat_data(ctx, argv[i])))
    i++;
  else
    dst = dptr;

  if(dst == nullptr)
    return JS_EXCEPTION;

  if(js_point_read(ctx, argv[i], &point)) {
    x = point.x;
    y = point.y;
    i++;
  } else {
    JS_ToInt32(ctx, &x, argv[i]);
    JS_ToInt32(ctx, &y, argv[i + 1]);
    i += 2;
  }

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], radius);

  if(argc > i) {
    js_color_read(ctx, argv[i], &color);
    i++;
  }

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

  cv::circle(*dst,
             point,
             radius,
             *reinterpret_cast<cv::Scalar*>(&color),
             thickness < 0 ? cv::FILLED : thickness,
             lineType);
  return JS_UNDEFINED;
}

static JSValue
js_draw_contour(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {
  cv::Mat* dst;
  int i = 0, ret = -1;
  JSContoursData<int> contours;
  int32_t contourIdx = -1, lineType = cv::LINE_8;

  JSColorData<double> color;
  int thickness = 1;
  bool antialias = true;

  if(argc > i && (dst = js_mat_data(ctx, argv[i])))
    i++;
  else
    dst = dptr;

  if(dst == nullptr)
    return JS_EXCEPTION;

  if(i == argc || !JS_IsArray(ctx, argv[++i]))
    return JS_EXCEPTION;

  // js_array<JSContourData<int>>::to_vector(ctx, argv[i], contours);

  if(argc > i && JS_IsNumber(argv[++i])) {
    JS_ToInt32(ctx, &contourIdx, argv[i]);
  }

  if(argc > i) {
    js_color_read(ctx, argv[i], &color);
    i++;
  }

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  if(argc > i && JS_IsNumber(argv[++i])) {
    JS_ToInt32(ctx, &lineType, argv[i]);
  }
  std::cerr << "draw_contour() contours.length=" << contours.size() << " contourIdx=" << contourIdx
            << " thickness=" << thickness << std::endl;

  cv::drawContours(*dst, contours, contourIdx, *reinterpret_cast<cv::Scalar*>(&color), thickness, lineType);

  std::cerr << "draw_contour() ret:" << ret << " color: " << *reinterpret_cast<cv::Scalar*>(&color)
            << std::endl;
  return JS_UNDEFINED;
}

static JSValue
js_draw_line(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {
  cv::Mat* dst;
  int i = 0, ret = -1;
  JSPointData<double> points[2];
  JSColorData<uint8_t> color;
  cv::Scalar scalar;
  int thickness = 1;
  bool antialias = true;

  if(argc > i && (dst = js_mat_data(ctx, argv[i])))
    i++;
  else
    dst = dptr;

  if(dst == nullptr)
    return JS_EXCEPTION;

  if(argc > i)
    if(js_point_read(ctx, argv[i], &points[0])) {
      i++;
    }
  if(argc > i)
    if(js_point_read(ctx, argv[i], &points[1])) {
      i++;
    }

  if(argc > i)
    if(js_color_read(ctx, argv[i], &color)) {
      i++;

      scalar[0] = color.arr[0];
      scalar[1] = color.arr[1];
      scalar[2] = color.arr[2];
      scalar[3] = color.arr[3];
    }

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  if(argc > i && js.is_bool(argv[i]))
    js.get_boolean(argv[i++], antialias);

  cv::line(*dst, points[0], points[1], scalar, thickness, antialias ? cv::LINE_AA : cv::LINE_8);
  return JS_UNDEFINED;
}

static JSValue
js_draw_polygon(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {
  cv::Mat* dst;
  int i = 0, ret = -1;
  point_vector<int> points;
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
    const JSPointData<int>* pts = points.data();

    std::cerr << "drawPolygon() points: " << (points) << " color: " << to_string(color) << std::endl;

    // cv::fillPoly(*dptr, points, color, antialias ? cv::LINE_AA : cv::LINE_8);
    (thickness <= 0 ? cv::fillPoly(*dst, &pts, &size, 1, color, lineType)
                    : cv::polylines(*dst, &pts, &size, 1, true, color, thickness, lineType));

    return JS_UNDEFINED;
  }
  return JS_EXCEPTION;
}

static JSValue
js_draw_rect(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {
  cv::Mat* dst;
  int i = 0, ret = -1;
  JSRectData<double> rect;
  JSPointData<double> points[2];
  JSColorData<uint8_t> color;
  cv::Scalar scalar;
  int thickness = 1;
  bool antialias = true;

  if(argc > i && (dst = js_mat_data(ctx, argv[i])))
    i++;
  else
    dst = dptr;

  if(dst == nullptr)
    return JS_EXCEPTION;

  if(argc > i) {
    if(js_rect_read(ctx, argv[i], &rect))
      i++;
    //    js.get_rect(argv[i++], rect);
  }

  if(argc > i)
    if(js_color_read(ctx, argv[i], &color)) {
      i++;

      scalar[0] = color.arr[0];
      scalar[1] = color.arr[1];
      scalar[2] = color.arr[2];
      scalar[3] = color.arr[3];
    }

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  if(argc > i && js.is_bool(argv[i]))
    js.get_boolean(argv[i++], antialias);

  points[0].x = rect.x;
  points[0].y = rect.y;
  points[1].x = rect.x + rect.width;
  points[1].y = rect.y + rect.height;

  // cv::rectangle(*dst, points[0], points[1], color, thickness, antialias ? cv::LINE_AA : cv::LINE_8);
  cv::rectangle(*dst, rect, scalar, thickness, antialias ? cv::LINE_AA : cv::LINE_8);

  return JS_UNDEFINED;
}

static JSValue
js_put_text(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {
  cv::Mat* dst;
  int i = 0, ret = -1;
  JSColorData<double> color;
  JSPointData<double> point;
  const char* text;
  int32_t fontFace = cv::FONT_HERSHEY_SIMPLEX, thickness = 1, lineType = cv::LINE_8;
  double fontScale = 1;
  bool bottomLeftOrigin = false;

  if(argc > i && (dst = js_mat_data(ctx, argv[i])))
    i++;
  else
    dst = dptr;

  if(dst == nullptr)
    return JS_EXCEPTION;

  text = JS_ToCString(ctx, argv[i++]);

  if(js_point_read(ctx, argv[i], &point)) {
    i++;
  } else {
    int32_t x, y;
    JS_ToInt32(ctx, &x, argv[i]);
    JS_ToInt32(ctx, &y, argv[i + 1]);

    point.x = x;
    point.y = y;
    i += 2;
  }

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], fontFace);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], fontScale);

  if(argc > i) {
    js_color_read(ctx, argv[i], &color);
    i++;
  }

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], lineType);

  if(argc > i)
    bottomLeftOrigin = JS_ToBool(ctx, argv[i++]);

  cv::putText(*dst,
              text,
              point,
              fontFace,
              fontScale,
              *reinterpret_cast<cv::Scalar*>(&color),
              thickness,
              lineType,
              bottomLeftOrigin);

  return JS_UNDEFINED;
}

static JSValue
js_get_text_size(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {
  int i = 0, baseline = 0;
  JSSizeData<double> size;
  const char* text;
  int32_t fontFace = cv::FONT_HERSHEY_SIMPLEX, thickness = 1;
  double fontScale = 1;
  JSValue baselineVal;
  std::array<int32_t, 2> dim;

  text = JS_ToCString(ctx, argv[i++]);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], fontFace);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], fontScale);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  size = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);

  baselineVal = JS_NewInt32(ctx, baseline);

  if(JS_IsFunction(ctx, argv[i])) {
    JS_Call(ctx, argv[i], JS_UNDEFINED, 1, const_cast<JSValueConst*>(&baselineVal));
  } else if(JS_IsArray(ctx, argv[i])) {
    JS_SetPropertyUint32(ctx, argv[i], 0, baselineVal);
  } else if(JS_IsObject(argv[i])) {
    JS_SetPropertyStr(ctx, argv[i], "y", baselineVal);
  }
  dim[0] = size.width;
  dim[1] = size.height;

  return js_array<int32_t>::from_sequence(ctx, dim.cbegin(), dim.cend());
}

static JSValue
js_get_font_scale_from_height(JSContext* ctx,
                              jsrt::const_value this_val,
                              int argc,
                              jsrt::const_value* argv) {
  int i = 0, baseline = 0;
  int32_t fontFace, pixelHeight, thickness = 1;
  double fontScale;

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], fontFace);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], pixelHeight);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  fontScale = cv::getFontScaleFromHeight(fontFace, pixelHeight, thickness);

  return JS_NewFloat64(ctx, fontScale);
}

JSValue draw_proto = JS_UNDEFINED, draw_class = JS_UNDEFINED;
JSClassID js_draw_class_id = 0;

JSClassDef js_draw_class = {
    .class_name = "Draw",
    .finalizer = 0,
};

const JSCFunctionListEntry js_draw_proto_funcs[] = {
    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "Draw", JS_PROP_CONFIGURABLE),
};

const JSCFunctionListEntry js_draw_static_funcs[] = {JS_CFUNC_DEF("circle", 1, &js_draw_circle),
                                                     JS_CFUNC_DEF("contour", 1, &js_draw_contour),
                                                     JS_CFUNC_DEF("line", 1, &js_draw_line),
                                                     JS_CFUNC_DEF("polygon", 1, &js_draw_polygon),
                                                     JS_CFUNC_DEF("rect", 1, &js_draw_rect),
                                                     JS_CFUNC_DEF("text", 2, &js_put_text),
                                                     JS_CFUNC_DEF("textSize", 5, &js_get_text_size),
                                                     JS_CFUNC_DEF("fontScaleFromHeight",
                                                                  2,
                                                                  &js_get_font_scale_from_height)};

const JSCFunctionListEntry js_draw_global_funcs[] = {
    JS_CFUNC_DEF("drawCircle", 1, &js_draw_circle),
    JS_CFUNC_DEF("drawContour", 1, &js_draw_contour),
    JS_CFUNC_DEF("drawLine", 1, &js_draw_line),
    JS_CFUNC_DEF("drawPolygon", 1, &js_draw_polygon),
    JS_CFUNC_DEF("drawRect", 1, &js_draw_rect),
    JS_CFUNC_DEF("putText", 2, &js_put_text),
    JS_CFUNC_DEF("getTextSize", 5, &js_get_text_size),
    JS_CFUNC_DEF("getFontScaleFromHeight", 2, &js_get_font_scale_from_height),
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

  if(js_draw_class_id == 0) {
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
  }

  if(m) {
    JS_SetModuleExportList(ctx, m, js_draw_static_funcs, countof(js_draw_static_funcs));
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
  JS_AddModuleExportList(ctx, m, js_draw_static_funcs, countof(js_draw_static_funcs));
  return m;
}

void
js_draw_constructor(JSContext* ctx, JSValue parent, const char* name) {
  if(JS_IsUndefined(draw_class))
    js_draw_init(ctx, 0);

  JS_SetPropertyStr(ctx, parent, name ? name : "Draw", draw_class);
}
}
