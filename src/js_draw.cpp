#include "./jsbindings.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "plot-cv.h"
#include "color.h"
#include "geometry.h"

#if defined(JS_DRAW_MODULE) || defined(quickjs_draw_EXPORTS)
#define JS_INIT_MODULE js_init_module
#else
#define JS_INIT_MODULE js_init_module_draw
#endif

extern "C" {

cv::Mat* dptr = 0;

static JSValue
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

    /* std::cerr << "drawCircle() center: " << (point) << " radius: " << radius
             << " color: " << to_string(color) << std::endl;*/

    // cv::fillPoly(*dptr, points, color, antialias ? cv::LINE_AA : cv::LINE_8);
    cv::circle(*dptr, point, radius, color, thickness < 0 ? cv::FILLED : thickness, lineType);
  }
  return js._true;
}

static JSValue
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

  std::cerr << "draw_contour() ret:" << ret << " color: " << color << std::endl;
  return js._true;
}

static JSValue
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

static JSValue
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

    std::cerr << "drawPolygon() points: " << (points) << " color: " << to_string(color) << std::endl;

    // cv::fillPoly(*dptr, points, color, antialias ? cv::LINE_AA : cv::LINE_8);
    (thickness <= 0 ? cv::fillPoly(*dptr, &pts, &size, 1, color, lineType) : cv::polylines(*dptr, &pts, &size, 1, true, color, thickness, lineType));
  }
  return js._true;
}

static JSValue
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

  cv::rectangle(*dptr, points[0], points[1], color, thickness, antialias ? cv::LINE_AA : cv::LINE_8);

  return js._true;
}

int
js_draw_init(JSContext* ctx, JSModuleDef* m) {

  if(m) {
    JS_SetModuleExport(ctx, m, "drawCircle", JS_NewCFunction(ctx, &js_draw_circle, "drawCircle", 5));
    JS_SetModuleExport(ctx, m, "drawContour", JS_NewCFunction(ctx, &js_draw_contour, "drawContour", 4));
    JS_SetModuleExport(ctx, m, "drawLine", JS_NewCFunction(ctx, &js_draw_line, "drawLine", 5));
    JS_SetModuleExport(ctx, m, "drawPolygon", JS_NewCFunction(ctx, &js_draw_polygon, "drawPolygon", 4));
    JS_SetModuleExport(ctx, m, "drawRect", JS_NewCFunction(ctx, &js_draw_rect, "drawRect", 4));
  }

  return 0;
}

int
js_draw_functions(JSContext* ctx, JSValue parent) {

  JS_SetPropertyStr(ctx, parent, "drawCircle", JS_NewCFunction(ctx, &js_draw_circle, "drawCircle", 5));
  JS_SetPropertyStr(ctx, parent, "drawContour", JS_NewCFunction(ctx, &js_draw_contour, "drawContour", 4));
  JS_SetPropertyStr(ctx, parent, "drawLine", JS_NewCFunction(ctx, &js_draw_line, "drawLine", 5));
  JS_SetPropertyStr(ctx, parent, "drawPolygon", JS_NewCFunction(ctx, &js_draw_polygon, "drawPolygon", 4));
  JS_SetPropertyStr(ctx, parent, "drawRect", JS_NewCFunction(ctx, &js_draw_rect, "drawRect", 4));

  return 0;
}

JSModuleDef* __attribute__((visibility("default"))) JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_draw_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "Draw");
  return m;
}
}
