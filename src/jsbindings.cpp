#include "color.h"
#include "geometry.h"
#include "jsbindings.h"
#include "plot-cv.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

extern jsrt js;
extern cv::Mat* dptr;

extern "C" {

JSValue
js_draw_line(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {
  int ret = -1;
  cv::Point2f points[2];
  color_type color;
  int thickness = 1;
  bool antialias = true;

  if(argc > 0 && js.is_point(argv[0]))
    js.get_point(argv[0], points[0]);

  if(argc > 1 && js.is_point(argv[1]))
    js.get_point(argv[1], points[1]);

  if(argc > 2 && js.is_array(argv[2]))
    js.get_int_array(argv[2], color);

  if(argc > 3 && js.is_number(argv[3]))
    js.get_number(argv[3], thickness);

  if(argc > 4 && js.is_boolean(argv[4]))
    js.get_boolean(argv[4], antialias);

  cv::line(*dptr, points[0], points[1], color, thickness, antialias ? cv::LINE_AA : cv::LINE_8);
  return js._true;
}

JSValue
js_draw_rect(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {
  int i = 0, ret = -1;
  cv::Rect2f rect;
  cv::Point2f points[2];
  color_type color;
  int thickness = 1;
  bool antialias = true;

  if(argc > i && js.is_rect(argv[i]))
    js.get_rect(argv[i++], rect);

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

  int ret = -1;
  contour_vector points;
  color_type color;
  int thickness = 1;
  bool antialias = true;

  points.resize(1);
  if(argc >= 1 && js.is_array(argv[0]))
    js.get_point_array(argv[0], points[0]);

  if(argc >= 2 && js.is_array(argv[1]))
    js.get_int_array(argv[1], color);

  if(argc >= 3 && js.is_number(argv[2]))
    js.get_number(argv[2], thickness);

  if(argc >= 4 && js.is_boolean(argv[3]))
    js.get_boolean(argv[3], antialias);

  if(dptr != nullptr)
    cv::drawContours(*dptr, points, -1, color, thickness, antialias ? cv::LINE_AA : cv::LINE_8);

  logfile << "draw_contour() ret:" << ret << " color: " << color << std::endl;
  return js._true;
}

JSValue
js_draw_polygon(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {

  int i = 0, ret = -1;
  point_vector points;
  cv::Scalar color;
  bool antialias = true;
  int thickness = -1;

  if(argc > i && js.is_array_like(argv[i]))
    js.get_point_array(argv[i++], points);

  if(argc > i && js.is_array(argv[i]))
    js.get_int_array(argv[i++], color);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  if(argc > i && js.is_boolean(argv[i]))
    js.get_boolean(argv[i++], antialias);

  if(dptr != nullptr) {
    const int size = points.size();
    int lineType = antialias ? cv::LINE_AA : cv::LINE_8;
    const cv::Point* pts = points.data();

    std::cerr << "drawPolygon() points: " << (points) << " color: " << to_string(color) << std::endl;

    // cv::fillPoly(*dptr, points, color, antialias ? cv::LINE_AA : cv::LINE_8);
    (thickness <= 0 ? cv::fillPoly(*dptr, &pts, &size, 1, color, lineType) : cv::polylines(*dptr, &pts, &size, 1, true, color, thickness, lineType));
  }
  return js._true;
}


JSValue
js_draw_circle(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {

  int i = 0, ret = -1;
  cv::Point point;
int radius = 0;
  cv::Scalar color;

  bool antialias = true;
  int thickness = -1;

  if(argc > i && js.is_point(argv[i]))
    js.get_point(argv[i++], point);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], radius);


  if(argc > i && js.is_array(argv[i]))
    js.get_int_array(argv[i++], color);

  if(argc > i && js.is_number(argv[i]))
    js.get_number(argv[i++], thickness);

  if(argc > i && js.is_boolean(argv[i]))
    js.get_boolean(argv[i++], antialias);

  if(dptr != nullptr) {
    int lineType = antialias ? cv::LINE_AA : cv::LINE_8;

    std::cerr << "drawCircle() center: " << (point) << " radius: " <<radius << " color: " << to_string(color) << std::endl;

    // cv::fillPoly(*dptr, points, color, antialias ? cv::LINE_AA : cv::LINE_8);
    cv::circle(*dptr, point, radius, color, thickness < 0 ? cv::FILLED : thickness, lineType);
  }
  return js._true;
}
}