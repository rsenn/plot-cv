#include "color.h"
#include "geometry.h"
#include "jsbindings.h"
#include "plot-cv.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

extern jsrt js;
extern cv::Mat* mptr;

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

  cv::line(*mptr, points[0], points[1], color, thickness, antialias ? cv::LINE_AA : cv::LINE_8);
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
      *mptr, points[0], points[1], color, thickness, antialias ? cv::LINE_AA : cv::LINE_8);

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

  if(mptr != nullptr)
    cv::drawContours(*mptr, points, -1, color, thickness, antialias ? cv::LINE_AA : cv::LINE_8);

  logfile << "draw_contour() ret:" << ret << " color: " << color << std::endl;
  return js._true;
}

JSValue
js_draw_polygon(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv) {

  int i = 0, ret = -1;
  point_vector points;
  cv::Scalar color;
  bool antialias = true;

  if(argc > i && js.is_array_like(argv[i]))
    js.get_point_array(argv[i++], points);

  if(argc > i && js.is_array(argv[i]))
    js.get_int_array(argv[i++], color);

  if(argc > i && js.is_boolean(argv[i]))
    js.get_boolean(argv[i++], antialias);

  if(mptr != nullptr) {
    const int size = points.size();
    int lineType = antialias ? cv::LINE_AA : cv::LINE_8;
    const cv::Point* pts = points.data();

    std::cerr << "fillPoly() points: " << to_string(points) << " color: " << to_string(color)
              << std::endl;

    // cv::fillPoly(*mptr, points, color, antialias ? cv::LINE_AA : cv::LINE_8);
    cv::fillPoly(*mptr, &pts, &size, 1, color, lineType);
  }
  return js._true;
}
}