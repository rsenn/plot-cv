#ifndef JSBINDINGS_H
#define JSBINDINGS_H
#include "js.h"

extern "C" {
JSValue js_draw_line(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv);
JSValue js_draw_rect(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv);
JSValue js_draw_contour(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv);
JSValue js_draw_polygon(JSContext* ctx, jsrt::const_value this_val, int argc, jsrt::const_value* argv);
};

#endif