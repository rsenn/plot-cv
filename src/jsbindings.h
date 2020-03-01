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
JSValue js_point_ctor(JSContext*, JSValueConst new_target, int argc, JSValueConst* argv);
JSValue js_point_get_xy(JSContext*, JSValueConst this_val, int magic);
JSValue js_point_set_xy(JSContext*, JSValueConst this_val, JSValue val, int magic);
JSValue js_point_norm(JSContext*, JSValueConst this_val, int argc, JSValueConst* argv);
int js_point_init(JSContext*, void* m, const char* name, bool exp = true);

};

#endif