#include "./jsbindings.h"
#include "js.h"
#include "quickjs/cutils.h"
#include "quickjs/quickjs.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#if defined(JS_POINT_MODULE) || defined(quickjs_point_EXPORTS)
#define JS_INIT_MODULE VISIBLE js_init_module
#else
#define JS_INIT_MODULE VISIBLE js_init_module_point
#endif

extern "C" {

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

static JSValue
js_point_cross(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = js_point_data(ctx, this_val);
  JSPointData* other = js_point_data(ctx, argv[0]);
  double retval;
  if(!s || !other)
    return JS_EXCEPTION;
  retval = s->cross(*other);
  return JS_NewFloat64(ctx, retval);
}

static JSValue
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

JSPointData*
js_point_data(JSContext* ctx, JSValue val) {
  return static_cast<JSPointData*>(JS_GetOpaque2(ctx, val, js_point_class_id));
}

static JSValue
js_point_ddot(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = js_point_data(ctx, this_val);
  JSPointData* other = js_point_data(ctx, argv[0]);
  double retval;
  if(!s || !other)
    return JS_EXCEPTION;
  retval = s->ddot(*other);
  return JS_NewFloat64(ctx, retval);
}

static JSValue
js_point_diff(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = js_point_data(ctx, this_val);
  JSPointData* other = js_point_data(ctx, argv[0]);

  JSValue ret;
  if(!s || !other)
    return JS_EXCEPTION;

  ret = js_point_new(ctx, s->x - other->x, s->y - other->y);
  return ret;
}

static void
js_point_finalizer(JSRuntime* rt, JSValue val) {
  JSPointData* s = static_cast<JSPointData*>(JS_GetOpaque(val, js_point_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */
  js_free_rt(rt, s);
}

static JSValue
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

static JSValue
js_point_inside(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = js_point_data(ctx, this_val);
  JSRectData r = js_rect_get(ctx, argv[0]);
  bool retval;
  if(!s /*|| !r*/)
    return JS_EXCEPTION;

  retval = s->inside(r);

  return JS_NewBool(ctx, retval);
}

static JSValue
js_point_norm(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = js_point_data(ctx, this_val);
  if(!s)
    return JS_EXCEPTION;
  return JS_NewFloat64(ctx, sqrt((double)s->x * s->x + (double)s->y * s->y));
}

static JSValue
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

static JSValue
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

static JSValue
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

static JSValue
js_point_sum(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData* s = js_point_data(ctx, this_val);
  JSPointData* other = js_point_data(ctx, argv[0]);

  JSValue ret;
  if(!s || !other)
    return JS_EXCEPTION;

  ret = js_point_new(ctx, s->x + other->x, s->y + other->y);
  return ret;
}

static JSValue
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

JSValue point_proto = JS_UNDEFINED, point_class = JS_UNDEFINED;
JSClassID js_point_class_id;

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
    // JS_CFUNC_DEF("getRotationMatrix2D", 0, js_point_getrotationmatrix2d),
    JS_CFUNC_DEF("toString", 0, js_point_to_string),
};

int
js_point_init(JSContext* ctx, JSModuleDef* m) {

  /* create the Point class */
  JS_NewClassID(&js_point_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_point_class_id, &js_point_class);

  point_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx, point_proto, js_point_proto_funcs, countof(js_point_proto_funcs));
  JS_SetClassProto(ctx, js_point_class_id, point_proto);

  point_class = JS_NewCFunction2(ctx, js_point_ctor, "Point", 2, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, point_class, point_proto);

  if(m)
    JS_SetModuleExport(ctx, m, "Point", point_class);
  /* else
     JS_SetPropertyStr(ctx, *static_cast<JSValue*>(m), name, point_class);*/
  return 0;
}

void
js_point_constructor(JSContext* ctx, JSValue parent, const char* name) {
  if(JS_IsUndefined(point_class))
    js_point_init(ctx, 0);

  JS_SetPropertyStr(ctx, parent, name ? name : "Point", point_class);
}

JSModuleDef* JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_point_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "Point");
  return m;
}
}
