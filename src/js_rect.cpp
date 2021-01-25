#include "jsbindings.h"
#include "js_rect.h"
#include "js_size.h"
#include "js_point.h"
#include "js_alloc.h"

#if defined(JS_RECT_MODULE) || defined(quickjs_rect_EXPORTS)
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module
#else
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module_rect
#endif

extern "C" VISIBLE JSValue
js_rect_new(JSContext* ctx, double x, double y, double w, double h) {
  JSValue ret;
  JSRectData<double>* s;

  ret = JS_NewObjectProtoClass(ctx, rect_proto, js_rect_class_id);

  s = js_allocate<JSRectData<double>>(ctx);

  new(s) JSRectData<double>();
  s->x = x;
  s->y = y;
  s->width = w;
  s->height = h;

  JS_SetOpaque(ret, s);
  return ret;
}

VISIBLE JSValue
js_rect_wrap(JSContext* ctx, const JSRectData<double>& rect) {
  return js_rect_new(ctx, rect.x, rect.y, rect.width, rect.height);
}

static JSValue
js_rect_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  double x, y, w, h;
  JSRectData<double> rect;

  if(argc > 0) {
    if(!js_rect_read(ctx, argv[0], &rect)) {
      JSPointData<double> point;
      if(argc >= 2 && js_point_read(ctx, argv[0], &point)) {
        JSSizeData<double> size;
        JSPointData<double> point2;

        x = point.x;
        y = point.y;
        if(js_size_read(ctx, argv[1], &size)) {
          w = size.width;
          h = size.height;
        } else if(js_point_read(ctx, argv[1], &point2)) {
          w = point2.x - point.x;
          h = point2.y - point.y;
        } else {
          return JS_EXCEPTION;
        }
      } else {
        if(JS_ToFloat64(ctx, &x, argv[0]))
          return JS_EXCEPTION;
        if(argc < 2 || JS_ToFloat64(ctx, &y, argv[1]))
          return JS_EXCEPTION;
        if(argc < 3 || JS_ToFloat64(ctx, &w, argv[2]))
          return JS_EXCEPTION;
        if(argc < 4 || JS_ToFloat64(ctx, &h, argv[3]))
          return JS_EXCEPTION;
      }
    }
  }

  return js_rect_new(ctx, x, y, w, h);
}

VISIBLE JSRectData<double>*
js_rect_data(JSContext* ctx, JSValueConst val) {
  return static_cast<JSRectData<double>*>(JS_GetOpaque2(ctx, val, js_rect_class_id));
}

void
js_rect_finalizer(JSRuntime* rt, JSValue val) {
  JSRectData<double>* s = static_cast<JSRectData<double>*>(JS_GetOpaque(val, js_rect_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */

  s->~JSRectData<double>();
  js_deallocate(rt, s);
}

static JSValue
js_rect_get_xywh(JSContext* ctx, JSValueConst this_val, int magic) {
  JSValue ret = JS_UNDEFINED;
  JSRectData<double>* s = static_cast<JSRectData<double>*>(JS_GetOpaque2(ctx, this_val, js_rect_class_id));
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

static JSValue
js_rect_set_xywh(JSContext* ctx, JSValueConst this_val, JSValueConst val, int magic) {
  JSRectData<double>* s = static_cast<JSRectData<double>*>(JS_GetOpaque2(ctx, this_val, js_rect_class_id));
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

static JSValue
js_rect_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSRectData<double>* s = static_cast<JSRectData<double>*>(JS_GetOpaque2(ctx, this_val, js_rect_class_id));
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

static JSValue
js_rect_method(JSContext* ctx, JSValueConst rect, int argc, JSValueConst* argv, int magic) {
  JSRectData<double>* s = static_cast<JSRectData<double>*>(JS_GetOpaque2(ctx, rect, js_rect_class_id));
  JSValue ret = JS_UNDEFINED;
  JSPointData<double> point = js_point_get(ctx, argv[0]);
  if(magic == 0)
    ret = JS_NewBool(ctx, s->contains(point));
  if(magic == 1)
    ret = JS_NewBool(ctx, s->empty());
  if(magic == 2)
    ret = JS_NewFloat64(ctx, s->area());

  if(magic == 3 || magic == 4) {
    JSPointData<double> pt = magic == 3 ? s->br() : s->tl();

    ret = js_point_new(ctx, pt.x, pt.y);
  }
  if(magic == 5) {
    cv::Size2d sz = s->size();
    ret = js_size_new(ctx, sz.width, sz.height);
  }
  return ret;
}

JSValue rect_proto = JS_UNDEFINED, rect_class = JS_UNDEFINED;
JSClassID js_rect_class_id;

JSClassDef js_rect_class = {
    .class_name = "Rect",
    .finalizer = js_rect_finalizer,
};

const JSCFunctionListEntry js_rect_proto_funcs[] = {
    JS_CGETSET_ENUMERABLE_DEF("x", js_rect_get_xywh, js_rect_set_xywh, 0),
    JS_CGETSET_ENUMERABLE_DEF("y", js_rect_get_xywh, js_rect_set_xywh, 1),
    JS_CGETSET_ENUMERABLE_DEF("width", js_rect_get_xywh, js_rect_set_xywh, 2),
    JS_CGETSET_ENUMERABLE_DEF("height", js_rect_get_xywh, js_rect_set_xywh, 3),
    JS_CGETSET_MAGIC_DEF("x2", js_rect_get_xywh, js_rect_set_xywh, 4),
    JS_CGETSET_MAGIC_DEF("y2", js_rect_get_xywh, js_rect_set_xywh, 5),
    JS_ALIAS_DEF("x1", "x"),
    JS_ALIAS_DEF("y1", "y"),
    JS_CFUNC_MAGIC_DEF("contains", 0, js_rect_method, 0),
    JS_CFUNC_MAGIC_DEF("empty", 0, js_rect_method, 1),
    JS_CFUNC_MAGIC_DEF("area", 0, js_rect_method, 2),
    JS_CFUNC_MAGIC_DEF("br", 0, js_rect_method, 3),
    JS_CFUNC_MAGIC_DEF("tl", 0, js_rect_method, 4),
    JS_CFUNC_MAGIC_DEF("size", 0, js_rect_method, 5),
    JS_CFUNC_DEF("toString", 0, js_rect_to_string),
    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "Rect", JS_PROP_CONFIGURABLE)

};

int
js_rect_init(JSContext* ctx, JSModuleDef* m) {

  /* create the Rect class */
  JS_NewClassID(&js_rect_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_rect_class_id, &js_rect_class);

  rect_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx, rect_proto, js_rect_proto_funcs, countof(js_rect_proto_funcs));
  JS_SetClassProto(ctx, js_rect_class_id, rect_proto);

  rect_class = JS_NewCFunction2(ctx, js_rect_ctor, "Rect", 0, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, rect_class, rect_proto);

  if(m)
    JS_SetModuleExport(ctx, m, "Rect", rect_class);

  return 0;
}

void
js_rect_constructor(JSContext* ctx, JSValue parent, const char* name) {
  if(JS_IsUndefined(rect_class))
    js_rect_init(ctx, 0);

  JS_SetPropertyStr(ctx, parent, name ? name : "Rect", rect_class);
}

#ifdef JS_RECT_MODULE
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module
#else
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module_rect
#endif

extern "C" JSModuleDef*
JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_rect_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "Rect");
  return m;
}
