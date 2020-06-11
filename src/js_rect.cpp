#include "./jsbindings.h"

#if defined(JS_RECT_MODULE) || defined(quickjs_rect_EXPORTS)
#define JS_INIT_MODULE VISIBLE js_init_module
#else
#define JS_INIT_MODULE VISIBLE js_init_module_rect
#endif

extern "C" {

static JSValue
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

JSRectData*
js_rect_data(JSContext* ctx, JSValue val) {
  return static_cast<JSRectData*>(JS_GetOpaque2(ctx, val, js_rect_class_id));
}

void
js_rect_finalizer(JSRuntime* rt, JSValue val) {
  JSRectData* s = static_cast<JSRectData*>(JS_GetOpaque(val, js_rect_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */
  js_free_rt(rt, s);
}

static JSValue
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
  else if(magic == 4)
    ret = JS_NewFloat64(ctx, s->x + s->width);
  else if(magic == 5)
    ret = JS_NewFloat64(ctx, s->y + s->height);
  return ret;
}

static JSValue
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
  else if(magic == 4)
    s->width = v - s->x;
  else if(magic == 5)
    s->height = v - s->y;

  return JS_UNDEFINED;
}

static JSValue
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

JSValue rect_proto = JS_UNDEFINED, rect_class = JS_UNDEFINED;
JSClassID js_rect_class_id;

JSClassDef js_rect_class = {
    "Rect",
    .finalizer = js_rect_finalizer,
};

const JSCFunctionListEntry js_rect_proto_funcs[] = {
    JS_CGETSET_MAGIC_DEF("x", js_rect_get_xywh, js_rect_set_xywh, 0),
    JS_CGETSET_MAGIC_DEF("y", js_rect_get_xywh, js_rect_set_xywh, 1),
    JS_CGETSET_MAGIC_DEF("width", js_rect_get_xywh, js_rect_set_xywh, 2),
    JS_CGETSET_MAGIC_DEF("height", js_rect_get_xywh, js_rect_set_xywh, 3),
    JS_CGETSET_MAGIC_DEF("x2", js_rect_get_xywh, js_rect_set_xywh, 4),
    JS_CGETSET_MAGIC_DEF("y2", js_rect_get_xywh, js_rect_set_xywh, 5),
    JS_ALIAS_DEF("x1", "x"),
    JS_ALIAS_DEF("y1", "y"),
    JS_CFUNC_DEF("toString", 0, js_rect_to_string),
};

int
js_rect_init(JSContext* ctx, JSModuleDef* m) {

  /* create the Rect class */
  JS_NewClassID(&js_rect_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_rect_class_id, &js_rect_class);

  rect_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx, rect_proto, js_rect_proto_funcs, countof(js_rect_proto_funcs));
  JS_SetClassProto(ctx, js_rect_class_id, rect_proto);

  rect_class = JS_NewCFunction2(ctx, js_rect_ctor, "Rect", 2, JS_CFUNC_constructor, 0);
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
#define JS_INIT_MODULE VISIBLE js_init_module
#else
#define JS_INIT_MODULE VISIBLE js_init_module_rect
#endif

JSModuleDef*
JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_rect_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "Rect");
  return m;
}
}
