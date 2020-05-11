#include "./jsbindings.h"

#if defined(JS_LINE_MODULE) || defined(quickjs_line_EXPORTS)
#define JS_INIT_MODULE js_init_module
#else
#define JS_INIT_MODULE js_init_module_line
#endif

extern "C" {

static JSValue
js_line_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSLineData* s;
  JSValue obj = JS_UNDEFINED;
  JSValue proto;

  s = static_cast<JSLineData*>(js_mallocz(ctx, sizeof(*s)));
  if(!s)
    return JS_EXCEPTION;
  if(JS_ToFloat64(ctx, &s->operator[](0), argv[0]))
    goto fail;
  if(JS_ToFloat64(ctx, &s->operator[](1), argv[1]))
    goto fail;
  if(JS_ToFloat64(ctx, &s->operator[](2), argv[2]))
    goto fail;
  if(JS_ToFloat64(ctx, &s->operator[](3), argv[3]))
    goto fail;
  /* using new_target to get the prototype is necessary when the
     class is extended. */
  proto = JS_GetPropertyStr(ctx, new_target, "prototype");
  if(JS_IsException(proto))
    goto fail;
  obj = JS_NewObjectProtoClass(ctx, proto, js_line_class_id);
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

JSLineData*
js_line_data(JSContext* ctx, JSValue val) {
  return static_cast<JSLineData*>(JS_GetOpaque2(ctx, val, js_line_class_id));
}

void
js_line_finalizer(JSRuntime* rt, JSValue val) {
  JSLineData* s = static_cast<JSLineData*>(JS_GetOpaque(val, js_line_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */
  js_free_rt(rt, s);
}

static JSValue
js_line_get_xy12(JSContext* ctx, JSValueConst this_val, int magic) {
  JSValue ret = JS_UNDEFINED;
  JSLineData* s = static_cast<JSLineData*>(JS_GetOpaque2(ctx, this_val, js_line_class_id));
  if(!s)
    ret = JS_EXCEPTION;
  else if(magic == 0)
    ret = JS_NewFloat64(ctx, s->operator[](0));
  else if(magic == 1)
    ret = JS_NewFloat64(ctx, s->operator[](1));
  else if(magic == 2)
    ret = JS_NewFloat64(ctx, s->operator[](2));
  else if(magic == 3)
    ret = JS_NewFloat64(ctx, s->operator[](3));
  return ret;
}

static JSValue
js_line_set_xy12(JSContext* ctx, JSValueConst this_val, JSValue val, int magic) {
  JSLineData* s = static_cast<JSLineData*>(JS_GetOpaque2(ctx, this_val, js_line_class_id));
  double v;
  if(!s)
    return JS_EXCEPTION;
  if(JS_ToFloat64(ctx, &v, val))
    return JS_EXCEPTION;
  if(magic == 0)
    s->operator[](0) = v;
  else if(magic == 1)
    s->operator[](1) = v;
  else if(magic == 2)
    s->operator[](2) = v;
  else if(magic == 3)
    s->operator[](3) = v;

  return JS_UNDEFINED;
}

static JSValue
js_line_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSLineData* s = static_cast<JSLineData*>(JS_GetOpaque2(ctx, this_val, js_line_class_id));
  std::ostringstream os;
  JSValue x1v, y1v, x2v, y2v;
  double x1 = -1, y1 = -1, x2 = 0, y2 = 0;
  /* if(!s)
     return JS_EXCEPTION;*/

  if(s) {
    x1 = s->operator[](0);
    y1 = s->operator[](1);
    x2 = s->operator[](2);
    y2 = s->operator[](3);
  } else {
    x1v = JS_GetPropertyStr(ctx, this_val, "x1");
    y1v = JS_GetPropertyStr(ctx, this_val, "y1");
    x2v = JS_GetPropertyStr(ctx, this_val, "x2");
    y2v = JS_GetPropertyStr(ctx, this_val, "y2");

    if(JS_IsNumber(x1v) && JS_IsNumber(y1v) && JS_IsNumber(x2v) && JS_IsNumber(y2v)) {
      JS_ToFloat64(ctx, &x1, x1v);
      JS_ToFloat64(ctx, &y1, y1v);
      JS_ToFloat64(ctx, &x2, x2v);
      JS_ToFloat64(ctx, &y2, y2v);
    }
  }

  os << "{x1:" << x1 << ",y1:" << y1 << ",x2:" << x2 << ",y2:" << y2 << "}" << std::endl;

  return JS_NewString(ctx, os.str().c_str());
}

JSValue line_proto = JS_UNDEFINED, line_class = JS_UNDEFINED;
JSClassID js_line_class_id;

JSClassDef js_line_class = {
    "Line",
    .finalizer = js_line_finalizer,
};

const JSCFunctionListEntry js_line_proto_funcs[] = {
    JS_CGETSET_MAGIC_DEF("x1", js_line_get_xy12, js_line_set_xy12, 0),
    JS_CGETSET_MAGIC_DEF("y1", js_line_get_xy12, js_line_set_xy12, 1),
    JS_CGETSET_MAGIC_DEF("x2", js_line_get_xy12, js_line_set_xy12, 2),
    JS_CGETSET_MAGIC_DEF("y2", js_line_get_xy12, js_line_set_xy12, 3),
    /*  JS_CGETSET_MAGIC_DEF("x2", js_line_get_xy12, js_line_set_xy12, 2),
       JS_CGETSET_MAGIC_DEF("y2", js_line_get_xy12, js_line_set_xy12, 3),
        JS_ALIAS_DEF("x1", "x1"),
       JS_ALIAS_DEF("y1", "y1"),*/
    JS_CFUNC_DEF("toString", 0, js_line_to_string),
};

int
js_line_init(JSContext* ctx, JSModuleDef* m) {

  /* create the Line class */
  JS_NewClassID(&js_line_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_line_class_id, &js_line_class);

  line_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx, line_proto, js_line_proto_funcs, countof(js_line_proto_funcs));
  JS_SetClassProto(ctx, js_line_class_id, line_proto);

  line_class = JS_NewCFunction2(ctx, js_line_ctor, "Line", 2, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, line_class, line_proto);

  if(m)
    JS_SetModuleExport(ctx, m, "Line", line_class);

  return 0;
}

void
js_line_constructor(JSContext* ctx, JSValue parent, const char* name) {
  if(JS_IsUndefined(line_class))
    js_line_init(ctx, 0);

  JS_SetPropertyStr(ctx, parent, name ? name : "Line", line_class);
}
#ifdef JS_LINE_MODULE
#define JS_INIT_MODULE js_init_module
#else
#define JS_INIT_MODULE js_init_module_line
#endif

JSModuleDef* __attribute__((visibility("default"))) JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_line_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "Line");
  return m;
}
}
