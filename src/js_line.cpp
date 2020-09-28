#include "./jsbindings.h"

#if defined(JS_LINE_MODULE) || defined(quickjs_line_EXPORTS)
#define JS_INIT_MODULE VISIBLE js_init_module
#else
#define JS_INIT_MODULE VISIBLE js_init_module_line
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
  if(JS_ToFloat64(ctx, &s->arr[0], argv[0]))
    goto fail;
  if(JS_ToFloat64(ctx, &s->arr[1], argv[1]))
    goto fail;
  if(JS_ToFloat64(ctx, &s->arr[2], argv[2]))
    goto fail;
  if(JS_ToFloat64(ctx, &s->arr[3], argv[3]))
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
js_line_data(JSContext* ctx, JSValueConst val) {
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
    ret = JS_NewFloat64(ctx, s->vec[0]);
  else if(magic == 1)
    ret = JS_NewFloat64(ctx, s->vec[1]);
  else if(magic == 2)
    ret = JS_NewFloat64(ctx, s->vec[2]);
  else if(magic == 3)
    ret = JS_NewFloat64(ctx, s->vec[3]);
  return ret;
}

static JSValue
js_line_get_ab(JSContext* ctx, JSValueConst this_val, int magic) {
  JSValue ret = JS_UNDEFINED;
  JSLineData* s = static_cast<JSLineData*>(JS_GetOpaque2(ctx, this_val, js_line_class_id));
  if(!s)
    ret = JS_EXCEPTION;
  else if(magic == 0)
    ret = js_point_new(ctx, s->vec[0], s->vec[1]);
  else if(magic == 1)
    ret = js_point_new(ctx, s->vec[2], s->vec[3]);

  return ret;
}

static JSValue
js_line_set_xy12(JSContext* ctx, JSValueConst this_val, JSValueConst val, int magic) {
  JSLineData* s = static_cast<JSLineData*>(JS_GetOpaque2(ctx, this_val, js_line_class_id));
  double v;
  if(!s)
    return JS_EXCEPTION;
  if(JS_ToFloat64(ctx, &v, val))
    return JS_EXCEPTION;
  if(magic == 0)
    s->vec[0] = v;
  else if(magic == 1)
    s->vec[1] = v;
  else if(magic == 2)
    s->vec[2] = v;
  else if(magic == 3)
    s->vec[3] = v;

  return JS_UNDEFINED;
}

static JSValue
js_line_set_ab(JSContext* ctx, JSValueConst this_val, JSValueConst val, int magic) {
  JSLineData* s = static_cast<JSLineData*>(JS_GetOpaque2(ctx, this_val, js_line_class_id));
  JSPointData pt = js_point_get(ctx, val);

  if(!s)
    return JS_EXCEPTION;

  if(magic == 0) {
    s->vec[0] = pt.x;
    s->vec[1] = pt.y;

  } else if(magic == 1) {
    s->vec[2] = pt.x;
    s->vec[3] = pt.y;
  }

  return JS_UNDEFINED;
}

static JSValue
js_line_points(JSContext* ctx, JSValueConst line, int argc, JSValueConst* argv) {
  JSLineData* s = static_cast<JSLineData*>(JS_GetOpaque2(ctx, line, js_line_class_id));
  JSValue obj = JS_EXCEPTION, p;
  int i;
  obj = JS_NewArray(ctx);
  if(!JS_IsException(obj)) {
    std::pair<cv::Point2d, cv::Point2d> points = s->pt;
    JS_SetPropertyUint32(ctx, obj, 0, js_point_new(ctx, points.first.x, points.first.y));
    JS_SetPropertyUint32(ctx, obj, 1, js_point_new(ctx, points.second.x, points.second.y));
  }
  return obj;
}
static JSValue
js_line_array(JSContext* ctx, JSValueConst line, int argc, JSValueConst* arg) {
  JSLineData* s = static_cast<JSLineData*>(JS_GetOpaque2(ctx, line, js_line_class_id));
  JSValue obj = JS_EXCEPTION, p;
  int i;

  obj = JS_NewArray(ctx);
  if(!JS_IsException(obj)) {

    for(i = 0; i < 4; i++) JS_SetPropertyUint32(ctx, obj, i, JS_NewFloat64(ctx, s->arr[i]));
  }
  return obj;
}

static JSValue
js_call_method(JSContext* ctx, JSValue obj, const char* name, int argc, JSValueConst* argv) {
  JSValue fn, ret = JS_UNDEFINED;

  fn = JS_GetPropertyStr(ctx, obj, name);
  if(!JS_IsUndefined(fn))
    ret = JS_Call(ctx, fn, obj, argc, argv);
  return ret;
}

#define JS_LINE_AS_POINTS 0x01
#define JS_LINE_AS_VECTOR 0x02

#define JS_LINE_GET_ITERATOR 0x80
#define JS_LINE_TO_STRING 0x40
static JSValue
js_line_iterator(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSLineData* s = static_cast<JSLineData*>(JS_GetOpaque2(ctx, this_val, js_line_class_id));
  JSValue method, ret = JS_UNDEFINED;

  if(magic & JS_LINE_AS_POINTS)
    ret = js_line_points(ctx, this_val, argc, argv);
  if(magic & JS_LINE_AS_VECTOR)
    ret = js_line_array(ctx, this_val, argc, argv);

  if(magic & JS_LINE_GET_ITERATOR)
    ret = js_call_method(ctx, ret, "values", 0, NULL);

  if(magic & JS_LINE_TO_STRING) {
    JSValueConst args[1] = {0};
    args[0] = JS_NewString(ctx, " ");

    ret = js_call_method(ctx, ret, "join", 1, args);
  }

  return ret;
}
JSValue line_proto = JS_UNDEFINED, line_class = JS_UNDEFINED;
JSClassID js_line_class_id;

JSClassDef js_line_class = {
    "Line",
    .finalizer = js_line_finalizer,
};

const JSCFunctionListEntry js_line_proto_funcs[] = {
    JS_CGETSET_ENUMERABLE_DEF("x1", js_line_get_xy12, js_line_set_xy12, 0),
    JS_CGETSET_ENUMERABLE_DEF("y1", js_line_get_xy12, js_line_set_xy12, 1),
    JS_CGETSET_ENUMERABLE_DEF("x2", js_line_get_xy12, js_line_set_xy12, 2),
    JS_CGETSET_ENUMERABLE_DEF("y2", js_line_get_xy12, js_line_set_xy12, 3),
    JS_CGETSET_MAGIC_DEF("a", js_line_get_ab, js_line_set_ab, 0),
    JS_CGETSET_MAGIC_DEF("b", js_line_get_ab, js_line_set_ab, 1),
    JS_CGETSET_MAGIC_DEF("0", js_line_get_ab, js_line_set_ab, 0),
    JS_CGETSET_MAGIC_DEF("1", js_line_get_ab, js_line_set_ab, 1),
    JS_CFUNC_DEF("toArray", 0, js_line_array),
    JS_CFUNC_MAGIC_DEF("toPoints", 0, js_line_iterator, JS_LINE_AS_POINTS),
    JS_CFUNC_MAGIC_DEF("toString", 0, js_line_iterator, JS_LINE_AS_POINTS | JS_LINE_TO_STRING),
    JS_CFUNC_MAGIC_DEF("values", 0, js_line_iterator, JS_LINE_AS_VECTOR | JS_LINE_GET_ITERATOR),
    JS_ALIAS_DEF("[Symbol.iterator]", "values"),
    JS_ALIAS_DEF("[Symbol.toStringTag]", "toString"),
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
#define JS_INIT_MODULE VISIBLE js_init_module
#else
#define JS_INIT_MODULE VISIBLE js_init_module_line
#endif

JSModuleDef*
JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_line_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "Line");
  return m;
}
}
