#include "jsbindings.h"
#include "js_size.h"
#include "js_array.h"
#include "js_alloc.h"

#if defined(JS_SIZE_MODULE) || defined(quickjs_size_EXPORTS)
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module
#else
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module_size
#endif

static JSValue
js_size_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSSizeData<double>* s;
  JSValue obj = JS_UNDEFINED;
  JSValue proto;

  s = js_allocate<JSSizeData<double>>(ctx);
  if(!s)
    return JS_EXCEPTION;
  new(s) JSSizeData<double>();

  if(argc > 0) {
    if(!js_size_read(ctx, argv[0], s)) {
      if(JS_ToFloat64(ctx, &s->width, argv[0]))
        goto fail;
      if(argc < 2 || JS_ToFloat64(ctx, &s->height, argv[1]))
        goto fail;
    }
  }

  /* using new_target to get the prototype is necessary when the
     class is extended. */
  proto = JS_GetPropertyStr(ctx, new_target, "prototype");
  if(JS_IsException(proto))
    goto fail;
  obj = JS_NewObjectProtoClass(ctx, proto, js_size_class_id);
  JS_FreeValue(ctx, proto);
  if(JS_IsException(obj))
    goto fail;
  JS_SetOpaque(obj, s);
  return obj;
fail:
  js_deallocate(ctx, s);
  JS_FreeValue(ctx, obj);
  return JS_EXCEPTION;
}

VISIBLE JSSizeData<double>*
js_size_data(JSContext* ctx, JSValueConst val) {
  return static_cast<JSSizeData<double>*>(JS_GetOpaque2(ctx, val, js_size_class_id));
}

void
js_size_finalizer(JSRuntime* rt, JSValue val) {
  JSSizeData<double>* s = static_cast<JSSizeData<double>*>(JS_GetOpaque(val, js_size_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */
  js_free_rt(rt, s);
}

static JSValue
js_size_get_wh(JSContext* ctx, JSValueConst this_val, int magic) {
  JSSizeData<double>* s = js_size_data(ctx, this_val);
  if(!s)
    return JS_EXCEPTION;
  if(magic == 0)
    return JS_NewFloat64(ctx, s->width);
  else if(magic == 1)
    return JS_NewFloat64(ctx, s->height);
  return JS_UNDEFINED;
}

VISIBLE JSValue
js_size_new(JSContext* ctx, double w, double h) {
  JSValue ret;
  JSSizeData<double>* s;

  ret = JS_NewObjectProtoClass(ctx, size_proto, js_size_class_id);

  s = js_allocate<JSSizeData<double>>(ctx);
  s->width = w;
  s->height = h;

  JS_SetOpaque(ret, s);
  return ret;
}

VISIBLE JSValue
js_size_wrap(JSContext* ctx, const JSSizeData<double>& sz) {
  return js_size_new(ctx, sz.width, sz.height);
}

static JSValue
js_size_set_wh(JSContext* ctx, JSValueConst this_val, JSValueConst val, int magic) {
  JSSizeData<double>* s = js_size_data(ctx, this_val);
  double v;
  if(!s)
    return JS_EXCEPTION;
  if(JS_ToFloat64(ctx, &v, val))
    return JS_EXCEPTION;
  if(magic == 0)
    s->width = v;
  else
    s->height = v;
  return JS_UNDEFINED;
}

static JSValue
js_size_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSSizeData<double>* s = js_size_data(ctx, this_val);
  std::ostringstream os;
  JSValue wv, hv;
  double width = -1, height = -1;

  wv = JS_GetPropertyStr(ctx, this_val, "width");
  hv = JS_GetPropertyStr(ctx, this_val, "height");

  if(JS_IsNumber(wv) && JS_IsNumber(hv)) {
    JS_ToFloat64(ctx, &width, wv);
    JS_ToFloat64(ctx, &height, hv);
  } else if(s) {
    width = s->width;
    height = s->height;
  }

  os << "{width:" << width << ",height:" << height << "}" << std::endl;

  return JS_NewString(ctx, os.str().c_str());
}

static JSValue
js_size_to_array(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSSizeData<double>* s = js_size_data(ctx, this_val);
  std::array<double, 2> arr;

  arr[0] = s->width;
  arr[1] = s->height;

  return js_array<double>::from_sequence(ctx, arr.cbegin(), arr.cend());
}

static JSValue
js_size_mul(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSSizeData<double> size, *s = js_size_data(ctx, this_val);
  double factor;
  JS_ToFloat64(ctx, &factor, argv[0]);

  size = *s;
  size.width *= factor;
  size.height *= factor;

  return js_size_wrap(ctx, size);
}

static JSValue
js_size_div(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSSizeData<double> size, *s = js_size_data(ctx, this_val);
  double divider;
  JS_ToFloat64(ctx, &divider, argv[0]);

  size = *s;
  size.width /= divider;
  size.height /= divider;

  return js_size_wrap(ctx, size);
}

static JSValue
js_size_symbol_iterator(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSValue arr, iter, symbol;
  jsrt js(ctx);
  arr = js_size_to_array(ctx, this_val, argc, argv);
  symbol = js.get_symbol("iterator");
  iter = js.get_property(arr, symbol);
  return JS_Call(ctx, iter, arr, 0, argv);
}

JSValue size_proto, size_class;
JSClassID js_size_class_id;

JSClassDef js_size_class = {
    .class_name = "Size",
    .finalizer = js_size_finalizer,
};

const JSCFunctionListEntry js_size_proto_funcs[] = {
    JS_CGETSET_ENUMERABLE_DEF("width", js_size_get_wh, js_size_set_wh, 0),
    JS_CGETSET_ENUMERABLE_DEF("height", js_size_get_wh, js_size_set_wh, 1),
    JS_CFUNC_DEF("toString", 0, js_size_to_string),
    JS_CFUNC_DEF("toArray", 0, js_size_to_array),
    JS_CFUNC_DEF("mul", 1, js_size_mul),
    JS_CFUNC_DEF("div", 1, js_size_div),
    JS_ALIAS_DEF("values", "toArray"),
    JS_CFUNC_DEF("[Symbol.iterator]", 0, js_size_symbol_iterator),
    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "Size", JS_PROP_CONFIGURABLE),
};

int
js_size_init(JSContext* ctx, JSModuleDef* m) {

  /* create the Size class */
  JS_NewClassID(&js_size_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_size_class_id, &js_size_class);

  size_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx, size_proto, js_size_proto_funcs, countof(js_size_proto_funcs));
  JS_SetClassProto(ctx, js_size_class_id, size_proto);

  size_class = JS_NewCFunction2(ctx, js_size_ctor, "Size", 0, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, size_class, size_proto);

  if(m)
    JS_SetModuleExport(ctx, m, "Size", size_class);
  /*else
    JS_SetPropertyStr(ctx, *static_cast<JSValue*>(m), name, size_class);*/
  return 0;
}

void
js_size_constructor(JSContext* ctx, JSValue parent, const char* name) {
  if(JS_IsUndefined(size_class))
    js_size_init(ctx, 0);

  JS_SetPropertyStr(ctx, parent, name ? name : "Size", size_class);
}

#ifdef JS_SIZE_MODULE
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module
#else
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module_size
#endif

extern "C" JSModuleDef*
JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_size_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "Size");
  return m;
}
