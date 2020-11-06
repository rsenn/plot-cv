#include "./jsbindings.h"

#if defined(JS_VIDEO_CAPTURE_MODULE) || defined(quickjs_video_capture_EXPORTS)
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module
#else
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module_video_capture
#endif

JSClassID js_video_capture_class_id;
JSValue video_capture_proto = JS_UNDEFINED, video_capture_class = JS_UNDEFINED;

static JSValue
js_video_capture_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSVideoCaptureData* s;
  JSValue obj = JS_UNDEFINED;
  JSValue proto, ret;
  int32_t camID, apiPreference;

  s = static_cast<JSVideoCaptureData*>(js_mallocz(ctx, sizeof(*s)));

  new(s) JSVideoCaptureData();

  if(!s)
    return JS_EXCEPTION;

  if(!JS_ToInt32(ctx, &camID, argv[0])) {
    if(JS_ToInt32(ctx, &apiPreference, argv[1]))
      apiPreference = cv::CAP_ANY;

    s->open(camID, apiPreference);
  }

  /* using new_target to get the prototype is necessary when the
     class is extended. */
  proto = JS_GetPropertyStr(ctx, new_target, "prototype");
  if(JS_IsException(proto))
    goto fail;
  obj = JS_NewObjectProtoClass(ctx, proto, js_video_capture_class_id);
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

VISIBLE JSVideoCaptureData*
js_video_capture_data(JSContext* ctx, JSValueConst val) {
  return static_cast<JSVideoCaptureData*>(JS_GetOpaque2(ctx, val, js_video_capture_class_id));
}

void
js_video_capture_finalizer(JSRuntime* rt, JSValue val) {
  JSVideoCaptureData* s =
      static_cast<JSVideoCaptureData*>(JS_GetOpaque(val, js_video_capture_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */

  s->~JSVideoCaptureData();
  js_free_rt(rt, s);
}

static JSValue
js_video_capture_method(
    JSContext* ctx, JSValueConst video_capture, int argc, JSValueConst* argv, int magic) {
  JSVideoCaptureData* s = static_cast<JSVideoCaptureData*>(
      JS_GetOpaque2(ctx, video_capture, js_video_capture_class_id));
  JSValue ret = JS_UNDEFINED;
  JSPointData point = js_point_get(ctx, argv[0]);
  if(magic == 0) {
    int32_t propID;
    double value = 0;
    if(!JS_ToInt32(ctx, &propID, argv[0]))
      ret = JS_NewFloat64(ctx, s->get(propID));
    else
      ret = JS_EXCEPTION;
  }
  if(magic == 1) {
    int32_t propID;
    double value = 0;
    if(!JS_ToInt32(ctx, &propID, argv[0])) {
      JS_ToFloat64(ctx, &value, argv[1]);
      s->set(propID, value);
    } else
      ret = JS_EXCEPTION;
  }
  if(magic == 2) {
    std::string backend = s->getBackendName();
    ret = JS_NewString(ctx, backend.c_str());
  }
  if(magic == 3)
    ret = JS_NewBool(ctx, s->grab());

  if(magic == 4)
    ret = JS_NewBool(ctx, s->isOpened());
  if(magic == 5) {
    int32_t camID, apiPreference;
    if(!JS_ToInt32(ctx, &camID, argv[0])) {
      if(JS_ToInt32(ctx, &apiPreference, argv[1]))
        apiPreference = cv::CAP_ANY;

      ret = JS_NewBool(ctx, s->open(camID, apiPreference));
    } else
      ret = JS_EXCEPTION;
  }

  return ret;
}

VISIBLE JSValue
js_video_capture_wrap(JSContext* ctx, cv::VideoCapture* cap) {
  JSValue ret;

  ret = JS_NewObjectProtoClass(ctx, video_capture_proto, js_video_capture_class_id);

  // cap->addref();

  JS_SetOpaque(ret, cap);

  return ret;
}

JSClassDef js_video_capture_class = {
    .class_name = "VideoCapture",
    .finalizer = js_video_capture_finalizer,
};

const JSCFunctionListEntry js_video_capture_proto_funcs[] = {
    /*JS_CGETSET_ENUMERABLE_DEF("x", js_video_capture_get_xywh, js_video_capture_set_xywh, 0),
    JS_CGETSET_ENUMERABLE_DEF("y", js_video_capture_get_xywh, js_video_capture_set_xywh, 1),
    JS_CGETSET_ENUMERABLE_DEF("width", js_video_capture_get_xywh, js_video_capture_set_xywh, 2),
    JS_CGETSET_ENUMERABLE_DEF("height", js_video_capture_get_xywh, js_video_capture_set_xywh, 3),
    JS_CGETSET_MAGIC_DEF("x2", js_video_capture_get_xywh, js_video_capture_set_xywh, 4),
    JS_CGETSET_MAGIC_DEF("y2", js_video_capture_get_xywh, js_video_capture_set_xywh, 5),
*/
    JS_CFUNC_MAGIC_DEF("get", 1, js_video_capture_method, 0),
    JS_CFUNC_MAGIC_DEF("set", 2, js_video_capture_method, 1),
    JS_CFUNC_MAGIC_DEF("getBackendName", 0, js_video_capture_method, 2),
    JS_CFUNC_MAGIC_DEF("grab", 0, js_video_capture_method, 3),
    JS_CFUNC_MAGIC_DEF("isOpened", 0, js_video_capture_method, 4),
    JS_CFUNC_MAGIC_DEF("open", 0, js_video_capture_method, 5),
    JS_CFUNC_MAGIC_DEF("read", 0, js_video_capture_method, 6),
    JS_CFUNC_MAGIC_DEF("retrieve", 0, js_video_capture_method, 7),
    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "VideoCapture", JS_PROP_CONFIGURABLE),

};

int
js_video_capture_init(JSContext* ctx, JSModuleDef* m) {

  /* create the VideoCapture class */
  JS_NewClassID(&js_video_capture_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_video_capture_class_id, &js_video_capture_class);

  video_capture_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx,
                             video_capture_proto,
                             js_video_capture_proto_funcs,
                             countof(js_video_capture_proto_funcs));
  JS_SetClassProto(ctx, js_video_capture_class_id, video_capture_proto);

  video_capture_class =
      JS_NewCFunction2(ctx, js_video_capture_ctor, "VideoCapture", 2, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, video_capture_class, video_capture_proto);

  if(m)
    JS_SetModuleExport(ctx, m, "VideoCapture", video_capture_class);

  return 0;
}

void
js_video_capture_constructor(JSContext* ctx, JSValue parent, const char* name) {
  if(JS_IsUndefined(video_capture_class))
    js_video_capture_init(ctx, 0);

  JS_SetPropertyStr(ctx, parent, name ? name : "VideoCapture", video_capture_class);
}

#ifdef JS_VIDEO_CAPTURE_MODULE
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module
#else
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module_video_capture
#endif

extern "C" JSModuleDef*
JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_video_capture_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "VideoCapture");
  return m;
}
