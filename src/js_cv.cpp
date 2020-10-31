#include "./jsbindings.h"
#include "./geometry.h"
#include "../quickjs/cutils.h"

#if defined(JS_CV_MODULE) || defined(quickjs_cv_EXPORTS)
#define JS_INIT_MODULE VISIBLE js_init_module
#else
#define JS_INIT_MODULE VISIBLE js_init_module_cv
#endif 
 
static JSValue
js_cv_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSValue obj = JS_UNDEFINED;
  JSValue proto;
  JSSizeData size;
 
  return obj;
}


static JSValue
js_cv_hough_lines(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData s;

  double angle = 0, scale = 1;
  cv::Mat m;

  JSValue ret;
  if(argc == 0)
    return JS_EXCEPTION;
  if(argc > 0) {
    s = js_point_get(ctx, argv[0]);
    if(argc > 1) {
      JS_ToFloat64(ctx, &angle, argv[1]);
      if(argc > 2) {
        JS_ToFloat64(ctx, &scale, argv[2]);
      }
    }
  }

  m = cv::getRotationMatrix2D(s, angle, scale);

  ret = js_mat_wrap(ctx, m);
  return ret;
}


JSValue cv_proto, cv_class;
JSClassID js_cv_class_id;

JSClassDef js_cv_class = {
    "cv"
};
  
const JSCFunctionListEntry js_cv_static_funcs[] = {
    JS_CFUNC_DEF("HoughLines", 3, js_cv_hough_lines),    
    {0}};

int
js_cv_init(JSContext* ctx, JSModuleDef* m) {
  /* create the cv class */
  JS_NewClassID(&js_cv_class_id);
   JS_NewClass(JS_GetRuntime(ctx), js_cv_class_id, &js_cv_class);
 
/*  cv_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx, cv_proto, js_cv_proto_funcs, countof(js_cv_proto_funcs));
  JS_SetClassProto(ctx, js_cv_class_id, cv_proto);
*/
  
  cv_class = JS_NewCFunction2(ctx, js_cv_ctor, "cv", 2, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, cv_class, cv_proto);

  JS_SetPropertyFunctionList(ctx, cv_class, js_cv_static_funcs, countof(js_cv_static_funcs));

  JSValue g = JS_GetGlobalObject(ctx);
  int32array_ctor = JS_GetProperty(ctx, g, JS_ATOM_Int32Array);
  int32array_proto = JS_GetPrototype(ctx, int32array_ctor);

  if(m)
    JS_SetModuleExport(ctx, m, "cv", cv_class);
  /*else
    JS_SetPropertyStr(ctx, *static_cast<JSValue*>(m), "cv", cv_class);*/
  return 0;
}

extern "C" JSModuleDef*
JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_cv_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "cv");
  return m;
}
