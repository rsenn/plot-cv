#include <quickjs.h>

// Example function to be exposed to JavaScript
static JSValue
js_add(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  int a, b;
  if(JS_ToInt32(ctx, &a, argv[0]) || JS_ToInt32(ctx, &b, argv[1]))
    return JS_EXCEPTION;
  return JS_NewInt32(ctx, a + b);
}

// Module initialization function
static int
js_mymodule_init(JSContext* ctx, JSModuleDef* m) {
  // Expose the `add` function to JavaScript
  JSValue obj = JS_NewObject(ctx);
  JS_SetPropertyStr(ctx, obj, "add", JS_NewCFunction(ctx, js_add, "add", 2));
  JS_SetModuleExport(ctx, m, "default", obj);
  return 0;
}

// Module definition
JSModuleDef*
js_init_module(JSContext* ctx, const char* module_name) {
  JSModuleDef* m = JS_NewCModule(ctx, module_name, js_mymodule_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "default");
  return m;
}
