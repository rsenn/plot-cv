#include "./jsbindings.h"
#include "js.h"
#include "quickjs/cutils.h"
#include "quickjs/quickjs.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#if defined(JS_POINT_ITERATOR_MODULE) || defined(quickjs_point_iterator_EXPORTS)
#define JS_INIT_MODULE VISIBLE js_init_module
#else
#define JS_INIT_MODULE VISIBLE js_init_module_point_iterator
#endif

extern "C" {

JSValue point_iterator_proto, point_iterator_class;
VISIBLE JSClassID js_point_iterator_class_id;

VISIBLE JSValue
js_point_iterator_new(JSContext* ctx, const std::pair<JSPointData*, JSPointData*>& range, int magic) {
  JSPointIteratorData* it;
  JSValue iterator;
  int class_id;

  iterator = JS_NewObjectClass(ctx, js_point_iterator_class_id);
  if(JS_IsException(iterator))
    goto fail;
  it = static_cast<JSPointIteratorData*>(js_malloc(ctx, sizeof(*it)));
  if(!it)
    goto fail1;
  it->magic = magic;
  it->first = range.first;
  it->second = range.second;

  JS_SetOpaque(iterator, it);
  return iterator;
fail1:
  JS_FreeValue(ctx, iterator);
fail:
  return JS_EXCEPTION;
}

JSValue
js_point_iterator_result(JSContext* ctx, JSValue val, BOOL done) {
  JSValue obj;
  obj = JS_NewObject(ctx);
  if(JS_IsException(obj)) {
    JS_FreeValue(ctx, val);
    return obj;
  }
  if(JS_DefinePropertyValue(ctx, obj, JS_ATOM_value, val, JS_PROP_C_W_E) < 0) {
    goto fail;
  }
  if(JS_DefinePropertyValue(ctx, obj, JS_ATOM_done, JS_NewBool(ctx, done), JS_PROP_C_W_E) < 0) {
  fail:
    JS_FreeValue(ctx, obj);
    return JS_EXCEPTION;
  }
  return obj;
}

static JSValue
js_point_iterator_next(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, BOOL* pdone, int magic) {
  JSPointIteratorData* it = static_cast<JSPointIteratorData*>(JS_GetOpaque(this_val, js_point_iterator_class_id));
  JSPointData* ptr;
  JSValue result;
  ptr = it->first;
  *pdone = (it->first == it->second);
  result = js_point_new(ctx, it->first->x, it->first->y);
  it->first++;
  return result;
fail:

  return JS_EXCEPTION;
}

static JSValue
js_point_iterator_create(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {

  return JS_EXCEPTION;
}

static void
js_point_iterator_finalizer(JSRuntime* rt, JSValue val) {
  JSPointIteratorData* s = static_cast<JSPointIteratorData*>(JS_GetOpaque(val, js_point_iterator_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */

  js_free_rt(rt, s);
}

static JSValue
js_point_iterator_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSPointIteratorData* s;
  JSContourData* v;
  JSValue obj = JS_UNDEFINED;
  JSValue proto;
  assert(0);

  s = static_cast<JSPointIteratorData*>(js_mallocz(ctx, sizeof(*s)));
  if(!s)
    return JS_EXCEPTION;

  v = static_cast<JSContourData*>(JS_GetOpaque(argv[0], 0 /*js_contour_class_id*/));

  s->first = &(*v)[0];
  s->second = s->first + v->size();

  proto = JS_GetPropertyStr(ctx, new_target, "prototype");
  if(JS_IsException(proto))
    goto fail;
  obj = JS_NewObjectProtoClass(ctx, proto, js_point_iterator_class_id);
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

JSClassDef js_point_iterator_class = {
    "PointIterator",
    .finalizer = js_point_iterator_finalizer,
};

const JSCFunctionListEntry js_point_iterator_proto_funcs[] = {
    JS_ITERATOR_NEXT_DEF("next", 0, js_point_iterator_next, 0),
    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "PointIterator", JS_PROP_CONFIGURABLE),
};
// const JSCFunctionListEntry js_point_iterator_static_funcs[] = {JS_CFUNC_MAGIC_DEF("create", 0, js_point_iterator_create, 0)};

int
js_point_iterator_init(JSContext* ctx, JSModuleDef* m) {

  /* create the PointIterator class */
  JS_NewClassID(&js_point_iterator_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_point_iterator_class_id, &js_point_iterator_class);

  point_iterator_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx, point_iterator_proto, js_point_iterator_proto_funcs, countof(js_point_iterator_proto_funcs));
  JS_SetClassProto(ctx, js_point_iterator_class_id, point_iterator_proto);

  point_iterator_class = JS_NewCFunction2(ctx, js_point_iterator_ctor, "PointIterator", 2, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */

  JS_SetConstructor(ctx, point_iterator_class, point_iterator_proto);
  // JS_SetPropertyFunctionList(ctx, point_iterator_class, js_point_iterator_static_funcs, countof(js_point_iterator_static_funcs));

  if(m)
    JS_SetModuleExport(ctx, m, "PointIterator", point_iterator_class);
  /* else
     JS_SetPropertyStr(ctx, *static_cast<JSValue*>(m), name, point_iterator_class);*/
  return 0;
}

JSModuleDef*
JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_point_iterator_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "PointIterator");
  return m;
}

void
js_point_iterator_constructor(JSContext* ctx, JSValue parent, const char* name) {
  if(JS_IsUndefined(point_iterator_class))
    js_point_iterator_init(ctx, 0);

  JS_SetPropertyStr(ctx, parent, name ? name : "PointIterator", point_iterator_class);
}

static JSValue
js_point_iterator_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointIteratorData* s = static_cast<JSPointIteratorData*>(JS_GetOpaque2(ctx, this_val, js_point_iterator_class_id));
  std::ostringstream os;
  if(!s)
    return JS_EXCEPTION;

  // os << "{x:" << s->x << ",y:" << s->y << "}" << std::endl;

  return JS_NewString(ctx, os.str().c_str());
}
}
