#include "./jsbindings.h"
#include "js.h"
#include "quickjs/cutils.h"
#include "quickjs/quickjs.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#if defined(JS_POINT_ITERATOR_MODULE) || defined(quickjs_point_iterator_EXPORTS)
#define JS_INIT_MODULE js_init_module
#else
#define JS_INIT_MODULE js_init_module_point_iterator
#endif

extern "C" {

JSValue point_iterator_proto, point_iterator_class;
JSClassID js_point_iterator_class_id;

static JSValue
js_point_iterator_next(
    JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, BOOL* pdone, int magic) {
  JSPointIteratorData* it =
      static_cast<JSPointIteratorData*>(JS_GetOpaque(this_val, js_point_iterator_class_id));
  JSPointData* ptr;
  JSValue point;

  point = JS_NewObjectClass(ctx, js_point_iterator_class_id);
  if(JS_IsException(point))
    goto fail;
  ptr = static_cast<JSPointData*>(js_malloc(ctx, sizeof(*ptr)));
  if(!ptr)
    goto fail1;
  ptr = it->begin;

  *pdone = (it->begin == it->end);
  it->begin++;
  JS_SetOpaque(point, ptr);
  return point;
fail1:
  JS_FreeValue(ctx, point);
fail:
  return JS_EXCEPTION;
}

static JSValue
js_point_iterator_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSPointIteratorData* s;
  JSContourData* v;
  JSValue obj = JS_UNDEFINED;
  JSValue proto;

  s = static_cast<JSPointIteratorData*>(js_mallocz(ctx, sizeof(*s)));
  if(!s)
    return JS_EXCEPTION;

  v = static_cast<JSContourData*>(JS_GetOpaque(argv[0], js_contour_class_id));

  s->begin = &(*v)[0];
  s->end = s->begin + v->size();

  /*  if(JS_ToFloat64(ctx, &s->x, argv[0]))
      goto fail;
    if(JS_ToFloat64(ctx, &s->y, argv[1]))
      goto fail;*/
  /* using new_target to get the prototype is necessary when the
     class is extended. */
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

static void
js_point_iterator_finalizer(JSRuntime* rt, JSValue val) {
  JSPointIteratorData* s =
      static_cast<JSPointIteratorData*>(JS_GetOpaque(val, js_point_iterator_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */
  js_free_rt(rt, s);
}

JSClassDef js_point_iterator_class = {
    "PointIterator",
    .finalizer = js_point_iterator_finalizer,
};

const JSCFunctionListEntry js_point_iterator_proto_funcs[] = {
    JS_ITERATOR_NEXT_DEF("next", 0, js_point_iterator_next, 0),
    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "PointIterator", JS_PROP_CONFIGURABLE),
};

int
js_point_iterator_init(JSContext* ctx, JSModuleDef* m) {

  /* create the PointIterator class */
  JS_NewClassID(&js_point_iterator_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_point_iterator_class_id, &js_point_iterator_class);

  point_iterator_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx,
                             point_iterator_proto,
                             js_point_iterator_proto_funcs,
                             countof(js_point_iterator_proto_funcs));
  JS_SetClassProto(ctx, js_point_iterator_class_id, point_iterator_proto);

  point_iterator_class =
      JS_NewCFunction2(ctx, js_point_iterator_ctor, "PointIterator", 2, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, point_iterator_class, point_iterator_proto);

  if(m)
    JS_SetModuleExport(ctx, m, "PointIterator", point_iterator_class);
  /* else
     JS_SetPropertyStr(ctx, *static_cast<JSValue*>(m), name, point_iterator_class);*/
  return 0;
}

JSModuleDef* __attribute__((visibility("default")))
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
  JSPointIteratorData* s =
      static_cast<JSPointIteratorData*>(JS_GetOpaque2(ctx, this_val, js_point_iterator_class_id));
  std::ostringstream os;
  if(!s)
    return JS_EXCEPTION;

  // os << "{x:" << s->x << ",y:" << s->y << "}" << std::endl;

  return JS_NewString(ctx, os.str().c_str());
}

JSValue
js_create_point_iterator(
    JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSContourData* s = js_contour_data(ctx, this_val);
  JSPointIteratorData* it;
  JSValue iterator;
  int class_id;

  iterator = JS_NewObjectClass(ctx, js_point_iterator_class_id);
  if(JS_IsException(iterator))
    goto fail;
  it = static_cast<JSPointIteratorData*>(js_malloc(ctx, sizeof(*it)));
  if(!it)
    goto fail1;
  it->begin = &(*s)[0];
  it->end = it->begin + s->size();
  JS_SetOpaque(iterator, it);
  return iterator;
fail1:
  JS_FreeValue(ctx, iterator);
fail:
  return JS_EXCEPTION;
}
}
