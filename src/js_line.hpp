#ifndef JS_LINE_HPP
#define JS_LINE_HPP

#include "jsbindings.hpp"

template<class T>
static inline int
js_line_read(JSContext* ctx, JSValueConst line, T* out) {
  int ret = 1;
  JSValue x1 = JS_UNDEFINED, y1 = JS_UNDEFINED, x2 = JS_UNDEFINED, y2 = JS_UNDEFINED;
  if(JS_IsArray(ctx, line)) {
    js_array_to(ctx, line, out->array);
    return 1;
  } else if(js_is_iterable(ctx, line)) {
    if(js_iterable_to(ctx, line, out->array) == 4)
      return 1;
  }
  x1 = JS_GetPropertyStr(ctx, line, "x1");
  y1 = JS_GetPropertyStr(ctx, line, "y1");
  x2 = JS_GetPropertyStr(ctx, line, "x2");
  y2 = JS_GetPropertyStr(ctx, line, "y2");

  if(JS_IsNumber(x1) && JS_IsNumber(y1) && JS_IsNumber(x2) && JS_IsNumber(y2)) {
    ret &= js_number_read(ctx, x1, &out->array[0]);
    ret &= js_number_read(ctx, y1, &out->array[1]);
    ret &= js_number_read(ctx, x2, &out->array[2]);
    ret &= js_number_read(ctx, y2, &out->array[3]);
  } else {
    ret = 0;
  }
  if(!JS_IsUndefined(x1))
    JS_FreeValue(ctx, x1);
  if(!JS_IsUndefined(y1))
    JS_FreeValue(ctx, y1);
  if(!JS_IsUndefined(x2))
    JS_FreeValue(ctx, x2);
  if(!JS_IsUndefined(y2))
    JS_FreeValue(ctx, y2);
  return ret;
}

template<class T>
static JSLineData<T>
js_line_get(JSContext* ctx, JSValueConst line) {
  std::array<T, 4> r;
  js_line_read(ctx, line, r);
  return *reinterpret_cast<JSLineData<T>*>(r);
}

static inline int
js_line_write(JSContext* ctx, JSValue out, JSLineTraits<double>::array_type line) {
  int ret = 0;
  ret += JS_SetPropertyStr(ctx, out, "x1", JS_NewFloat64(ctx, line[0]));
  ret += JS_SetPropertyStr(ctx, out, "y1", JS_NewFloat64(ctx, line[1]));
  ret += JS_SetPropertyStr(ctx, out, "x2", JS_NewFloat64(ctx, line[2]));
  ret += JS_SetPropertyStr(ctx, out, "y2", JS_NewFloat64(ctx, line[3]));
  return ret;
}

template<class T>
static JSLineData<T>
js_line_set(JSContext* ctx, JSValue out, T x1, T y1, T x2, T y2) {
  std::array<T, 4> r{x1, y1, x2, y2};
  js_line_write(ctx, out, r);
  return r;
}

extern "C" int js_line_init(JSContext*, JSModuleDef*);

#endif /* defined(JS_LINE_HPP) */
