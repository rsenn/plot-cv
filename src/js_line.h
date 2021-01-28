#ifndef JS_LINE_H
#define JS_LINE_H

#include "jsbindings.h"

template<class T>
static inline int
js_line_read(JSContext* ctx, JSValueConst line, T* out) {
  int ret = 1;
  JSValue x1 = JS_UNDEFINED, y1 = JS_UNDEFINED, x2 = JS_UNDEFINED, y2 = JS_UNDEFINED;
  if(JS_IsArray(ctx, line)) {
    x1 = JS_GetPropertyUint32(ctx, line, 0);
    y1 = JS_GetPropertyUint32(ctx, line, 1);
    x2 = JS_GetPropertyUint32(ctx, line, 2);
    y2 = JS_GetPropertyUint32(ctx, line, 3);

  } else {
    x1 = JS_GetPropertyStr(ctx, line, "x1");
    y1 = JS_GetPropertyStr(ctx, line, "y1");
    x2 = JS_GetPropertyStr(ctx, line, "x2");
    y2 = JS_GetPropertyStr(ctx, line, "y2");
  }
  if(JS_IsNumber(x1) && JS_IsNumber(y1) && JS_IsNumber(x2) && JS_IsNumber(y2)) {
    ret &= !JS_ToFloat64(ctx, &out->arr[0], x1);
    ret &= !JS_ToFloat64(ctx, &out->arr[1], y1);
    ret &= !JS_ToFloat64(ctx, &out->arr[2], x2);
    ret &= !JS_ToFloat64(ctx, &out->arr[3], y2);
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

#endif /* defined(JS_LINE_H) */
