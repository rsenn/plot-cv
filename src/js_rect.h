#ifndef JS_LINE_H
#define JS_LINE_H

static inline int
js_line_read(JSContext* ctx, JSValueConst line, JSLineData<double>* out) {
  int ret = 1;
  JSValue x = JS_UNDEFINED, y = JS_UNDEFINED, w = JS_UNDEFINED, h = JS_UNDEFINED;
  if(JS_IsArray(ctx, line)) {
    x = JS_GetPropertyUint32(ctx, line, 0);
    y = JS_GetPropertyUint32(ctx, line, 1);
    w = JS_GetPropertyUint32(ctx, line, 2);
    h = JS_GetPropertyUint32(ctx, line, 3);

  } else {
    x = JS_GetPropertyStr(ctx, line, "x");
    y = JS_GetPropertyStr(ctx, line, "y");
    w = JS_GetPropertyStr(ctx, line, "width");
    h = JS_GetPropertyStr(ctx, line, "height");
  }
  if(JS_IsNumber(x) && JS_IsNumber(y) && JS_IsNumber(w) && JS_IsNumber(h)) {
    ret &= !JS_ToFloat64(ctx, &out->x, x);
    ret &= !JS_ToFloat64(ctx, &out->y, y);
    ret &= !JS_ToFloat64(ctx, &out->width, w);
    ret &= !JS_ToFloat64(ctx, &out->height, h);
  } else {
    ret = 0;
  }
  if(!JS_IsUndefined(x))
    JS_FreeValue(ctx, x);
  if(!JS_IsUndefined(y))
    JS_FreeValue(ctx, y);
  if(!JS_IsUndefined(w))
    JS_FreeValue(ctx, w);
  if(!JS_IsUndefined(h))
    JS_FreeValue(ctx, h);
  return ret;
}

static JSLineData<double>
js_line_get(JSContext* ctx, JSValueConst line) {
  JSLineData<double> r = {0, 0, 0, 0};
  js_line_read(ctx, line, &r);
  return r;
}

static inline int
js_line_write(JSContext* ctx, JSValue out, JSLineData<double> line) {
  int ret = 0;
  ret += JS_SetPropertyStr(ctx, out, "x", JS_NewFloat64(ctx, line.x));
  ret += JS_SetPropertyStr(ctx, out, "y", JS_NewFloat64(ctx, line.y));
  ret += JS_SetPropertyStr(ctx, out, "width", JS_NewFloat64(ctx, line.width));
  ret += JS_SetPropertyStr(ctx, out, "height", JS_NewFloat64(ctx, line.height));
  return ret;
}

static JSLineData<double>
js_line_set(JSContext* ctx, JSValue out, double x, double y, double w, double h) {
  const JSLineData<double> r = {x, y, w, h};
  js_line_write(ctx, out, r);
  return r;
}

#endif /* defined(JS_LINE_H) */
