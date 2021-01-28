#ifndef JS_SIZE_HPP
#define JS_SIZE_HPP

template<class T>
static inline int
js_size_read(JSContext* ctx, JSValueConst size, JSSizeData<T>* out) {
  int ret = 1;
  JSValue w = JS_UNDEFINED, h = JS_UNDEFINED;

  if(JS_IsArray(ctx, size)) {
    w = JS_GetPropertyUint32(ctx, size, 0);
    h = JS_GetPropertyUint32(ctx, size, 1);
  } else if(JS_IsObject(size)) {
    w = JS_GetPropertyStr(ctx, size, "width");
    h = JS_GetPropertyStr(ctx, size, "height");
  }
  if(JS_IsNumber(w) && JS_IsNumber(h)) {
    ret &= js_number_read(ctx, w, &out->width);
    ret &= js_number_read(ctx, w, &out->height);
  } else {
    ret = 0;
  }
  if(!JS_IsUndefined(w))
    JS_FreeValue(ctx, w);
  if(!JS_IsUndefined(h))
    JS_FreeValue(ctx, h);
  return ret;
}

static inline JSSizeData<double>
js_size_get(JSContext* ctx, JSValueConst size) {
  JSSizeData<double> r = {0, 0};
  js_size_read(ctx, size, &r);
  return r;
}

#endif /* defined(JS_SIZE_H) */
