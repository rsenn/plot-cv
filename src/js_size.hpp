#ifndef JS_SIZE_HPP
#define JS_SIZE_HPP

template<class T>
static inline int
js_size_read(JSContext* ctx, JSValueConst size, JSSizeData<T>* out) {
  int ret = 1;
  JSValue width = JS_UNDEFINED, height = JS_UNDEFINED;

  if(JS_IsArray(ctx, size)) {
    width = JS_GetPropertyUint32(ctx, size, 0);
    height = JS_GetPropertyUint32(ctx, size, 1);
  } else if(JS_IsObject(size)) {
    width = JS_GetPropertyStr(ctx, size, "width");
    height = JS_GetPropertyStr(ctx, size, "height");
  }
  if(JS_IsNumber(width) && JS_IsNumber(height)) {
    ret &= js_number_read(ctx, width, &out->width);
    ret &= js_number_read(ctx, height, &out->height);
  } else {
    ret = 0;
  }
  if(!JS_IsUndefined(width))
    JS_FreeValue(ctx, width);
  if(!JS_IsUndefined(height))
    JS_FreeValue(ctx, height);
  return ret;
}

static inline JSSizeData<double>
js_size_get(JSContext* ctx, JSValueConst size) {
  JSSizeData<double> r = {0, 0};
  js_size_read(ctx, size, &r);
  return r;
}

#endif /* defined(JS_SIZE_H) */
