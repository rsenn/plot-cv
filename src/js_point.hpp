#ifndef JS_POINT_HPP
#define JS_POINT_HPP

/*static inline JSValue
js_point_create(JSContext* ctx, double x, double y) {

  JSValue point = js_new(ctx, "Point");

  JS_SetPropertyStr(ctx, point, "x", JS_NewFloat64(ctx, x));
  JS_SetPropertyStr(ctx, point, "y", JS_NewFloat64(ctx, y));
  return point;
}*/

extern "C" JSValue js_point_clone(JSContext* ctx, const JSPointData<double>& point);

template<class T>
static inline int
js_point_read(JSContext* ctx, JSValueConst point, JSPointData<T>* out) {
  int ret = 1;
  JSValue x = JS_UNDEFINED, y = JS_UNDEFINED;
  if(JS_IsArray(ctx, point)) {
    x = JS_GetPropertyUint32(ctx, point, 0);
    y = JS_GetPropertyUint32(ctx, point, 1);
  } else if(JS_IsObject(point)) {
    x = JS_GetPropertyStr(ctx, point, "x");
    y = JS_GetPropertyStr(ctx, point, "y");
  }
  if(JS_IsNumber(x) && JS_IsNumber(y)) {
    ret &= js_number_read(ctx, &out->x, x);
    ret &= js_number_read(ctx, &out->y, y);
  } else {
    ret = 0;
  }
  if(!JS_IsUndefined(x))
    JS_FreeValue(ctx, x);
  if(!JS_IsUndefined(y))
    JS_FreeValue(ctx, y);
  return ret;
}

static inline JSPointData<double>
js_point_get(JSContext* ctx, JSValueConst point) {
  JSPointData<double> r;
  js_point_read(ctx, point, &r);
  return r;
}

static inline bool
js_is_point(JSContext* ctx, JSValueConst point) {
  JSPointData<double> r;

  if(js_point_data(ctx, point))
    return true;

  if(js_point_read(ctx, point, &r))
    return true;

  return false;
}

extern "C" int js_point_init(JSContext*, JSModuleDef*);

#endif /* defined(JS_POINT_H) */