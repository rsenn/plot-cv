#ifndef JS_CONTOUR_H
#define JS_CONTOUR_H

#include "geometry.h"

template<class T>
static inline JSValue
js_contour_new(JSContext* ctx, const JSContourData<T>& points) {
  JSValue ret;
  JSContourData<T>* contour;
  ret = JS_NewObjectProtoClass(ctx, contour_proto, js_contour_class_id);
  contour = static_cast<JSContourData<T>*>(js_mallocz(ctx, sizeof(JSContourData<T>)));
  new(contour) JSContourData<T>();
  contour->resize(points.size());
  transform_points(points.cbegin(), points.cend(), contour->begin());
  
  JS_SetOpaque(ret, contour);
  return ret;
};

static inline int
js_contour_read(JSContext* ctx, JSValueConst contour, JSContourData<double>* out) {
  int ret = 0;
  return ret;
}

static inline JSContourData<double>
js_contour_get(JSContext* ctx, JSValueConst contour) {
  JSContourData<double> r = {};
  js_contour_read(ctx, contour, &r);
  return r;
}

template<class T>
JSValue
js_contours_new(JSContext* ctx, const std::vector<JSContourData<T>>& contours) {

  JSValue ret = JS_NewArray(ctx);
  uint32_t i, size = contours.size();

  for(i = 0; i < size; i++) {
    JSValue contour = js_contour_new(ctx, contours[i]);
    JS_SetPropertyUint32(ctx, ret, i, contour);
  }

  return ret;
}

#endif /* defined(JS_CONTOUR_H) */
