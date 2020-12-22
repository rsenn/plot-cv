#ifndef JS_CONTOUR_H
#define JS_CONTOUR_H

static inline int
js_contour_read(JSContext* ctx, JSValueConst contour, JSContourData* out) {
  int ret = 0;
  return ret;
}

static JSContourData
js_contour_get(JSContext* ctx, JSValueConst contour) {
  JSContourData r = {};
  js_contour_read(ctx, contour, &r);
  return r;
}

#endif /* defined(JS_CONTOUR_H) */
