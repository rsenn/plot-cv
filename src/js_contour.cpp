#include "./jsbindings.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "plot-cv.h"
#include "color.h"
#include "geometry.h"
#include "psimpl.h"

#include <iomanip>

#if defined(JS_CONTOUR_MODULE) || defined(quickjs_contour_EXPORTS)
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module
#else
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module_contour
#endif

// using namespace cv;

static JSValue
js_contour_new(JSContext* ctx, const std::vector<cv::Point_<float>>& points) {
  JSValue ret;
  JSContourData* contour;

  ret = JS_NewObjectProtoClass(ctx, contour_proto, js_contour_class_id);

  contour = static_cast<JSContourData*>(js_mallocz(ctx, sizeof(JSContourData)));

  new(contour) JSContourData();

  contour->resize(points.size());

  transform_points(points.cbegin(), points.cend(), contour->begin());
  /*
    std::transform(points.cbegin(),
                   points.cend(),
                   std::back_inserter(*contour),
                   [](const cv::Point2f& pt) -> cv::Point2d { return cv::Point2d(pt.x, pt.y); });
  */
  JS_SetOpaque(ret, contour);
  return ret;
};

VISIBLE JSValue
js_contour2i_new(JSContext* ctx, const std::vector<cv::Point_<int>>& points) {
  JSValue ret;
  JSContourData* contour;

  ret = JS_NewObjectProtoClass(ctx, contour_proto, js_contour_class_id);

  contour = static_cast<JSContourData*>(js_mallocz(ctx, sizeof(JSContourData)));

  std::transform(points.cbegin(), points.cend(), std::back_inserter(*contour), [](const cv::Point& pt) -> cv::Point2d {
    return cv::Point2d(pt.x, pt.y);
  });

  JS_SetOpaque(ret, contour);
  return ret;
};

VISIBLE JSValue
js_contour2d_new(JSContext* ctx, const std::vector<cv::Point_<double>>& points) {
  JSValue ret;
  JSContourData* contour;

  ret = JS_NewObjectProtoClass(ctx, contour_proto, js_contour_class_id);

  contour = static_cast<JSContourData*>(js_mallocz(ctx, sizeof(JSContourData)));

  std::copy(points.cbegin(), points.cend(), std::back_inserter(*contour));

  JS_SetOpaque(ret, contour);
  return ret;
}

template<class Value> JSValue js_vector_to_array(JSContext* ctx, const std::vector<Value>& vec);

template<>
JSValue
js_vector_to_array(JSContext* ctx, const std::vector<int>& vec) {
  JSValue ret = JS_NewArray(ctx);
  uint32_t i, n = vec.size();
  for(i = 0; i < n; i++) {
    JS_SetPropertyUint32(ctx, ret, i, JS_NewInt32(ctx, vec[i]));
  }
  return ret;
}

template<class Vector>
static JSValue
js_vector_to_array(std::enable_if_t<std::is_same<Vector, cv::Vec4i>::value, JSContext*> ctx,
                   const std::vector<Vector>& vec) {
  JSValue ret = JS_NewArray(ctx);
  uint32_t i, j, n = vec.size();
  for(i = 0; i < n; i++) {
    JSValue item = JS_NewArray(ctx);
    for(j = 0; j < 4; j++) {
      JS_SetPropertyUint32(ctx, item, j, JS_NewInt32(ctx, vec[i][j]));
    }
    JS_SetPropertyUint32(ctx, ret, i, item);
  }
  return ret;
}

static JSValue
js_vector_to_array(JSContext* ctx, const std::vector<cv::Point_<float>>& vec) {
  JSValue ret = JS_NewArray(ctx);
  uint32_t i, n = vec.size();
  for(i = 0; i < n; i++) {
    JSValue item = js_point_new(ctx, vec[i].x, vec[i].y);

    JS_SetPropertyUint32(ctx, ret, i, item);
  }
  return ret;
}

static JSValue
js_vector_to_array(JSContext* ctx, const std::vector<std::vector<cv::Point2d>>& contours) {
  JSValue ret = JS_NewArray(ctx);
  uint32_t i, size = contours.size();

  for(i = 0; i < size; i++) {
    JS_SetPropertyUint32(ctx, ret, i, js_contour2d_new(ctx, contours[i]));
  }
  return ret;
}

extern "C" {

static JSValue
js_contour_approxpolydp(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSValue ret = JS_UNDEFINED;
  double epsilon;
  bool closed = false;
  std::vector<cv::Point> curve;
  std::vector<cv::Point2f> approxCurve;
  JSContourData *out, *v;

  v = js_contour_data(ctx, this_val);

  if(!v)
    return JS_EXCEPTION;
  out = static_cast<JSContourData*>(JS_GetOpaque2(ctx, argv[0], js_contour_class_id));

  if(argc > 1) {
    JS_ToFloat64(ctx, &epsilon, argv[1]);

    if(argc > 2) {
      closed = !!JS_ToBool(ctx, argv[2]);
    }
  }

  std::transform(v->begin(), v->end(), std::back_inserter(curve), [](const cv::Point2d& pt) -> cv::Point {
    return cv::Point(pt.x, pt.y);
  });

  cv::approxPolyDP(curve, approxCurve, epsilon, closed);

  std::transform(approxCurve.begin(),
                 approxCurve.end(),
                 std::back_inserter(*out),
                 [](const cv::Point2f& pt) -> cv::Point2d { return cv::Point2d(pt.x, pt.y); });

  return JS_UNDEFINED;
}

static JSValue
js_contour_arclength(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  cv::Point2f pt;
  bool closed = false;
  std::vector<cv::Point2f> contour;
  JSPointData* point;
  double retval;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  if(argc > 0) {
    closed = !!JS_ToBool(ctx, argv[0]);
  }

  std::transform(v->begin(), v->end(), std::back_inserter(contour), [](const cv::Point2d& pt) -> cv::Point2f {
    return cv::Point2f(pt.x, pt.y);
  });

  retval = cv::arcLength(contour, closed);

  ret = JS_NewFloat64(ctx, retval);

  return ret;
}

static JSValue
js_contour_area(JSContext* ctx, JSValueConst this_val) {
  JSContourData* v;
  JSValue ret = JS_UNDEFINED;
  double area;
  std::vector<cv::Point2f> contour;
  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;
  std::transform(v->begin(), v->end(), std::back_inserter(contour), [](const cv::Point2d& pt) -> cv::Point2f {
    return cv::Point2f(pt.x, pt.y);
  });
  area = cv::contourArea(contour);
  ret = JS_NewFloat64(ctx, area);
  return ret;
}

static JSValue
js_contour_boundingrect(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSValue ret = JS_UNDEFINED;
  cv::Rect2f rect;
  std::vector<cv::Point2f> curve;
  JSContourData* v;
  JSRectData r;

  v = js_contour_data(ctx, this_val);

  if(!v)
    return JS_EXCEPTION;

  std::transform(v->begin(), v->end(), std::back_inserter(curve), [](const cv::Point2d& pt) -> cv::Point2f {
    return cv::Point2f(pt.x, pt.y);
  });

  rect = cv::boundingRect(curve);

  ret = js_new(ctx, "Rect");
  // ret = JS_NewObject(ctx, rect_proto, js_rect_class_id);
  // r = static_cast<JSRectData*>(js_mallocz(ctx, sizeof(JSRectData)));
  //*r = rect;
  // JS_SetOpaque(ret, r);
  //
  r = rect;

  js_rect_write(ctx, ret, r);

  return ret;
}

static JSValue
js_contour_center(JSContext* ctx, JSValueConst this_val) {
  JSContourData* v;
  JSValue ret = JS_UNDEFINED;
  double area;
  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;
  {
    std::vector<cv::Point> points;
    points.resize(v->size());
    std::copy(v->begin(), v->end(), points.begin());
    cv::Moments mu = cv::moments(points);
    cv::Point centroid = cv::Point(mu.m10 / mu.m00, mu.m01 / mu.m00);

    ret = js_point_new(ctx, centroid.x, centroid.y);
  }

  return ret;
}

static JSValue
js_contour_convexhull(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSValue ret = JS_UNDEFINED;
  bool clockwise = false, returnPoints = true;
  std::vector<cv::Point2f> curve, hull;
  std::vector<int> hullIndices;
  JSContourData *out, *v;

  v = js_contour_data(ctx, this_val);

  if(!v)
    return JS_EXCEPTION;

  if(argc > 0) {
    clockwise = !!JS_ToBool(ctx, argv[0]);

    if(argc > 1) {
      returnPoints = !!JS_ToBool(ctx, argv[1]);
    }
  }

  std::transform(v->begin(), v->end(), std::back_inserter(curve), [](const cv::Point2d& pt) -> cv::Point2f {
    return cv::Point2f(pt.x, pt.y);
  });

  if(returnPoints)
    cv::convexHull(curve, hull, clockwise, true);
  else
    cv::convexHull(curve, hullIndices, clockwise, false);

  if(returnPoints) {
    ret = js_contour_new(ctx, hull);
  } else {
    uint32_t i, size = hullIndices.size();

    ret = JS_NewArray(ctx);

    for(i = 0; i < size; i++) {
      JS_SetPropertyUint32(ctx, ret, i, JS_NewInt32(ctx, hullIndices[i]));
    }
  }

  return ret;
}

static JSValue
js_contour_convexitydefects(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* s = js_contour_data(ctx, this_val);
  JSValue ret = JS_UNDEFINED;

  std::vector<int> hullIndices;
  std::vector<cv::Vec4i> defects;

  if(argc > 0) {
    int64_t n = js_array_to_vector(ctx, argv[0], hullIndices);
    if(n == 0)
      return JS_EXCEPTION;
  }

  if(s->size() == 0 || hullIndices.size() == 0)
    return JS_EXCEPTION;

  defects.resize(hullIndices.size());
  cv::convexityDefects(*s, hullIndices, defects);

  ret = js_vector_vec4i_to_array(ctx, defects);

  return ret;
}

static JSValue
js_contour_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSContourData* v;
  JSValue obj = JS_UNDEFINED;
  JSValue proto;

  v = static_cast<JSContourData*>(js_mallocz(ctx, sizeof(JSContourData)));
  if(!v)
    return JS_EXCEPTION;

  new(v) JSContourData();

  /* using new_target to get the prototype is necessary when the
     class is extended. */
  proto = JS_GetPropertyStr(ctx, new_target, "prototype");
  if(JS_IsException(proto))
    goto fail;
  obj = JS_NewObjectProtoClass(ctx, proto, js_contour_class_id);
  JS_FreeValue(ctx, proto);
  if(JS_IsException(obj))
    goto fail;
  JS_SetOpaque(obj, v);

  if(argc > 0) {
    int i;
    // jsrt js(ctx);

    for(i = 0; i < argc; i++) {
      JSPointData p;
      if(JS_IsArray(ctx, argv[i])) {
        if(js_array_length(ctx, argv[i]) > 0) {
          JSValue pt = JS_GetPropertyUint32(ctx, argv[i], 0);

          if(js_is_point(ctx, pt)) {
            js_array_to_vector /*<JSPointData>*/ (ctx, argv[i], *v);
            JS_FreeValue(ctx, pt);
            continue;
          }
          JS_FreeValue(ctx, pt);
        }
      }

      if(js_point_read(ctx, argv[i], &p)) {
        v->push_back(p);
        continue;
      }
      goto fail;
    }
  }

  return obj;
fail:
  js_free(ctx, v);
  JS_FreeValue(ctx, obj);
  return JS_EXCEPTION;
}

VISIBLE JSContourData*
js_contour_data(JSContext* ctx, JSValueConst val) {
  return static_cast<JSContourData*>(JS_GetOpaque2(ctx, val, js_contour_class_id));
}

void
js_contour_finalizer(JSRuntime* rt, JSValue val) {
  JSContourData* s = static_cast<JSContourData*>(JS_GetOpaque(val, js_contour_class_id));

  if(s != nullptr)
    js_free_rt(rt, s);

  JS_FreeValueRT(rt, val);
}

static JSValue
js_contour_fitellipse(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *ptr;
  cv::RotatedRect rr;
  JSValue ret = JS_UNDEFINED;
  double area;
  std::vector<cv::Point2f> contour, ellipse;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  std::transform(v->begin(), v->end(), std::back_inserter(contour), [](const cv::Point2d& pt) -> cv::Point2f {
    return cv::Point2f(pt.x, pt.y);
  });

  rr = cv::fitEllipse(contour);

  ellipse.resize(5);
  rr.points(ellipse.data());

  ret = js_contour_new(ctx, ellipse);

  return ret;
}

static JSValue
js_contour_fitline(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *ptr;
  JSValue ret = JS_UNDEFINED;
  int64_t distType = cv::DIST_FAIR;
  double param = 0, reps = 0.01, aeps = 0.01;
  std::vector<cv::Point> contour;
  std::vector<cv::Point2f> points;
  cv::Vec4f line;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  if(argc > 0) {

    JS_ToInt64(ctx, &distType, argv[0]);

    if(argc > 1) {
      JS_ToFloat64(ctx, &param, argv[1]);
      if(argc > 2) {
        JS_ToFloat64(ctx, &reps, argv[2]);
        if(argc > 3) {
          JS_ToFloat64(ctx, &aeps, argv[3]);
        }
      }
    }
  }

  std::transform(v->begin(), v->end(), std::back_inserter(contour), [](const cv::Point2d& pt) -> cv::Point {
    return cv::Point(pt.x, pt.y);
  });

  cv::fitLine(contour, line, distType, param, reps, aeps);

  points.push_back(cv::Point2f(line[0], line[1]));
  points.push_back(cv::Point2f(line[2], line[3]));

  ret = js_contour_new(ctx, points);

  return ret;
}

static JSValue
js_contour_get(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* v;
  JSValue ret;
  JSValue x, y;
  int64_t i;
  JSPointData* point;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  JS_ToInt64(ctx, &i, argv[0]);

  if(i >= v->size() || i < 0)
    return JS_UNDEFINED;

  ret = js_point_new(ctx, (*v)[i].x, (*v)[i].y);
  return ret;
}

static JSValue
js_contour_intersectconvex(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  bool handleNested = true;
  std::vector<cv::Point2f> a, b, intersection;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  if(argc > 0) {
    other = static_cast<JSContourData*>(JS_GetOpaque2(ctx, argv[0], js_contour_class_id));

    if(argc > 1) {
      handleNested = !!JS_ToBool(ctx, argv[1]);
    }
  }

  std::transform(v->begin(), v->end(), std::back_inserter(a), [](const cv::Point2d& pt) -> cv::Point2f {
    return cv::Point2f(pt.x, pt.y);
  });

  std::transform(other->begin(), other->end(), std::back_inserter(b), [](const cv::Point2d& pt) -> cv::Point2f {
    return cv::Point2f(pt.x, pt.y);
  });

  cv::intersectConvexConvex(a, b, intersection, handleNested);

  ret = js_contour_new(ctx, intersection);
  return ret;
}

static JSValue
js_contour_isconvex(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  bool isConvex;
  std::vector<cv::Point2f> contour;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  std::transform(v->begin(), v->end(), std::back_inserter(contour), [](const cv::Point2d& pt) -> cv::Point2f {
    return cv::Point2f(pt.x, pt.y);
  });

  isConvex = cv::isContourConvex(contour);

  ret = JS_NewBool(ctx, isConvex);

  return ret;
}

static JSValue
js_contour_length(JSContext* ctx, JSValueConst this_val) {
  JSContourData* v;
  JSValue ret;
  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;
  ret = JS_NewInt64(ctx, v->size());
  return ret;
}

static JSValue
js_contour_minarearect(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  cv::RotatedRect rr;

  std::vector<cv::Point2f> contour, minarea;
  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;
  std::transform(v->begin(), v->end(), std::back_inserter(contour), [](const cv::Point2d& pt) -> cv::Point2f {
    return cv::Point2f(pt.x, pt.y);
  });
  rr = cv::minAreaRect(contour);
  minarea.resize(5);
  rr.points(minarea.data());

  ret = js_contour_new(ctx, minarea);
  return ret;
}

static JSValue
js_contour_minenclosingcircle(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  cv::RotatedRect rr;

  std::vector<cv::Point2f> contour;
  cv::Point2f center;
  float radius;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  std::transform(v->begin(), v->end(), std::back_inserter(contour), [](const cv::Point2d& pt) -> cv::Point2f {
    return cv::Point2f(pt.x, pt.y);
  });

  cv::minEnclosingCircle(contour, center, radius);

  ret = JS_NewObject(ctx);

  JS_SetPropertyStr(ctx, ret, "center", js_point_new(ctx, center.x, center.y));
  JS_SetPropertyStr(ctx, ret, "radius", JS_NewFloat64(ctx, radius));

  return ret;
}

static JSValue
js_contour_minenclosingtriangle(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  cv::RotatedRect rr;

  std::vector<cv::Point2f> contour, triangle;
  cv::Point2f center;
  float radius;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  std::transform(v->begin(), v->end(), std::back_inserter(contour), [](const cv::Point2d& pt) -> cv::Point2f {
    return cv::Point2f(pt.x, pt.y);
  });

  cv::minEnclosingTriangle(contour, triangle);

  ret = js_contour_new(ctx, triangle);

  return ret;
}

static JSValue
js_contour_pointpolygontest(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  cv::RotatedRect rr;
  cv::Point2f pt;
  bool measureDist = false;
  std::vector<cv::Point2f> contour, triangle;
  JSPointData point;
  double retval;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  if(argc > 0) {
    point = js_point_get(ctx, argv[0]);

    pt.x = point.x;
    pt.y = point.y;

    if(argc > 1) {
      measureDist = !!JS_ToBool(ctx, argv[1]);
    }
  }

  std::transform(v->begin(), v->end(), std::back_inserter(contour), [](const cv::Point2d& pt) -> cv::Point2f {
    return cv::Point2f(pt.x, pt.y);
  });

  retval = cv::pointPolygonTest(contour, pt, measureDist);

  ret = JS_NewFloat64(ctx, retval);

  return ret;
}

static JSValue
js_contour_psimpl(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSContourData* s = js_contour_data(ctx, this_val);
  int32_t shift = 1;
  uint32_t size = s->size();
  JSValue ret = JS_UNDEFINED;
  JSContourData r;
  double arg1 = 0, arg2 = 0;
  double* it;
  cv::Point2d* start = &(*s)[0];
  cv::Point2d* end = start + size;
  r.resize(size);
  it = (double*)&r[0];

  if(!s)
    return JS_EXCEPTION;

  if(argc > 0) {
    JS_ToFloat64(ctx, &arg1, argv[0]);
    if(argc > 1) {
      JS_ToFloat64(ctx, &arg2, argv[1]);
    }
  }

  if(magic == 0) {
    if(arg1 == 0)
      arg1 = 2;
    it = psimpl::simplify_reumann_witkam<2>((double*)start, (double*)end, arg1, it);
  } else if(magic == 1) {
    if(arg1 == 0)
      arg1 = 2;
    if(arg2 == 0)
      arg2 = 10;
    it = psimpl::simplify_opheim<2>((double*)start, (double*)end, arg1, arg2, it);
  } else if(magic == 2) {
    if(arg1 == 0)
      arg1 = 2;
    if(arg2 == 0)
      arg2 = 10;
    it = psimpl::simplify_lang<2>((double*)start, (double*)end, arg1, arg2, it);
  } else if(magic == 3) {
    if(arg1 == 0)
      arg1 = 2;
    it = psimpl::simplify_douglas_peucker<2>((double*)start, (double*)end, arg1, it);
  } else if(magic == 4) {
    if(arg1 == 0)
      arg1 = 2;
    it = psimpl::simplify_nth_point<2>((double*)start, (double*)end, arg1, it);
  } else if(magic == 5) {
    if(arg1 == 0)
      arg1 = 2;
    it = psimpl::simplify_radial_distance<2>((double*)start, (double*)end, arg1, it);
  } else if(magic == 6) {
    if(arg1 == 0)
      arg1 = 2;
    if(arg2 == 0)
      arg2 = 1;
    it = psimpl::simplify_perpendicular_distance<2>((double*)start, (double*)end, arg1, arg2, it);
  }
  size = it - (double*)&r[0];
  r.resize(size / 2);
  ret = js_contour2d_new(ctx, r);
  return ret;
}

static JSValue
js_contour_push(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* v;
  int i;
  double x, y;
  JSValueConst xv, yv;
  JSPointData point;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  for(i = 0; i < argc; i++) {
    if(JS_IsObject(argv[i])) {
      xv = JS_GetPropertyStr(ctx, argv[i], "x");
      yv = JS_GetPropertyStr(ctx, argv[i], "y");
    } else if(JS_IsArray(ctx, argv[i])) {

      xv = JS_GetPropertyUint32(ctx, argv[i], 0);
      yv = JS_GetPropertyUint32(ctx, argv[i], 1);
    } else if(i + 1 < argc) {
      xv = argv[i++];
      yv = argv[i];
    }
    JS_ToFloat64(ctx, &point.x, xv);
    JS_ToFloat64(ctx, &point.y, yv);

    v->push_back(point);
  }
  return JS_UNDEFINED;
}

static JSValue
js_contour_pop(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* v;
  JSValue ret;
  JSValue x, y;
  JSPointData *ptr, point;
  int64_t n = 0;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;
  n = v->size();
  if(n > 0) {
    point = (*v)[n - 1];
    v->pop_back();
  } else {
    return JS_EXCEPTION;
  }

  ret = js_point_new(ctx, point.x, point.y);

  return ret;
}

static JSValue
js_contour_unshift(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* v;
  int i;
  double x, y;
  JSValueConst xv, yv;
  JSPointData point;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  for(i = 0; i < argc; i++) {
    if(JS_IsObject(argv[i])) {
      xv = JS_GetPropertyStr(ctx, argv[i], "x");
      yv = JS_GetPropertyStr(ctx, argv[i], "y");
    } else if(JS_IsArray(ctx, argv[i])) {

      xv = JS_GetPropertyUint32(ctx, argv[i], 0);
      yv = JS_GetPropertyUint32(ctx, argv[i], 1);
    } else if(i + 1 < argc) {
      xv = argv[i++];
      yv = argv[i];
    }
    JS_ToFloat64(ctx, &point.x, xv);
    JS_ToFloat64(ctx, &point.y, yv);

    v->insert(v->begin(), point);
  }
  return JS_UNDEFINED;
}

static JSValue
js_contour_shift(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* v;
  JSValue ret;
  JSValue x, y;
  JSPointData *ptr, point;
  int64_t n = 0;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;
  n = v->size();
  if(n > 0) {
    point = (*v)[0];
    v->erase(v->begin());
  } else {
    return JS_EXCEPTION;
  }

  ret = js_point_new(ctx, point.x, point.y);

  return ret;
}

static JSValue
js_contour_concat(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *other, *r;
  int i;
  double x, y;
  JSValue ret;
  JSValueConst xv, yv;
  JSPointData point;

  if(!(v = js_contour_data(ctx, this_val)))
    return JS_EXCEPTION;

  if(!(other = js_contour_data(ctx, argv[0])))
    return JS_EXCEPTION;

  ret = js_contour_new(ctx, *v);

  r = js_contour_data(ctx, ret);
  std::copy(other->cbegin(), other->cend(), std::back_inserter(*r));
  return ret;
}

static JSValue
js_contour_rotatedrectangleintersection(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData *v, *other = nullptr, *ptr;
  JSValue ret = JS_UNDEFINED;
  bool handleNested = true;
  std::vector<cv::Point2f> a, b, intersection;

  v = js_contour_data(ctx, this_val);
  if(!v)
    return JS_EXCEPTION;

  if(argc > 0) {
    other = static_cast<JSContourData*>(JS_GetOpaque2(ctx, argv[0], js_contour_class_id));
  }

  std::transform(v->begin(), v->end(), std::back_inserter(a), [](const cv::Point2d& pt) -> cv::Point2f {
    return cv::Point2f(pt.x, pt.y);
  });

  std::transform(other->begin(), other->end(), std::back_inserter(b), [](const cv::Point2d& pt) -> cv::Point2f {
    return cv::Point2f(pt.x, pt.y);
  });
  {
    cv::RotatedRect rra(a[0], a[1], a[2]);
    cv::RotatedRect rrb(b[0], b[1], b[2]);

    cv::rotatedRectangleIntersection(rra, rrb, intersection);

    ret = js_contour_new(ctx, intersection);
  }
  return ret;
}

static JSValue
js_contour_rotatepoints(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* s = js_contour_data(ctx, this_val);
  int32_t shift = 1;
  uint32_t size = s->size();
  if(!s)
    return JS_EXCEPTION;
  if(argc > 0)
    JS_ToInt32(ctx, &shift, argv[0]);

  shift %= size;

  if(shift > 0) {
    std::rotate(s->begin(), s->begin() + shift, s->end());

  } else if(shift < 0) {
    std::rotate(s->rbegin(), s->rbegin() + (-shift), s->rend());
  }
  return JSValue(this_val);
}

static JSValue
js_contour_toarray(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSContourData* s = js_contour_data(ctx, this_val);
  uint32_t i, size = s->size();
  JSValue ret = JS_UNDEFINED;

  if(!s)
    return JS_EXCEPTION;

  ret = JS_NewArray(ctx);

  for(i = 0; i < size; i++) {
    JS_SetPropertyUint32(ctx, ret, i, js_point_clone(ctx, (*s)[i]));
  }

  return ret;
}

static JSValue
js_contour_tostring(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSContourData* s = js_contour_data(ctx, this_val);
  std::ostringstream os;
  int i = 0;
  int prec = 9;
  int32_t flags;

  if(!s)
    return JS_EXCEPTION;

  if(magic == 1 && !(flags & 0x100))
    os << "new Contour(";

  if(argc > 0)
    JS_ToInt32(ctx, &flags, argv[0]);
  else
    flags = 0;

  if(!(flags & 0x100))
    os << '[';

  std::for_each(s->begin(), s->end(), [&i, &os, flags, prec](const JSPointData& point) {
    if(i > 0)
      os << ((flags & 0x10) ? " " : ",");

    if(!(flags & 0x100))
      os << ((flags & 0x03) == 0 ? "{" : "[");

    if(flags & 0x03)
      os << std::setprecision(prec) << point.x << "," << point.y;
    else
      os << "x:" << std::setprecision(prec) << point.x << ",y:" << point.y;

    if(!(flags & 0x100))
      os << ((flags & 0x03) == 0 ? "}" : "]");

    i++;
  });
  if(!(flags & 0x100))
    os << ']'; // << std::endl;

  if(magic == 1 && !(flags & 0x100))
    os << ")";

  return JS_NewString(ctx, os.str().c_str());
}

static JSValue
js_contour_rect(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSValue ret = JS_UNDEFINED;
  std::vector<cv::Point2d> points;
  JSRectData s = js_rect_get(ctx, argv[0]);

  points.push_back(cv::Point2d(s.x, s.y));
  points.push_back(cv::Point2d(s.x + s.width, s.y));
  points.push_back(cv::Point2d(s.x + s.width, s.y + s.height));
  points.push_back(cv::Point2d(s.x, s.y + s.height));
  points.push_back(cv::Point2d(s.x, s.y));

  ret = js_contour2d_new(ctx, points);
  return ret;
}

JSValue contour_proto, contour_class;
VISIBLE JSClassID js_contour_class_id;

JSClassDef js_contour_class = {
    .class_name = "Contour",
    .finalizer = js_contour_finalizer,
};

JSValue
js_contour_iterator(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSContourData* s = js_contour_data(ctx, this_val);

  return js_point_iterator_new(ctx, std::make_pair(&(*s)[0], &(*s)[s->size()]), magic);
}

const JSCFunctionListEntry js_contour_proto_funcs[] = {
    JS_CFUNC_DEF("push", 1, js_contour_push),
    JS_CFUNC_DEF("pop", 0, js_contour_pop),
    JS_CFUNC_DEF("unshift", 1, js_contour_unshift),
    JS_CFUNC_DEF("shift", 0, js_contour_shift),
    JS_CFUNC_DEF("concat", 1, js_contour_concat),
    JS_CFUNC_DEF("get", 1, js_contour_get),
    JS_CGETSET_DEF("length", js_contour_length, NULL),
    JS_CGETSET_DEF("area", js_contour_area, NULL),
    JS_CGETSET_DEF("center", js_contour_center, NULL),
    JS_CFUNC_DEF("approxPolyDP", 1, js_contour_approxpolydp),
    JS_CFUNC_DEF("convexHull", 1, js_contour_convexhull),
    JS_CFUNC_DEF("boundingRect", 0, js_contour_boundingrect),
    JS_CFUNC_DEF("fitEllipse", 0, js_contour_fitellipse),
    JS_CFUNC_DEF("fitLine", 0, js_contour_fitline),
    JS_CFUNC_DEF("intersectConvex", 0, js_contour_intersectconvex),
    JS_CFUNC_DEF("isConvex", 0, js_contour_isconvex),
    JS_CFUNC_DEF("minAreaRect", 0, js_contour_minarearect),
    JS_CFUNC_DEF("minEnclosingCircle", 0, js_contour_minenclosingcircle),
    JS_CFUNC_DEF("minEnclosingTriangle", 0, js_contour_minenclosingtriangle),
    JS_CFUNC_DEF("pointPolygonTest", 0, js_contour_pointpolygontest),
    JS_CFUNC_DEF("rotatedRectangleIntersection", 0, js_contour_rotatedrectangleintersection),
    JS_CFUNC_DEF("arcLength", 0, js_contour_arclength),
    JS_CFUNC_DEF("rotatePoints", 1, js_contour_rotatepoints),
    JS_CFUNC_DEF("convexityDefects", 1, js_contour_convexitydefects),
    JS_CFUNC_MAGIC_DEF("simplifyReumannWitkam", 0, js_contour_psimpl, 0),
    JS_CFUNC_MAGIC_DEF("simplifyOpheim", 0, js_contour_psimpl, 1),
    JS_CFUNC_MAGIC_DEF("simplifyLang", 0, js_contour_psimpl, 2),
    JS_CFUNC_MAGIC_DEF("simplifyDouglasPeucker", 0, js_contour_psimpl, 3),
    JS_CFUNC_MAGIC_DEF("simplifyNthPoint", 0, js_contour_psimpl, 4),
    JS_CFUNC_MAGIC_DEF("simplifyRadialDistance", 0, js_contour_psimpl, 5),
    JS_CFUNC_MAGIC_DEF("simplifyPerpendicularDistance", 0, js_contour_psimpl, 6),
    JS_CFUNC_DEF("toArray", 0, js_contour_toarray),
    JS_CFUNC_MAGIC_DEF("toString", 0, js_contour_tostring, 0),
    JS_CFUNC_MAGIC_DEF("toSource", 0, js_contour_tostring, 1),
    JS_CFUNC_MAGIC_DEF("entries", 0, js_contour_iterator, 0),
    JS_CFUNC_MAGIC_DEF("keys", 0, js_contour_iterator, 1),
    JS_CFUNC_MAGIC_DEF("values", 0, js_contour_iterator, 2),

    JS_ALIAS_DEF("[Symbol.iterator]", "entries"),
    JS_ALIAS_DEF("size", "length"),
    //    JS_ALIAS_DEF("[Symbol.toStringTag]", "toString"),

    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "Contour", JS_PROP_CONFIGURABLE),

};
const JSCFunctionListEntry js_contour_static_funcs[] = {
    JS_CFUNC_DEF("fromRect", 1, js_contour_rect),
    JS_PROP_INT32_DEF("FORMAT_XY", 0x00, 0),
    JS_PROP_INT32_DEF("FORMAT_01", 0x02, 0),
    JS_PROP_INT32_DEF("FORMAT_SPACE", 0x10, 0),
    JS_PROP_INT32_DEF("FORMAT_COMMA", 0x00, 0),
    JS_PROP_INT32_DEF("FORMAT_BRACKET", 0x00, 0),
    JS_PROP_INT32_DEF("FORMAT_NOBRACKET", 0x100, 0),
};

int
js_contour_init(JSContext* ctx, JSModuleDef* m) {

  /* create the Contour class */
  JS_NewClassID(&js_contour_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_contour_class_id, &js_contour_class);

  contour_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx, contour_proto, js_contour_proto_funcs, countof(js_contour_proto_funcs));
  JS_SetClassProto(ctx, js_contour_class_id, contour_proto);

  contour_class = JS_NewCFunction2(ctx, js_contour_ctor, "Contour", 2, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, contour_class, contour_proto);
  JS_SetPropertyFunctionList(ctx, contour_class, js_contour_static_funcs, countof(js_contour_static_funcs));

  if(m)
    JS_SetModuleExport(ctx, m, "Contour", contour_class);
  /*  else
      JS_SetPropertyStr(ctx, *static_cast<JSValue*>(m), "Contour", contour_class);*/
  return 0;
}

JSModuleDef*
JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_contour_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "Contour");
  return m;
}

void
js_contour_constructor(JSContext* ctx, JSValue parent, const char* name) {
  if(JS_IsUndefined(contour_class))
    js_contour_init(ctx, 0);

  JS_SetPropertyStr(ctx, parent, name ? name : "Contour", contour_class);
}

}

template<>
JSValue
js_contour_new<float>(JSContext* ctx, const std::vector<cv::Point_<float>>& points) {
  JSValue ret;
  JSContourData* contour;

  ret = JS_NewObjectProtoClass(ctx, contour_proto, js_contour_class_id);

  contour = static_cast<JSContourData*>(js_mallocz(ctx, sizeof(JSContourData)));

  contour->resize(points.size());

  transform_points(points.cbegin(), points.cend(), contour->begin());

  JS_SetOpaque(ret, contour);
  return ret;
};

template<>
JSValue
js_contour_new<double>(JSContext* ctx, const std::vector<cv::Point_<double>>& points) {
  JSValue ret;
  JSContourData* contour;

  ret = JS_NewObjectProtoClass(ctx, contour_proto, js_contour_class_id);

  contour = static_cast<JSContourData*>(js_mallocz(ctx, sizeof(JSContourData)));

  std::copy(points.cbegin(), points.cend(), std::back_inserter(*contour));

  JS_SetOpaque(ret, contour);
  return ret;
}
