#include "./jsbindings.h"
#include "./geometry.h"
#include "../quickjs/cutils.h"

#if defined(JS_MAT_MODULE) || defined(quickjs_mat_EXPORTS)
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module
#else
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module_mat
#endif

typedef struct JSMatIteratorData {
  JSValue obj;
  uint32_t row, col;
  int magic;
} JSMatIteratorData;

typedef struct JSMatSizeData {
  size_t rows, cols;
} JSMatSizeData;

VISIBLE JSMatData*
js_mat_data(JSContext* ctx, JSValueConst val) {
  return static_cast<JSMatData*>(JS_GetOpaque2(ctx, val, js_mat_class_id));
}

VISIBLE JSValue
js_mat_new(JSContext* ctx, int cols, int rows, int type) {
  JSValue ret;
  JSMatData* s;
  ret = JS_NewObjectProtoClass(ctx, mat_proto, js_mat_class_id);

  s = static_cast<JSMatData*>(js_mallocz(ctx, sizeof(JSMatData)));

  new(s) cv::Mat(cv::Size(cols, rows), type);

  *s = cv::Mat::zeros(cv::Size(cols, rows), type);

  s->addref();

  JS_SetOpaque(ret, s);
  return ret;
}

static JSValue
js_mat_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSValue obj = JS_UNDEFINED;
  JSValue proto;
  JSSizeData size;

  int64_t cols = 0, rows = 0;
  uint32_t type = CV_32FC1;

  if(argc > 0) {
    if(js_size_read(ctx, argv[0], &size)) {
      cols = size.width;
      rows = size.height;
      argv++;
      argc--;
    } else {
      JS_ToInt64(ctx, &rows, argv[0]);
      JS_ToInt64(ctx, &cols, argv[1]);
      argv += 2;
      argc -= 2;
    }

    if(argc > 0) {
      if(!JS_ToUint32(ctx, &type, argv[0])) {
        argv++;
        argc--;
      }
    }
  }

  obj = js_mat_new(ctx, cols, rows, type);

  return obj;
}

void
js_mat_finalizer(JSRuntime* rt, JSValue val) {
  JSMatData* s = static_cast<JSMatData*>(JS_GetOpaque(val, js_mat_class_id));

  s->release();

  js_free_rt(rt, s);

  JS_FreeValueRT(rt, val);
}

static JSValue
js_mat_funcs(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValue ret = JS_UNDEFINED;
  int64_t i = -1, i2 = -1;
  JSPointData pt;
  JSMatData* m = js_mat_data(ctx, this_val);

  if(argc > 0) {
    JS_ToInt64(ctx, &i, argv[0]);
    pt = js_point_get(ctx, argv[0]);
    if(argc > 1) {
      JS_ToInt64(ctx, &i2, argv[1]);
    }
  }

  if(magic == 0)
    ret = js_mat_wrap(ctx, m->col(i));
  else if(magic == 1)
    ret = js_mat_wrap(ctx, m->row(i));
  else if(magic == 2)
    ret = js_mat_wrap(ctx, m->colRange(i, i2));
  else if(magic == 3)
    ret = js_mat_wrap(ctx, m->rowRange(i, i2));
  else if(magic == 4) {
    cv::Point p;

    if(JS_IsNumber(argv[1])) {
      p.x = i;
      p.y = i2;
    } else {
      p = pt;
    }
    if(m->type() == CV_32FC1)
      ret = JS_NewFloat64(ctx, (*m).at<float>(p.y, p.x));
    else
      ret = JS_NewInt64(ctx, (*m).at<uint32_t>(p.y, p.x));

  } else if(magic == 5) {
    ret = js_mat_wrap(ctx, m->clone());
  } else if(magic == 6) {
    JSRectData rect = {0, 0, 0, 0};

    if(argc > 0)
      rect = js_rect_get(ctx, argv[0]);

    ret = js_mat_wrap(ctx, (*m)(rect));

  } else if(magic == 7) {
    JSRectData rect = {0, 0, 0, 0};

    if(argc > 0)
      rect = js_rect_get(ctx, argv[0]);

    ret = js_mat_wrap(ctx, (*m)(rect));
  } else {
    ret = JS_EXCEPTION;
  }

  return ret;
}

template<class T>
void
js_mat_get(JSContext* ctx, JSValueConst this_val, uint32_t row, uint32_t col, T& value) {
  cv::Mat* m = js_mat_data(ctx, this_val);

  if(m)
    value = (*m).at<T>(row, col);
  else
    value = T();
}

static JSValue
js_mat_get(JSContext* ctx, JSValueConst this_val, uint32_t row, uint32_t col) {
  JSValue ret = JS_EXCEPTION;
  cv::Mat* m = js_mat_data(ctx, this_val);
  uint32_t bytes = (1 << m->depth()) * m->channels();

  if(m) {
    switch(m->type()) {

      case CV_8UC1: {
        uint8_t value;
        js_mat_get(ctx, this_val, row, col, value);
        ret = JS_NewUint32(ctx, value);
        break;
      }
      case CV_8UC2: {
        cv::Vec2b value;
        js_mat_get(ctx, this_val, row, col, value);
        ret = JS_NewArray(ctx);
        JS_SetPropertyUint32(ctx, ret, 0, JS_NewUint32(ctx, value[0]));
        JS_SetPropertyUint32(ctx, ret, 1, JS_NewUint32(ctx, value[1]));
        break;
      }
      case CV_8UC3: {
        cv::Vec3b value;
        js_mat_get(ctx, this_val, row, col, value);
        ret = JS_NewArray(ctx);
        JS_SetPropertyUint32(ctx, ret, 0, JS_NewUint32(ctx, value[0]));
        JS_SetPropertyUint32(ctx, ret, 1, JS_NewUint32(ctx, value[1]));
        JS_SetPropertyUint32(ctx, ret, 2, JS_NewUint32(ctx, value[2]));
        break;
      }
      case CV_8UC4: {
        cv::Vec4b value;
        js_mat_get(ctx, this_val, row, col, value);
        ret = JS_NewArray(ctx);
        JS_SetPropertyUint32(ctx, ret, 0, JS_NewUint32(ctx, value[0]));
        JS_SetPropertyUint32(ctx, ret, 1, JS_NewUint32(ctx, value[1]));
        JS_SetPropertyUint32(ctx, ret, 2, JS_NewUint32(ctx, value[2]));
        JS_SetPropertyUint32(ctx, ret, 3, JS_NewUint32(ctx, value[3]));
        break;
      }
      case CV_16UC1: {
        uint16_t value;
        js_mat_get(ctx, this_val, row, col, value);
        ret = JS_NewUint32(ctx, value);
        break;
      }
      case CV_32SC1: {
        int32_t value;
        js_mat_get(ctx, this_val, row, col, value);
        ret = JS_NewInt32(ctx, value);
        break;
      }
      case CV_32FC1: {
        float value;
        js_mat_get(ctx, this_val, row, col, value);
        ret = JS_NewFloat64(ctx, value);
        break;
      }
      case CV_64FC1: {
        double value;
        js_mat_get(ctx, this_val, row, col, value);
        ret = JS_NewFloat64(ctx, value);
        break;
      }
    }
  }
  return ret;
}

static int
js_mat_get_wh(JSContext* ctx, JSMatSizeData* size, JSValueConst obj) {
  cv::Mat* m = js_mat_data(ctx, obj);

  if(m) {
    size->rows = m->rows;
    size->cols = m->cols;
    return 1;
  }
  return 0;
}

static JSValue
js_mat_at(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSMatData* m = js_mat_data(ctx, this_val);
  if(!m)
    return JS_EXCEPTION;
  JSPointData pt;
  JSValue ret;
  uint32_t row = 0, col = 0;
  if(js_point_read(ctx, argv[0], &pt)) {
    col = pt.x;
    row = pt.y;
  } else if(argc >= 2 && JS_IsNumber(argv[0]) && JS_IsNumber(argv[1])) {
    JS_ToUint32(ctx, &row, argv[0]);
    JS_ToUint32(ctx, &col, argv[1]);
    argc -= 2;
    argv += 2;
  } else if(argc >= 1 && JS_IsNumber(argv[0])) {
    JSMatSizeData dim = {static_cast<size_t>(m->rows), static_cast<size_t>(m->cols)};
    uint32_t idx;

    JS_ToUint32(ctx, &idx, argv[0]);
    row = idx / dim.cols;
    col = idx % dim.cols;
  }

  return js_mat_get(ctx, this_val, row, col);
}

static JSValue
js_mat_set(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  cv::Mat* m = js_mat_data(ctx, this_val);
  uint32_t bytes;
  if(!m)
    return JS_EXCEPTION;

  JSPointData pt;
  JSValue ret;
  int64_t col = -1, row = -1;

  if(js_point_read(ctx, argv[0], &pt)) {
    col = pt.x;
    row = pt.y;
    argc--;
    argv++;
  } else {
    if(argc >= 1) {
      JS_ToInt64(ctx, &row, argv[0]);
      argc--;
      argv++;
    }
    if(argc >= 1) {
      JS_ToInt64(ctx, &col, argv[0]);
      argc--;
      argv++;
    }
  }
  bytes = (1 << m->depth()) * m->channels();
  if(m->type() == CV_32FC1) {
    double data;
    if(JS_ToFloat64(ctx, &data, argv[0]))
      return JS_EXCEPTION;
    (*m).at<float>(row, col) = (float)data;
  } else if(bytes <= sizeof(uint)) {
    uint32_t mask = (1LU << (bytes * 8)) - 1;
    uint32_t data;
    if(JS_ToUint32(ctx, &data, argv[0]))
      return JS_EXCEPTION;

    if(bytes <= 1) {
      uint8_t* p = &(*m).at<uint8_t>(row, col);
      *p = (uint8_t)data & mask;
    } else if(bytes <= 2) {
      uint16_t* p = &(*m).at<uint16_t>(row, col);
      *p = (uint16_t)data & mask;

    } else if(bytes <= 4) {
      uint* p = &(*m).at<uint>(row, col);
      *p = (uint)data & mask;
    }

  } else
    return JS_EXCEPTION;
  return JS_UNDEFINED;
}

template<class T>
typename std::enable_if<std::is_integral<T>::value, void>::type
js_mat_vector_get(JSContext* ctx,
                  int argc,
                  JSValueConst* argv,
                  std::vector<T>& output,
                  std::vector<bool>& defined) {
  output.resize(static_cast<size_t>(argc));
  defined.resize(static_cast<size_t>(argc));
  for(int i = 0; i < argc; i++) {
    uint32_t val = 0;
    bool isDef = JS_IsNumber(argv[i]) && !JS_ToUint32(ctx, &val, argv[i]);

    output[i] = val;
    defined[i] = isDef;
  }
}

template<class T>
typename std::enable_if<std::is_floating_point<T>::value, void>::type
js_mat_vector_get(JSContext* ctx,
                  int argc,
                  JSValueConst* argv,
                  std::vector<T>& output,
                  std::vector<bool>& defined) {
  output.resize(static_cast<size_t>(argc));
  defined.resize(static_cast<size_t>(argc));
  for(int i = 0; i < argc; i++) {
    double val = 0;
    bool isDef = JS_IsNumber(argv[i]) && !JS_ToFloat64(ctx, &val, argv[i]);

    output[i] = val;
    defined[i] = isDef;
  }
}

template<class T>
typename std::enable_if<std::is_integral<typename T::value_type>::value, void>::type
js_mat_vector_get(JSContext* ctx,
                  int argc,
                  JSValueConst* argv,
                  std::vector<T>& output,
                  std::vector<bool>& defined) {
  const size_t bits = (sizeof(typename T::value_type) * 8);
  const size_t n = T::channels;
  output.resize(static_cast<size_t>(argc));
  defined.resize(static_cast<size_t>(argc));
  for(int i = 0; i < argc; i++) {
    double val = 0;
    bool isDef = JS_IsNumber(argv[i]) && !JS_ToFloat64(ctx, &val, argv[i]);
    if(isDef) {
      const uint64_t mask = (1U << bits) - 1;
      uint64_t ival = val;
      for(int j = 0; j < n; j++) {
        output[i][j] = ival & mask;
        ival >>= bits;
      }
    }
    defined[i] = isDef;
  }
};

template<class T>
static std::vector<T>
js_mat_set_vector(JSContext* ctx, JSMatData* m, int argc, JSValueConst* argv) {
  JSMatSizeData dim = {static_cast<size_t>(m->rows), static_cast<size_t>(m->cols)};
  uint32_t idx;
  std::vector<bool> defined;
  std::vector<T> v;
  js_mat_vector_get(ctx, argc, argv, v, defined);

  for(idx = 0; idx < v.size(); idx++)
    if(defined[idx])
      m->at<T>(idx / dim.cols, idx % dim.cols) = v[idx];
  return v;
}

static JSValue
js_mat_set_to(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSMatData* m = js_mat_data(ctx, this_val);
  uint32_t bytes;
  std::vector<bool> defined;
  if(!m)
    return JS_EXCEPTION;

  bytes = (1 << m->depth()) * m->channels();
  if(m->depth() == CV_16U && m->channels() > 1) {
    if(m->channels() == 2)
      js_mat_set_vector<cv::Vec<uint16_t, 2>>(ctx, m, argc, argv);
    else if(m->channels() == 3)
      js_mat_set_vector<cv::Vec<uint16_t, 3>>(ctx, m, argc, argv);
    else if(m->channels() == 4)
      js_mat_set_vector<cv::Vec<uint16_t, 4>>(ctx, m, argc, argv);
  } else if(m->depth() == CV_32F) {

    if(m->channels() == 1)
      js_mat_set_vector<float>(ctx, m, argc, argv);
    /*    else if(m->channels() == 2)
          js_mat_set_vector<cv::Vec<float, 2>>(ctx, s, argc, argv);
        else if(m->channels() == 3)
          js_mat_set_vector<cv::Vec<float, 3>>(ctx, s, argc, argv);
        else if(m->channels() == 4)
          js_mat_set_vector<cv::Vec<float, 4>>(ctx, s, argc, argv);*/

  } else if(bytes <= sizeof(uint)) {

    if(bytes <= 1) {
      std::vector<uint8_t> v;
      js_mat_vector_get(ctx, argc, argv, v, defined);
      m->setTo(cv::InputArray(v), defined);

    } else if(bytes <= 2) {
      std::vector<uint16_t> v;
      js_mat_vector_get(ctx, argc, argv, v, defined);
      m->setTo(cv::InputArray(v), defined);

    } else if(bytes <= 4) {
      js_mat_set_vector<uint32_t>(ctx, m, argc, argv);
    } else if(bytes <= 8) {
      js_mat_set_vector<uint64_t>(ctx, m, argc, argv);
      /*  JSMatSizeData dim;
        uint32_t idx;
        const auto v = js_mat_vector<uint32_t>::get(ctx, argc, argv, defined);
        if(!js_mat_get_wh(ctx, &dim, this_val))
          return JS_EXCEPTION;

        for(idx = 0; idx < v.size(); idx++) m->at<uint32_t>(idx / dim.cols, idx % dim.cols) =
        v[idx];*/
    }
  }

  return JS_UNDEFINED;
}

static JSValue
js_mat_get_props(JSContext* ctx, JSValueConst this_val, int magic) {
  cv::Mat* m = js_mat_data(ctx, this_val);
  if(!m)
    return JS_EXCEPTION;
  if(magic == 0)
    return JS_NewUint32(ctx, m->cols);
  else if(magic == 1)
    return JS_NewUint32(ctx, m->rows);
  else if(magic == 2)
    return JS_NewUint32(ctx, m->channels());
  else if(magic == 3)
    return JS_NewUint32(ctx, m->type());
  else if(magic == 4)
    return JS_NewUint32(ctx, m->depth());
  else if(magic == 5)
    return JS_NewBool(ctx, m->empty());
  else if(magic == 6)
    return JS_NewFloat64(ctx, m->total());
  return JS_UNDEFINED;
}

static JSValue
js_mat_tostring(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  cv::Mat* m = js_mat_data(ctx, this_val);
  int x, y;

  std::ostringstream os;
  int i = 0;
  if(!m)
    return JS_EXCEPTION;

  if(m->rows * m->cols > 50) {
    os << "cv::Mat(" << m->rows << ", " << m->cols << ", ";

    const char* tstr = (m->type() == CV_8UC4)
                           ? "CV_8UC4"
                           : (m->type() == CV_8UC2)
                                 ? "CV_8UC2"
                                 : (m->type() == CV_8UC3)
                                       ? "CV_8UC3"
                                       : (m->type() == CV_8UC1)
                                             ? "CV_8UC1"
                                             : (m->type() == CV_32FC1) ? "CV_32FC1" : "?";

    os << tstr << ")" << std::endl;
  } else {
    os << "Mat[";
    for(y = 0; y < m->rows; y++) {
      os << "\n  ";

      for(x = 0; x < m->cols; x++) {
        if(x > 0)
          os << ',';
        if(m->type() == CV_32FC1)
          os << m->at<float>(y, x);
        else
          os << std::setfill('0') << std::setbase(16)
             << std::setw(m->type() == CV_8UC4 ? 8 : m->type() == CV_8UC1 ? 2 : 6)
             << m->at<uint32_t>(y, x);
      }
    }

    os << ']' << std::endl;
  }

  return JS_NewString(ctx, os.str().c_str());
}

static JSValue
js_mat_getrotationmatrix2d(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData s;

  double angle = 0, scale = 1;
  cv::Mat m;

  JSValue ret;
  if(argc == 0)
    return JS_EXCEPTION;
  if(argc > 0) {
    s = js_point_get(ctx, argv[0]);
    if(argc > 1) {
      JS_ToFloat64(ctx, &angle, argv[1]);
      if(argc > 2) {
        JS_ToFloat64(ctx, &scale, argv[2]);
      }
    }
  }

  m = cv::getRotationMatrix2D(s, angle, scale);

  ret = js_mat_wrap(ctx, m);
  return ret;
}

VISIBLE JSValue
js_mat_wrap(JSContext* ctx, const cv::Mat& mat) {
  JSValue ret;
  JSMatData* s;

  ret = JS_NewObjectProtoClass(ctx, mat_proto, js_mat_class_id);

  s = static_cast<JSMatData*>(js_mallocz(ctx, sizeof(JSMatData)));

  new(s) cv::Mat(cv::Size(mat.cols, mat.rows), mat.type());
  *s = mat;

  s->addref();

  JS_SetOpaque(ret, s);

  return ret;
}

static JSValue
js_mat_create_vec(JSContext* ctx, int len, JSValue* vec) {
  JSValue obj = JS_EXCEPTION;
  int i;

  obj = JS_NewArray(ctx);
  if(!JS_IsException(obj)) {

    for(i = 0; i < len; i++) {

      if(JS_SetPropertyUint32(ctx, obj, i, vec[i]) < 0) {
        JS_FreeValue(ctx, obj);
        return JS_EXCEPTION;
      }
    }
  }
  return obj;
}

JSValue
js_create_mat_iterator(
    JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValue enum_obj, mat;
  JSMatIteratorData* it;
  mat = JS_DupValue(ctx, this_val);
  if(!JS_IsException(mat)) {
    enum_obj = JS_NewObjectProtoClass(ctx, mat_iterator_proto, js_mat_iterator_class_id);
    if(!JS_IsException(enum_obj)) {
      it = static_cast<JSMatIteratorData*>(js_malloc(ctx, sizeof(JSMatIteratorData)));

      it->obj = mat;
      it->row = 0;
      it->col = 0;
      it->magic = magic;

      JS_SetOpaque(/*ctx, */ enum_obj, it);
      return enum_obj;
    }
    JS_FreeValue(ctx, enum_obj);
  }
  JS_FreeValue(ctx, mat);
  return JS_EXCEPTION;
}

JSValue
js_mat_iterator_next(
    JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, BOOL* pdone, int magic) {
  JSMatIteratorData* it;
  uint32_t row, col;
  JSValue val, obj;
  JSMatSizeData dim;

  it = static_cast<JSMatIteratorData*>(JS_GetOpaque(this_val, js_mat_iterator_class_id));
  if(it) {
    if(!JS_IsUndefined(it->obj)) {
      if(js_mat_get_wh(ctx, &dim, it->obj)) {
        row = it->row;
        col = it->col;
        if(row >= dim.rows /*|| col >= dim.cols*/) {
          JS_FreeValue(ctx, it->obj);
          it->obj = JS_UNDEFINED;
        done:
          *pdone = TRUE;
          return JS_UNDEFINED;
        }
        if(col + 1 < dim.cols) {
          it->col = col + 1;
        } else {
          it->col = 0;
          it->row = row + 1;
        }
        *pdone = FALSE;
        if(it->magic == 0) {
          JSValue v[2] = {JS_NewUint32(ctx, row), JS_NewUint32(ctx, col)};
          return js_mat_create_vec(ctx, 2, v);
        } else if(it->magic == 1) {
          return js_mat_get(ctx, it->obj, row, col);
        } else {
          JSValue key[2] = {JS_NewUint32(ctx, row), JS_NewUint32(ctx, col)};
          JSValue entry[2] = {js_mat_create_vec(ctx, 2, key), js_mat_get(ctx, it->obj, row, col)};

          return js_mat_create_vec(ctx, 2, entry);
        }
      }
      *pdone = FALSE;
    }
  }
  return JS_EXCEPTION;
}

void
js_mat_iterator_finalizer(JSRuntime* rt, JSValue val) {
  JSMatIteratorData* it =
      static_cast<JSMatIteratorData*>(JS_GetOpaque(val, js_mat_iterator_class_id));
  js_free_rt(rt, it);
}

static JSValue
js_mat_iterator_dup(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  return JS_DupValue(ctx, this_val);
}

JSValue mat_proto, mat_class, mat_iterator_proto, mat_iterator_class;
JSClassID js_mat_class_id, js_mat_iterator_class_id;

JSClassDef js_mat_class = {
    .class_name = "Mat",
    .finalizer = js_mat_finalizer,
};

JSClassDef js_mat_iterator_class = {
    .class_name = "MatIterator",
    .finalizer = js_mat_iterator_finalizer,
};

const JSCFunctionListEntry js_mat_proto_funcs[] = {
    JS_CGETSET_MAGIC_DEF("cols", js_mat_get_props, NULL, 0),
    JS_CGETSET_MAGIC_DEF("rows", js_mat_get_props, NULL, 1),
    JS_CGETSET_MAGIC_DEF("channels", js_mat_get_props, NULL, 2),
    JS_CGETSET_MAGIC_DEF("type", js_mat_get_props, NULL, 3),
    JS_CGETSET_MAGIC_DEF("depth", js_mat_get_props, NULL, 4),
    JS_CGETSET_MAGIC_DEF("empty", js_mat_get_props, NULL, 5),
    JS_CGETSET_MAGIC_DEF("total", js_mat_get_props, NULL, 6),
    JS_CFUNC_MAGIC_DEF("col", 1, js_mat_funcs, 0),
    JS_CFUNC_MAGIC_DEF("row", 1, js_mat_funcs, 1),
    JS_CFUNC_MAGIC_DEF("colRange", 2, js_mat_funcs, 2),
    JS_CFUNC_MAGIC_DEF("rowRange", 2, js_mat_funcs, 3),
    // JS_CFUNC_MAGIC_DEF("at", 1, js_mat_funcs, 4),
    JS_CFUNC_MAGIC_DEF("clone", 0, js_mat_funcs, 5),
    JS_CFUNC_MAGIC_DEF("roi", 0, js_mat_funcs, 6),
    // JS_CFUNC_MAGIC_DEF("set", 3, js_mat_funcs, 7),
    // JS_CFUNC_DEF("findContours", 0, js_mat_findcontours),
    JS_CFUNC_DEF("toString", 0, js_mat_tostring),
    JS_CFUNC_DEF("at", 1, js_mat_at),
    JS_CFUNC_DEF("set", 2, js_mat_set),
    JS_CFUNC_DEF("setTo", 0, js_mat_set_to),
    JS_CFUNC_MAGIC_DEF("keys", 0, js_create_mat_iterator, 0),
    JS_CFUNC_MAGIC_DEF("values", 0, js_create_mat_iterator, 1),
    JS_CFUNC_MAGIC_DEF("entries", 0, js_create_mat_iterator, 2),
    JS_ALIAS_DEF("[Symbol.iterator]", "entries"),
    JS_ALIAS_DEF("[Symbol.toPrimitive]", "toString"),

    //    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "cv::Mat", JS_PROP_CONFIGURABLE)

};

const JSCFunctionListEntry js_mat_iterator_proto_funcs[] = {
    JS_ITERATOR_NEXT_DEF("next", 0, js_mat_iterator_next, 0),
    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "MatIterator", JS_PROP_CONFIGURABLE),
    JS_CFUNC_DEF("[Symbol.iterator]", 0, js_mat_iterator_dup),

};

const JSCFunctionListEntry js_mat_static_funcs[] = {
    JS_CFUNC_DEF("getRotationMatrix2D", 3, js_mat_getrotationmatrix2d),
    JS_PROP_INT32_DEF("CV_8U", CV_MAKETYPE(CV_8U, 1), JS_PROP_ENUMERABLE),
};

int
js_mat_init(JSContext* ctx, JSModuleDef* m) {
  /* create the Mat class */
  JS_NewClassID(&js_mat_class_id);
  JS_NewClassID(&js_mat_iterator_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_mat_class_id, &js_mat_class);
  JS_NewClass(JS_GetRuntime(ctx), js_mat_iterator_class_id, &js_mat_iterator_class);

  mat_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx, mat_proto, js_mat_proto_funcs, countof(js_mat_proto_funcs));
  JS_SetClassProto(ctx, js_mat_class_id, mat_proto);

  mat_iterator_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx,
                             mat_iterator_proto,
                             js_mat_iterator_proto_funcs,
                             countof(js_mat_iterator_proto_funcs));
  JS_SetClassProto(ctx, js_mat_iterator_class_id, mat_iterator_proto);

  mat_class = JS_NewCFunction2(ctx, js_mat_ctor, "Mat", 2, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, mat_class, mat_proto);

  JS_SetPropertyFunctionList(ctx, mat_class, js_mat_static_funcs, countof(js_mat_static_funcs));

  JSValue g = JS_GetGlobalObject(ctx);
  int32array_ctor = JS_GetProperty(ctx, g, JS_ATOM_Int32Array);
  int32array_proto = JS_GetPrototype(ctx, int32array_ctor);

  if(m)
    JS_SetModuleExport(ctx, m, "Mat", mat_class);
  /*else
    JS_SetPropertyStr(ctx, *static_cast<JSValue*>(m), "Mat", mat_class);*/
  return 0;
}

extern "C" JSModuleDef*
JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_mat_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "Mat");
  return m;
}
/*
void
js_mat_constructor(JSContext* ctx, JSValue parent, const char* name) {
  if(JS_IsUndefined(mat_class))
    js_mat_init(ctx, 0);

  JS_SetPropertyStr(ctx, parent, name ? name : "Mat", mat_class);
}*/
