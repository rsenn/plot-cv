#include "./jsbindings.h"
#include "./geometry.h"

#if defined(JS_MAT_MODULE) || defined(quickjs_mat_EXPORTS)
#define JS_INIT_MODULE VISIBLE js_init_module
#else
#define JS_INIT_MODULE VISIBLE js_init_module_mat
#endif

JSMatData*
js_mat_data(JSContext* ctx, JSValue val) {
  return static_cast<JSMatData*>(JS_GetOpaque2(ctx, val, js_mat_class_id));
}

static JSValue
js_mat_new(JSContext* ctx, int cols, int rows, int type) {
  JSValue ret;
  JSMatData* s;
  ret = JS_NewObjectProtoClass(ctx, mat_proto, js_mat_class_id);

  s = static_cast<JSMatData*>(js_mallocz(ctx, sizeof(JSMatData)));

  // new ( s) cv::Mat(cv::Size(cols, rows), type);*/
  s->mat = cv::Mat::zeros(cv::Size(cols, rows), type);
  /* const auto init = cv::Mat::zeros(cv::Size(cols, rows), type);

   memcpy(s, &init, sizeof(*s));*/

  //*s = init;

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
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */

  /*struct list_head {
      struct list_head *prev;
      struct list_head *next;
  } *list_ptr, *list;

  list_ptr = list = ((list_head**)rt)[17];
  std::cerr << __FUNCTION__ << " " << s << std::endl;

  for(int i = 0; list_ptr; list_ptr = list_ptr->next, i++) {
    if(list_ptr >= s)
  std::cerr << __FUNCTION__ << " " << i << ": " << list_ptr << std::endl;
  }

  s->cv::Mat::~Mat();
  */

  s->mat.release();
  js_free_rt(rt, s);
  //
}

static JSValue
js_mat_funcs(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValue ret = JS_UNDEFINED;
  int64_t i = -1, i2 = -1;
  JSPointData pt;
  cv::Mat* m = &js_mat_data(ctx, this_val)->mat;

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
  }

  return JS_EXCEPTION;
}

static JSValue
js_mat_get_props(JSContext* ctx, JSValueConst this_val, int magic) {
  cv::Mat* m = &js_mat_data(ctx, this_val)->mat;
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
  return JS_UNDEFINED;
}

static JSValue
js_mat_tostring(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  cv::Mat* m = &js_mat_data(ctx, this_val)->mat;
  int x, y;

  std::ostringstream os;
  int i = 0;
  if(!m)
    return JS_EXCEPTION;

  if(m->rows * m->cols > 50) {
    os << "cv::Mat(" << m->rows << ", " << m->cols << ", ";

    const char* tstr =
        (m->type() == CV_8UC4) ? "CV_8UC4" : (m->type() == CV_8UC2) ? "CV_8UC2" : (m->type() == CV_8UC3) ? "CV_8UC3" : (m->type() == CV_8UC1) ? "CV_8UC1" : (m->type() == CV_32FC1) ? "CV_32FC1" : "?";

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
          os << std::setfill('0') << std::setbase(16) << std::setw(m->type() == CV_8UC4 ? 8 : m->type() == CV_8UC1 ? 2 : 6) << m->at<uint32_t>(y, x);
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

JSValue
js_mat_wrap(JSContext* ctx, const cv::Mat& mat) {
  JSValue ret;
  JSMatData* s;

  ret = JS_NewObjectProtoClass(ctx, mat_proto, js_mat_class_id);

  s = static_cast<JSMatData*>(js_mallocz(ctx, sizeof(JSMatData)));

  s->mat = cv::Mat(cv::Size(mat.cols, mat.rows), mat.type());

  JS_SetOpaque(ret, s);

  return ret;
}

JSValue mat_proto, mat_class;
JSClassID js_mat_class_id;

JSClassDef js_mat_class = {
    "Mat",
    .finalizer = js_mat_finalizer,
};

const JSCFunctionListEntry js_mat_proto_funcs[] = {
    JS_CGETSET_MAGIC_DEF("cols", js_mat_get_props, NULL, 0),
    JS_CGETSET_MAGIC_DEF("rows", js_mat_get_props, NULL, 1),
    JS_CGETSET_MAGIC_DEF("channels", js_mat_get_props, NULL, 2),
    JS_CGETSET_MAGIC_DEF("type", js_mat_get_props, NULL, 3),
    JS_CGETSET_MAGIC_DEF("depth", js_mat_get_props, NULL, 4),
    JS_CGETSET_MAGIC_DEF("empty", js_mat_get_props, NULL, 5),
    JS_CFUNC_MAGIC_DEF("col", 1, js_mat_funcs, 0),
    JS_CFUNC_MAGIC_DEF("row", 1, js_mat_funcs, 1),
    JS_CFUNC_MAGIC_DEF("colRange", 2, js_mat_funcs, 2),
    JS_CFUNC_MAGIC_DEF("rowRange", 2, js_mat_funcs, 3),
    JS_CFUNC_MAGIC_DEF("at", 1, js_mat_funcs, 4),
    JS_CFUNC_MAGIC_DEF("clone", 0, js_mat_funcs, 5),
    JS_CFUNC_MAGIC_DEF("roi", 0, js_mat_funcs, 6),
    // JS_CFUNC_DEF("findContours", 0, js_mat_findcontours),
    JS_CFUNC_DEF("toString", 0, js_mat_tostring),
    //    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "cv::Mat", JS_PROP_CONFIGURABLE)

};
const JSCFunctionListEntry js_mat_static_funcs[] = {
    JS_CFUNC_DEF("getRotationMatrix2D", 3, js_mat_getrotationmatrix2d),
};

int
js_mat_init(JSContext* ctx, JSModuleDef* m) {
  /* create the Mat class */
  JS_NewClassID(&js_mat_class_id);
  JS_NewClass(JS_GetRuntime(ctx), js_mat_class_id, &js_mat_class);

  mat_proto = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx, mat_proto, js_mat_proto_funcs, countof(js_mat_proto_funcs));
  JS_SetClassProto(ctx, js_mat_class_id, mat_proto);

  mat_class = JS_NewCFunction2(ctx, js_mat_ctor, "Mat", 2, JS_CFUNC_constructor, 0);
  /* set proto.constructor and ctor.prototype */
  JS_SetConstructor(ctx, mat_class, mat_proto);

  JS_SetPropertyFunctionList(ctx, mat_class, js_mat_static_funcs, countof(js_mat_static_funcs));

  JS_DefinePropertyValueStr(ctx, mat_class, "CV_8U", JS_NewInt32(ctx, CV_MAKETYPE(CV_8U, 1)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_8S", JS_NewInt32(ctx, CV_MAKETYPE(CV_8S, 1)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_16U", JS_NewInt32(ctx, CV_MAKETYPE(CV_16U, 1)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_16S", JS_NewInt32(ctx, CV_MAKETYPE(CV_16S, 1)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_32S", JS_NewInt32(ctx, CV_MAKETYPE(CV_32S, 1)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_32F", JS_NewInt32(ctx, CV_MAKETYPE(CV_32F, 1)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_64F", JS_NewInt32(ctx, CV_MAKETYPE(CV_64F, 1)), JS_PROP_ENUMERABLE);

  JS_DefinePropertyValueStr(ctx, mat_class, "CV_8UC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_8U, 1)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_8UC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_8U, 2)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_8UC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_8U, 3)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_8UC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_8U, 4)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_8SC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_8S, 1)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_8SC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_8S, 2)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_8SC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_8S, 3)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_8SC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_8S, 4)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_16UC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_16U, 1)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_16UC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_16U, 2)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_16UC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_16U, 3)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_16UC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_16U, 4)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_16SC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_16S, 1)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_16SC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_16S, 2)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_16SC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_16S, 3)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_16SC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_16S, 4)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_32SC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_32S, 1)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_32SC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_32S, 2)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_32SC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_32S, 3)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_32SC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_32S, 4)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_32FC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_32F, 1)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_32FC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_32F, 2)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_32FC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_32F, 3)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_32FC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_32F, 4)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_64FC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_64F, 1)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_64FC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_64F, 2)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_64FC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_64F, 3)), JS_PROP_ENUMERABLE);
  JS_DefinePropertyValueStr(ctx, mat_class, "CV_64FC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_64F, 4)), JS_PROP_ENUMERABLE);

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
