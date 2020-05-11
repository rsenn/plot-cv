#include "./jsbindings.h"
#include "./geometry.h"

extern "C" {


static JSValue
js_mat_new(JSContext* ctx, int cols, int rows, int type) {
  JSValue ret;
  JSMatData* s;
  ret = JS_NewObjectProtoClass(ctx, mat_proto, js_mat_class_id);
  s = new cv::Mat(cv::Size(cols, rows), type);

  *s = cv::Mat::zeros(cv::Size(cols, rows), type);

  JS_SetOpaque(ret, s);
  return ret;
}

static JSValue
js_mat_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSMatData* s;
  JSValue obj = JS_UNDEFINED;
  JSValue proto;
  JSSizeData* size = nullptr;

  int64_t cols = 0, rows = 0;
  int32_t type = CV_32FC1;

  if(argc > 0) {
    JS_ToInt64(ctx, &rows, argv[0]);
    size = js_size_data(ctx, argv[0]);
    if(size != nullptr) {
      cols = size->width;
      rows = size->height;
    } else {
      JS_ToInt64(ctx, &cols, argv[1]);
      argc--;
      argv++;
    }
    if(argc > 1) {
      type = JS_ToInt32(ctx, &type, argv[1]);
    }
  }

  obj = js_mat_new(ctx, cols, rows, type);

  return obj;
fail:
  s->release();

  JS_FreeValue(ctx, obj);
  return JS_EXCEPTION;
}

JSMatData*
js_mat_data(JSContext* ctx, JSValue val) {
  return static_cast<JSMatData*>(JS_GetOpaque2(ctx, val, js_mat_class_id));
}

void
js_mat_finalizer(JSRuntime* rt, JSValue val) {
  JSMatData* s = static_cast<JSMatData*>(JS_GetOpaque(val, js_mat_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */

  s->release();
}

static JSValue
js_mat_findcontours(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSMatData* m = js_mat_data(ctx, this_val);
  JSValue ret = JS_UNDEFINED;
  int mode = cv::RETR_TREE;
  int approx = cv::CHAIN_APPROX_SIMPLE;
  cv::Point offset(0, 0);

  contour2i_vector contours;
  vec4i_vector hier;
  contour2f_vector poly;

  cv::findContours(*m, contours, hier, mode, approx, offset);

  poly.resize(contours.size());

  transform_contours(contours.cbegin(), contours.cend(), poly.begin());

  {
    JSValue hier_arr = js_vector_vec4i_to_array(ctx, hier);
    JSValue contours_obj = js_contours_new(ctx, poly);

    ret = JS_NewObject(ctx);

    JS_SetPropertyStr(ctx, ret, "hier", hier_arr);
    JS_SetPropertyStr(ctx, ret, "contours", contours_obj);
  }
  return ret;
}

static JSValue
js_mat_funcs(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValue ret = JS_UNDEFINED;
  int64_t i = -1, i2 = -1;
  JSPointData* pt = nullptr;
  JSMatData* m = js_mat_data(ctx, this_val);

  if(argc > 0) {
    JS_ToInt64(ctx, &i, argv[0]);
    pt = js_point_data(ctx, argv[0]);
    if(argc > 2) {
      JS_ToInt64(ctx, &i2, argv[2]);
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

    if(pt != nullptr) {
      p = *pt;
    } else {
      p.x = i;
      p.y = i2;
    }
    if(m->type() == CV_32FC1)
      ret = JS_NewFloat64(ctx, (*m).at<float>(p.y, p.x));
    else
      ret = JS_NewInt64(ctx, (*m).at<uint32_t>(p.y, p.x));

  } else if(magic == 5) {
    ret = js_mat_wrap(ctx, m->clone());
  } else if(magic == 6) {
    JSRectData* rect = argc > 0 ? js_rect_data(ctx, argv[0]) : nullptr;
    ret = js_mat_wrap(ctx, (*m)(*rect));
  }

  return JS_EXCEPTION;
}

static JSValue
js_mat_get_props(JSContext* ctx, JSValueConst this_val, int magic) {
  JSMatData* s = js_mat_data(ctx, this_val);
  if(!s)
    return JS_EXCEPTION;
  if(magic == 0)
    return JS_NewFloat64(ctx, s->cols);
  else if(magic == 1)
    return JS_NewFloat64(ctx, s->rows);
  else if(magic == 2)
    return JS_NewFloat64(ctx, s->channels());
  else if(magic == 3)
    return JS_NewFloat64(ctx, s->type());
  else if(magic == 4)
    return JS_NewFloat64(ctx, s->depth());
  else if(magic == 5)
    return JS_NewBool(ctx, s->empty());
  return JS_UNDEFINED;
}

static JSValue
js_mat_tostring(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSMatData* m = js_mat_data(ctx, this_val);
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

JSValue
js_mat_wrap(JSContext* ctx, const cv::Mat& mat) {
  JSValue ret;
  JSMatData* s;

  ret = JS_NewObjectProtoClass(ctx, mat_proto, js_mat_class_id);

  s = new cv::Mat(cv::Size(mat.cols, mat.rows), mat.type());
  *s = mat;

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
    JS_CFUNC_DEF("findContours", 0, js_mat_findcontours),
    JS_CFUNC_DEF("toString", 0, js_mat_tostring),
    //    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "cv::Mat", JS_PROP_CONFIGURABLE)

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

  JS_SetPropertyStr(ctx, mat_class, "CV_8UC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_8U, 1)));
  JS_SetPropertyStr(ctx, mat_class, "CV_8UC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_8U, 2)));
  JS_SetPropertyStr(ctx, mat_class, "CV_8UC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_8U, 3)));
  JS_SetPropertyStr(ctx, mat_class, "CV_8UC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_8U, 4)));
  JS_SetPropertyStr(ctx, mat_class, "CV_8SC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_8S, 1)));
  JS_SetPropertyStr(ctx, mat_class, "CV_8SC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_8S, 2)));
  JS_SetPropertyStr(ctx, mat_class, "CV_8SC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_8S, 3)));
  JS_SetPropertyStr(ctx, mat_class, "CV_8SC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_8S, 4)));
  JS_SetPropertyStr(ctx, mat_class, "CV_16UC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_16U, 1)));
  JS_SetPropertyStr(ctx, mat_class, "CV_16UC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_16U, 2)));
  JS_SetPropertyStr(ctx, mat_class, "CV_16UC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_16U, 3)));
  JS_SetPropertyStr(ctx, mat_class, "CV_16UC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_16U, 4)));
  JS_SetPropertyStr(ctx, mat_class, "CV_16SC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_16S, 1)));
  JS_SetPropertyStr(ctx, mat_class, "CV_16SC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_16S, 2)));
  JS_SetPropertyStr(ctx, mat_class, "CV_16SC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_16S, 3)));
  JS_SetPropertyStr(ctx, mat_class, "CV_16SC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_16S, 4)));
  JS_SetPropertyStr(ctx, mat_class, "CV_32SC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_32S, 1)));
  JS_SetPropertyStr(ctx, mat_class, "CV_32SC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_32S, 2)));
  JS_SetPropertyStr(ctx, mat_class, "CV_32SC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_32S, 3)));
  JS_SetPropertyStr(ctx, mat_class, "CV_32SC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_32S, 4)));
  JS_SetPropertyStr(ctx, mat_class, "CV_32FC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_32F, 1)));
  JS_SetPropertyStr(ctx, mat_class, "CV_32FC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_32F, 2)));
  JS_SetPropertyStr(ctx, mat_class, "CV_32FC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_32F, 3)));
  JS_SetPropertyStr(ctx, mat_class, "CV_32FC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_32F, 4)));
  JS_SetPropertyStr(ctx, mat_class, "CV_64FC1", JS_NewInt32(ctx, CV_MAKETYPE(CV_64F, 1)));
  JS_SetPropertyStr(ctx, mat_class, "CV_64FC2", JS_NewInt32(ctx, CV_MAKETYPE(CV_64F, 2)));
  JS_SetPropertyStr(ctx, mat_class, "CV_64FC3", JS_NewInt32(ctx, CV_MAKETYPE(CV_64F, 3)));
  JS_SetPropertyStr(ctx, mat_class, "CV_64FC4", JS_NewInt32(ctx, CV_MAKETYPE(CV_64F, 4)));
  JSValue g = JS_GetGlobalObject(ctx);
  int32array_ctor = JS_GetProperty(ctx, g, JS_ATOM_Int32Array);
  int32array_proto = JS_GetPrototype(ctx, int32array_ctor);

  if(true)
    JS_SetModuleExport(ctx, static_cast<JSModuleDef*>(m), "Mat", mat_class);
  /*else
    JS_SetPropertyStr(ctx, *static_cast<JSValue*>(m), "Mat", mat_class);*/
  return 0;
}


JSModuleDef*
js_init_mat_module(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_mat_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "Mat");
  return m;
}

}
