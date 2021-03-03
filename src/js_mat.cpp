#include "jsbindings.hpp"
#include "js_size.hpp"
#include "js_point.hpp"
#include "js_rect.hpp"
#include "js_array.hpp"
#include "js_alloc.hpp"
#include "geometry.hpp"
#include "util.hpp"
#include "../quickjs/cutils.h"
#include <list>
#include <map>
#include <fstream>

#if defined(JS_MAT_MODULE) || defined(quickjs_mat_EXPORTS)
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module
#else
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module_mat
#endif


enum {
  MAT_EXPR_AND = 0,
  MAT_EXPR_OR,
  MAT_EXPR_XOR,
  MAT_EXPR_MUL
};

extern "C" {
JSValue mat_proto = JS_UNDEFINED, mat_class = JS_UNDEFINED, mat_iterator_proto = JS_UNDEFINED,
        mat_iterator_class = JS_UNDEFINED;
JSClassID js_mat_class_id = 0, js_mat_iterator_class_id = 0;

static void
js_mat_free_func(JSRuntime* rt, void* opaque, void* ptr) {
  static_cast<JSMatData*>(opaque)->release();
}
}

static std::vector<JSMatData*> mat_list;
static std::list<JSMatData*> mat_freed;

typedef struct JSMatIteratorData {
  JSValue obj;
  uint32_t row, col;
  int magic;
} JSMatIteratorData;

typedef struct JSMatSizeData {
  uint32_t rows, cols;
} JSMatSizeData;

VISIBLE JSMatData*
js_mat_data(JSContext* ctx, JSValueConst val) {
  return static_cast<JSMatData*>(JS_GetOpaque2(ctx, val, js_mat_class_id));
}

static inline std::vector<int>
js_mat_sizes(const JSMatData& mat) {
  const cv::MatSize size(mat.size);
  std::vector<int> sizes;
  if(mat.dims == 2) {
    sizes.push_back(mat.rows);
    sizes.push_back(mat.cols);
  } else {
    std::copy(&size[0], &size[size.dims()], std::back_inserter(sizes));
  }
  return sizes;
}

static inline std::vector<std::string>
js_mat_dimensions(const JSMatData& mat) {
  std::vector<int> sizes = js_mat_sizes(mat);
  std::vector<std::string> dimensions;

  std::transform(sizes.cbegin(),
                 sizes.cend(),
                 std::back_inserter(dimensions),
                 static_cast<std::string (*)(int)>(&std::to_string));
  return dimensions;
}

static inline JSMatData*
js_mat_track(JSContext* ctx, JSMatData* s) {
  std::vector<cv::Mat*> deallocate;

  for(;;) {
    auto it2 = std::find(mat_freed.cbegin(), mat_freed.cend(), s);
    if(it2 != mat_freed.cend()) {
      deallocate.push_back(s);

      // std::cerr << "allocated @" << static_cast<void*>(s) << " which is in free list" << std::endl;

      // mat_freed.erase(it2);
      s = js_allocate<cv::Mat>(ctx);
      memcpy(s, deallocate[deallocate.size() - 1], sizeof(JSMatData));

    } else {
      break;
    }
  }

  mat_list.push_back(s);

  for(const auto& ptr : deallocate) js_deallocate(ctx, ptr);
  return s;
}

#ifdef DEBUG_MAT
static inline std::map<void*, std::vector<JSMatData*>>
js_mat_data(void* data = nullptr) {
  std::map<void*, std::vector<JSMatData*>> ret;
  for(auto mat : mat_list) {
    const auto u = mat->u;
    if(u != nullptr && (data == nullptr || u == data)) {
      void* data = u;
      if(ret.find(data) == ret.end()) {
        std::vector<JSMatData*> v{mat};
        ret.insert({data, v});
      } else {
        ret[data].push_back(mat);
      }
    }
  }
  return ret;
}

static inline void
js_mat_print_data(const std::map<void*, std::vector<JSMatData*>>& data, size_t minSize = 1) {

  for(const auto& [key, value] : data) {

    if(value.size() >= minSize) {
      std::cerr << "data @" << key << " =";

      for(const auto& ptr : value) {
        size_t refcount = ptr->u ? ptr->u->refcount : 0;
        std::cerr << " mat @" << static_cast<void*>(ptr);
        if(refcount > 1)
          std::cerr << " (refcount=" << refcount << ")";
      }
      std::cerr << std::endl;
    }
  }
}

static inline void
js_mat_dump(JSMatData* const s) {
  auto posList = std::find(mat_list.cbegin(), mat_list.cend(), s);
  bool inList = posList != mat_list.cend();
  bool inFreed = std::find(mat_freed.cbegin(), mat_freed.cend(), s) != mat_freed.cend();
  const auto u = s->u;
  std::cerr << " mat"
            << "[" << (posList - mat_list.cbegin()) << "]=" << static_cast<void*>(s);

  if(inList)
    std::cerr << ", inList=" << (inList ? "true" : "false");
  if(inFreed)
    std::cerr << ", inFreed=" << (inFreed ? "true" : "false");

  if(s->rows || s->cols || s->channels() > 1 || s->depth() > 0) {
    std::cerr << ", rows=" << s->rows;
    std::cerr << ", cols=" << s->cols;
    std::cerr << ", channels=" << s->channels();
    std::cerr << ", depth=" << s->depth();
  }

  if(u != nullptr) {

    // if(u->refcount)
    std::cerr << ", refcount=" << u->refcount;
    if(u->data)
      std::cerr << ", data=" << static_cast<void*>(u->data);
    if(u->size)
      std::cerr << ", size=" << u->size;
  }
}
#endif

VISIBLE JSValue
js_mat_new(JSContext* ctx, uint32_t rows, uint32_t cols, int type) {
  JSValue ret;
  JSMatData* s;

  if(JS_IsUndefined(mat_proto))
    js_mat_init(ctx, NULL);

  ret = JS_NewObjectProtoClass(ctx, mat_proto, js_mat_class_id);

  s = js_mat_track(ctx, js_allocate<cv::Mat>(ctx));

  if(cols || rows || type) {
    new(s) cv::Mat(rows, cols, type);
    // *s = cv::Mat::zeros(rows, cols,  type);
  } else {
    new(s) cv::Mat();
  }

  // s->addref();
#ifdef DEBUG_MAT
  std::cerr << ((cols > 0 || rows > 0) ? "js_mat_new (h,w)" : "js_mat_new      ");
  js_mat_dump(s);
  std::cerr << std::endl;
#endif

  JS_SetOpaque(ret, s);
  return ret;
}

VISIBLE JSValue
js_mat_wrap(JSContext* ctx, const cv::Mat& mat) {
  JSValue ret;
  JSMatData* s;
  ret = JS_NewObjectProtoClass(ctx, mat_proto, js_mat_class_id);

  s = js_mat_track(ctx, js_allocate<cv::Mat>(ctx));
  new(s) cv::Mat();
  *s = mat;
#ifdef DEBUG_MAT
  std::cerr << "js_mat_wrap     ";
  auto posList = std::find(mat_list.cbegin(), mat_list.cend(), const_cast<JSMatData*>(&mat));
  bool inList;
  if((inList = posList != mat_list.cend())) {
    std::cerr << "arg[" << (posList - mat_list.cbegin()) << "]=" << static_cast<const void*>(&mat);
    std::cerr << ", inList(arg)=" << (inList ? "true" : "false");
  } else {
    std::cerr << "arg=" << static_cast<const void*>(&mat);
  }
  js_mat_dump(s);
  std::cerr << std::endl;
#endif

  JS_SetOpaque(ret, s);
  return ret;
}

static std::pair<JSSizeData<uint32_t>, int>
js_mat_params(JSContext* ctx, int argc, JSValueConst* argv) {
  JSSizeData<uint32_t> size;
  int32_t type = 0;
  if(argc > 0) {
    if(js_size_read(ctx, argv[0], &size)) {
      argv++;
      argc--;
    } else {
      JS_ToUint32(ctx, &size.height, argv[0]);
      JS_ToUint32(ctx, &size.width, argv[1]);
      argv += 2;
      argc -= 2;
    }
    if(argc > 0) {
      if(!JS_ToInt32(ctx, &type, argv[0])) {
        argv++;
        argc--;
      }
    }
  }
  return std::make_pair(size, type);
}

static JSValue
js_mat_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {

  const auto& [size, type] = js_mat_params(ctx, argc, argv);

  return js_mat_new(ctx, size.height, size.width, type);
}

void
js_mat_finalizer(JSRuntime* rt, JSValue val) {
  JSMatData* s = static_cast<JSMatData*>(JS_GetOpaque(val, js_mat_class_id));

  auto it2 = std::find(mat_freed.cbegin(), mat_freed.cend(), s);
  auto it = std::find(mat_list.cbegin(), mat_list.cend(), s);

  if(it2 != mat_freed.cend()) {
#ifdef DEBUG_MAT
    std::cerr << "js_mat_finalizer";
    js_mat_dump(s);
#endif

    std::cerr << " ERROR: already freed" << std::endl;
    return;
  }
  JS_FreeValueRT(rt, val);
  if(it != mat_list.cend()) {
    size_t refcount = s->u != nullptr ? s->u->refcount : 0;
    if(s->u) {
      auto data = s->u;
#ifdef DEBUG_MAT
      std::cerr << "cv::Mat::release";
      std::cerr << " mat=" << static_cast<void*>(s);
      std::cerr << ", refcount=" << refcount;
      std::cerr << std::endl;
#endif
      if(refcount > 1)
        s->release();
      if(s->u)
        refcount = s->u->refcount;
      else
        refcount = 0;
    }
    if(s->u == nullptr) {
      mat_list.erase(it);
      mat_freed.push_front(s);
    }
#ifdef DEBUG_MAT
    std::cerr << "js_mat_finalizer";
    js_mat_dump(s);
    std::cerr << ", refcount=" << refcount;
    std::cerr << ", mat_list.size()=" << mat_list.size();
    std::cerr << ", mat_freed.size()=" << mat_freed.size() << std::endl;
#endif
  } else {
#ifdef DEBUG_MAT
    std::cerr << "js_mat_finalizer";
    js_mat_dump(s);
#endif
    std::cerr << " ERROR: not found" << std::endl;
  }

  js_deallocate(rt, s);
}

static JSValue
js_mat_funcs(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValue ret = JS_UNDEFINED;
  int64_t i = -1, i2 = -1;
  JSPointData<double> pt;
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
    JSRectData<double> rect = {0, 0, 0, 0};

    if(argc > 0)
      rect = js_rect_get(ctx, argv[0]);

    ret = js_mat_wrap(ctx, (*m)(rect));

  } else if(magic == 7) {
    JSRectData<double> rect = {0, 0, 0, 0};

    if(argc > 0)
      rect = js_rect_get(ctx, argv[0]);

    ret = js_mat_wrap(ctx, (*m)(rect));

  } else if(magic == 8) {
    m->release();
  } else if(magic == 9) {
    ret = js_mat_wrap(ctx, *m);

  } else if(magic == 10) {
    *m = cv::Mat::zeros(m->rows, m->cols, m->type());
  } else if(magic == 11) {
    *m = cv::Mat();
  } else if(magic == 12) {
    uint32_t rows = 0;
    cv::Scalar color{0, 0, 0, 0};

    JS_ToUint32(ctx, &rows, argv[0]);
    if(argc > 1) {
      js_color_read(ctx, argv[1], &color);
      m->resize(rows, color);
    } else {
      m->resize(rows);
    }
  } else if(magic == 13) {
    int32_t i = 0;

    JS_ToInt32(ctx, &i, argv[0]);
    ret = JS_NewInt64(ctx, m->step1(i));
  } else if(magic == 14) {
    /*
        JSPointData<double>* ofs;
        JSSizeData<double>* wholeSize;

        if(!js_size_data(ctx, argv[0])) {
          JS_ThrowTypeError(ctx, "argument 1 must be cv::Size");
          ret = JS_EXCEPTION;
        } else if(!js_point_data(ctx, argv[1])) {
          JS_ThrowTypeError(ctx, "argument 2 must be cv::Point");
          ret = JS_EXCEPTION;
        } else*/
    {
      cv::Size wholeSize;
      cv::Point ofs;
      m->locateROI(wholeSize, ofs);
      js_size_write(ctx, argv[0], wholeSize);
      js_point_write(ctx, argv[0], ofs);

      /**ofs = pt;
       *wholeSize = sz;*/
    }
  } else {
    ret = JS_EXCEPTION;
  }

  return ret;
}

static JSValue
js_mat_expr(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValue ret = JS_UNDEFINED;
  JSColorData<double> color;
  JSMatData *src, *dst, *o;
  double scale = 1.0;

  if((src = js_mat_data(ctx, this_val)) == nullptr)
    return JS_EXCEPTION;

  cv::Mat& mat = *src;

  if(argc < 1)
    return JS_EXCEPTION;

  if((o = js_mat_data(ctx, argv[0])) == nullptr)
    if(!js_color_read(ctx, argv[0], &color))
      return JS_EXCEPTION;

  if(magic == 3 && argc > 1) {
    JS_ToFloat64(ctx, &scale, argv[1]);
    argv++;
    argc--;
  }

  if(argc > 1) {
    dst = js_mat_data(ctx, argv[1]);
  }

  if(dst == nullptr)
    dst = src;

  cv::Mat& out = *dst;

  if(o == nullptr) {
    cv::Scalar& scalar = *reinterpret_cast<cv::Scalar*>(&color);

    switch(magic) {
      case MAT_EXPR_AND: out = mat & scalar; break;
      case MAT_EXPR_OR: out = mat | scalar; break;
      case MAT_EXPR_XOR: out = mat ^ scalar; break;
      case MAT_EXPR_MUL: out = mat.mul(scalar, scale); break;
    }
  } else {
    cv::Mat /*&*/ other = *o;

    switch(magic) {
      case MAT_EXPR_AND: cv::bitwise_and(*src, *o, *dst); /*out = mat & other;*/ break;
      case MAT_EXPR_OR:  cv::bitwise_or(*src, *o, *dst); /*out = mat | other;*/ break;
      case MAT_EXPR_XOR: cv::bitwise_xor(*src, *o, *dst);/* out = mat ^ other;*/ break;
      case MAT_EXPR_MUL: out = mat.mul(other, scale); break;
    }
  }

  return ret;
}

static JSValue
js_mat_init(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValue ret = JS_UNDEFINED;
  JSMatData* m;

  auto [size, type] = js_mat_params(ctx, argc, argv);

  if((m = js_mat_data(ctx, this_val))) {
    if(size.width == 0 || size.height == 0) {
      if(m->rows && m->cols) {
        size.width = m->cols;
        size.height = m->rows;
        type = m->type();
      }
    }
    ret = JS_DupValue(ctx, this_val);
  } else if(size.width > 0 && size.height > 0) {
    ret = js_mat_new(ctx, size.height, size.width, type);

    m = js_mat_data(ctx, ret);
  } else {
    return JS_EXCEPTION;
  }

  cv::Mat& mat = *m;

  switch(magic) {
    case 0: {
      mat = cv::Mat::zeros(size, type);
      break;
    }
    case 1: {
      mat = cv::Mat::ones(size, type);
      break;
    }
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
  JSPointData<double> pt;
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
    JSMatSizeData dim = {static_cast<uint32_t>(m->rows), static_cast<uint32_t>(m->cols)};
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

  JSPointData<double> pt;
  JSValue ret;
  int32_t col = -1, row = -1;

  if(js_point_read(ctx, argv[0], &pt)) {
    col = pt.x;
    row = pt.y;
    argc--;
    argv++;
  } else {
    if(argc >= 1) {
      JS_ToInt32(ctx, &row, argv[0]);
      argc--;
      argv++;
    }
    if(argc >= 1) {
      JS_ToInt32(ctx, &col, argv[0]);
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
js_mat_vector_get(
    JSContext* ctx, int argc, JSValueConst* argv, std::vector<T>& output, std::vector<bool>& defined) {
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
js_mat_vector_get(
    JSContext* ctx, int argc, JSValueConst* argv, std::vector<T>& output, std::vector<bool>& defined) {
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
js_mat_vector_get(
    JSContext* ctx, int argc, JSValueConst* argv, std::vector<T>& output, std::vector<bool>& defined) {
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
  JSMatSizeData dim = {static_cast<uint32_t>(m->rows), static_cast<uint32_t>(m->cols)};
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

  if(argc >= 1 && JS_IsArray(ctx, argv[0])) {
    cv::Scalar s;
    size_t n = js_array_to(ctx, argv[0], s);

    // std::cerr << "Scalar [ " << s[0] << ", " << s[1] << ", " << s[2] << ", " << s[3] << " ]" <<
    // std::endl; std::cerr << "Scalar.size() = " << n << std::endl;

    if(n >= m->channels()) {
      m->setTo(s);
      return JS_UNDEFINED;
    }
  }

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
  else if(magic == 7)
    return js_size_new(ctx, m->cols, m->rows);
  else if(magic == 8)
    return JS_NewBool(ctx, m->isContinuous());
  else if(magic == 9)
    return JS_NewBool(ctx, m->isSubmatrix());
  else if(magic == 10)
    return JS_NewUint32(ctx, m->step);
  else if(magic == 11)
    return JS_NewUint32(ctx, m->elemSize());
  else if(magic == 12)
    return JS_NewUint32(ctx, m->elemSize1());
  else if(magic == 13) {
    size_t size = m->elemSize() * m->total();
    uint8_t* ptr = m->ptr();

    m->addref();
    // m->addref();

    return JS_NewArrayBuffer(ctx, ptr, size, &js_mat_free_func, m, TRUE);
  } else if(magic == 13) {
    JSValueConst global, typed_array, buffer;
    int elem_size = m->elemSize();
    const char* ctor;
    global = JS_GetGlobalObject(ctx);

    printf("m->type()=%x m->channels()=%x\n", m->type(), m->channels());
    switch(m->type()) {
      case CV_8U:
      case CV_8S: ctor = "Uint8Array"; break;
      case CV_16U:
      case CV_16S: ctor = "Uint16Array"; break;
      case CV_32S: ctor = "Int32Array"; break;
      case CV_32F: ctor = "Float32Array"; break;
      case CV_64F: ctor = "Float64Array"; break;
      default:
        JS_ThrowTypeError(ctx, "cv:Mat type=%02x channels=%02x", m->type(), m->channels());
        return JS_EXCEPTION;
    }
    typed_array = JS_GetPropertyStr(ctx, global, ctor);
    buffer = js_mat_get_props(ctx, this_val, 9);

    return JS_CallConstructor(ctx, typed_array, 1, &buffer);
  }

  return JS_UNDEFINED;
}

static JSValue
js_mat_tostring(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  cv::Mat* m = js_mat_data(ctx, this_val);
  int x, y;

  std::ostringstream os;
  std::string str;
  int i = 0;
  if(!m)
    return JS_EXCEPTION;

  os << "Mat(";

  if(false && (m->rows > 0 && m->cols > 0) && m->dims == 2) {
    os << m->rows << ", " << m->cols;
  } else {
    std::vector<std::string> sizeStrs = js_mat_dimensions(*m);
    os << "[" << join(sizeStrs.cbegin(), sizeStrs.cend(), ", ") << "]";
  }

  if(m->depth() == CV_8U || m->channels() > 1) {
    os << ", ";
    const char* tstr;
    switch(m->depth() & 7) {
      case CV_8U: tstr = "CV_8U"; break;
      case CV_8S: tstr = "CV_8S"; break;
      case CV_16U: tstr = "CV_16U"; break;
      case CV_16S: tstr = "CV_16S"; break;
      case CV_32S: tstr = "CV_32S"; break;
      case CV_32F: tstr = "CV_32F"; break;
      case CV_64F: tstr = "CV_64F"; break;
    }
    os << tstr << 'C' << m->channels() << ")" /*<< std::endl*/;
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
             << std::setw(m->type() == CV_8UC4 ? 8 : m->type() == CV_8UC1 ? 2 : 6) << m->at<uint32_t>(y, x);
      }
    }

    os << ']' /*<< std::endl*/;
  }
  os << ' ';
  os << (void*)m->elemSize();
  os << "x";
  os << (void*)m->total();
  os << " @";
  os << (void*)m->ptr();

  str = os.str();

  return JS_NewStringLen(ctx, str.data(), str.size());
}

static JSValue
js_mat_inspect(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  cv::Mat* m = js_mat_data(ctx, this_val);
  int x, y;

  std::ostringstream os;
  std::string str;
  int i = 0;
  if(!m)
    return JS_EXCEPTION;

  int bytes = 1 << ((m->type() & 0x7) >> 1);
  char sign = (m->type() & 0x7) >= 5 ? 'F' : (m->type() & 1) ? 'S' : 'U';

  std::vector<std::string> sizeStrs = js_mat_dimensions(*m);
  ;

  os << "Mat "
     /*     << "@ "
          << reinterpret_cast<void*>(reinterpret_cast<char*>(m)  )*/
     << " [ ";
  if(sizeStrs.size() || m->type()) {
    os << "size: \x1b[0;33m" << join(sizeStrs.cbegin(), sizeStrs.cend(), "\x1b[m*\x1b[0;33m") << ", ";
    os << "type: \x1b[0;33mCV_" << (bytes * 8) << sign << 'C' << m->channels() << "\x1b[m, ";
    os << "elemSize: \x1b[0;33m" << m->elemSize() << "\x1b[m, ";
    os << "total: \x1b[0;33m" << m->total() << "\x1b[m, ";
    os << "dims: \x1b[0;33m" << m->dims;
  } else {
    os << "empty";
  }
  if(m->u)
    os << ", refcount = " << m->u->refcount;
  os << " ]";
  str = os.str();
  return JS_NewStringLen(ctx, str.data(), str.size());
}

static JSValue
js_mat_getrotationmatrix2d(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData<double> s;

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

static JSValue
js_mat_convert_to(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSMatData *m, *output;
  int32_t rtype;
  double alpha = 1, beta = 0;

  m = js_mat_data(ctx, this_val);
  output = js_mat_data(ctx, argv[0]);

  if(m == nullptr || output == nullptr)
    return JS_EXCEPTION;

  JS_ToInt32(ctx, &rtype, argv[1]);

  if(argc >= 3)
    JS_ToFloat64(ctx, &alpha, argv[2]);

  if(argc >= 4)
    JS_ToFloat64(ctx, &beta, argv[3]);

  m->convertTo(*output, rtype, alpha, beta);

  return JS_UNDEFINED;
}

static JSValue
js_mat_copy_to(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSMatData *m = nullptr, *output = nullptr, *mask = nullptr;

  m = js_mat_data(ctx, this_val);
  output = js_mat_data(ctx, argv[0]);

  if(argc > 1)
    mask = js_mat_data(ctx, argv[1]);

  if(m == nullptr || output == nullptr)
    return JS_EXCEPTION;

  if(mask)
    m->copyTo(*output, *mask);
  else
    m->copyTo(*output);
  /*if(mask)
    m->copyTo(*output, *mask);
  else
    *output = *m;*/

  return JS_UNDEFINED;
}

static JSValue
js_mat_reshape(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSMatData *m, mat;
  int32_t cn, rows = 0;
  JSValue ret = JS_EXCEPTION;

  m = js_mat_data(ctx, this_val);

  if(m == nullptr || argc < 1)
    return ret;

  if(argc >= 1 && JS_IsNumber(argv[0])) {
    JS_ToInt32(ctx, &cn, argv[0]);
    argv++;
    argc--;
  } else {
    cn = m->channels();
  }

  if(argc >= 1) {
    std::vector<int> newshape;
    if(JS_IsArray(ctx, argv[0])) {
      js_array_to(ctx, argv[0], newshape);
      if(argc >= 2 && JS_IsNumber(argv[1])) {
        uint32_t ndims;
        JS_ToUint32(ctx, &ndims, argv[1]);
        if(ndims > newshape.size())
          return JS_EXCEPTION;
        mat = m->reshape(cn, ndims, &newshape[0]);
      } else {
        mat = m->reshape(cn, newshape);
      }
    } else if(JS_IsNumber(argv[0])) {
      JS_ToInt32(ctx, &rows, argv[0]);
      mat = m->reshape(cn, rows);
    }
    ret = js_mat_wrap(ctx, mat);
  }

  return ret;
}

static JSValue
js_mat_class_func(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValueConst *v = argv, *e = &argv[argc];
  JSMatData result;
  JSMatData *prev = nullptr, *mat = nullptr;

  while(v < e) {
    JSValueConst arg = *v++;

    if(nullptr == (mat = js_mat_data(ctx, arg)))
      return JS_EXCEPTION;

    if(prev) {
      JSMatData const &a = *prev, &b = *mat;
      switch(magic) {
        case 0: result = a + b; break;
        case 1: result = a - b; break;
        case 2: result = a * b; break;
        case 3: result = a / b; break;
        case 4: result = a & b; break;
        case 5: result = a | b; break;
        case 6: result = a ^ b; break;
      }
      prev = &result;
    } else {
      prev = mat;
      result = cv::Mat::zeros(mat->rows, mat->cols, mat->type());
      mat->copyTo(result);
    }
  }

  return js_mat_wrap(ctx, result);
}

static JSValue
js_mat_fill(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSMatData* m;

  m = js_mat_data(ctx, this_val);

  if(m == nullptr)
    return JS_EXCEPTION;

  if(m->empty() || m->rows == 0 || m->cols == 0)
    return JS_EXCEPTION;

  cv::Mat& mat = *m;

  switch(magic) {
    case 0: {
      mat = cv::Mat::zeros(mat.rows, mat.cols, mat.type());
      break;
    }
    case 1: {
      mat = cv::Mat::ones(mat.rows, mat.cols, mat.type());
      break;
    }
  }

  return JS_DupValue(ctx, this_val);
}

static JSValue
js_mat_class_create(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValue ret = JS_UNDEFINED;

  const auto& [size, type] = js_mat_params(ctx, argc, argv);

  if(size.width == 0 || size.height == 0)
    return JS_EXCEPTION;

  ret = js_mat_new(ctx, uint32_t(0), uint32_t(0), int(0));
  JSMatData& mat = *js_mat_data(ctx, ret);

  switch(magic) {
    case 0: {
      mat = cv::Scalar::all(0);
      break;
    }
    case 1: {
      mat = cv::Scalar::all(1);
      break;
    }
  }

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
js_mat_iterator_new(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValue enum_obj, mat;
  JSMatIteratorData* it;
  mat = JS_DupValue(ctx, this_val);
  if(!JS_IsException(mat)) {
    enum_obj = JS_NewObjectProtoClass(ctx, mat_iterator_proto, js_mat_iterator_class_id);
    if(!JS_IsException(enum_obj)) {
      it = js_allocate<JSMatIteratorData>(ctx);

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
  JSMatIteratorData* it = static_cast<JSMatIteratorData*>(JS_GetOpaque(val, js_mat_iterator_class_id));
  js_deallocate(rt, it);
}

static JSValue
js_mat_iterator_dup(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  return JS_DupValue(ctx, this_val);
}

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
    JS_CGETSET_MAGIC_DEF("size", js_mat_get_props, NULL, 7),
    JS_CGETSET_MAGIC_DEF("continuous", js_mat_get_props, NULL, 8),
    JS_CGETSET_MAGIC_DEF("submatrix", js_mat_get_props, NULL, 9),
    JS_CGETSET_MAGIC_DEF("step", js_mat_get_props, NULL, 10),
    JS_CGETSET_MAGIC_DEF("elemSize", js_mat_get_props, NULL, 11),
    JS_CGETSET_MAGIC_DEF("elemSize1", js_mat_get_props, NULL, 12),
    JS_CGETSET_MAGIC_DEF("buffer", js_mat_get_props, NULL, 13),
    JS_CGETSET_MAGIC_DEF("array", js_mat_get_props, NULL, 14),
    JS_CFUNC_MAGIC_DEF("col", 1, js_mat_funcs, 0),
    JS_CFUNC_MAGIC_DEF("row", 1, js_mat_funcs, 1),
    JS_CFUNC_MAGIC_DEF("colRange", 2, js_mat_funcs, 2),
    JS_CFUNC_MAGIC_DEF("rowRange", 2, js_mat_funcs, 3),
    JS_CFUNC_MAGIC_DEF("clone", 0, js_mat_funcs, 5),
    JS_CFUNC_MAGIC_DEF("roi", 0, js_mat_funcs, 6),
    JS_CFUNC_MAGIC_DEF("release", 0, js_mat_funcs, 8),
    JS_CFUNC_MAGIC_DEF("dup", 0, js_mat_funcs, 9),
    JS_CFUNC_MAGIC_DEF("clear", 0, js_mat_funcs, 10),
    JS_CFUNC_MAGIC_DEF("reset", 0, js_mat_funcs, 11),
    JS_CFUNC_MAGIC_DEF("resize", 1, js_mat_funcs, 12),
    JS_CFUNC_MAGIC_DEF("step1", 0, js_mat_funcs, 13),
    JS_CFUNC_MAGIC_DEF("locateROI", 0, js_mat_funcs, 14),

    JS_CFUNC_MAGIC_DEF("and", 2, js_mat_expr, MAT_EXPR_AND),
    JS_CFUNC_MAGIC_DEF("or", 2, js_mat_expr, MAT_EXPR_OR),
    JS_CFUNC_MAGIC_DEF("xor", 3, js_mat_expr, MAT_EXPR_XOR),
    JS_CFUNC_MAGIC_DEF("mul", 3, js_mat_expr, MAT_EXPR_MUL),

    JS_CFUNC_MAGIC_DEF("zero", 2, js_mat_fill, 0),
    JS_CFUNC_MAGIC_DEF("one", 2, js_mat_fill, 1),

    JS_CFUNC_DEF("toString", 0, js_mat_tostring),
    JS_CFUNC_DEF("inspect", 0, js_mat_inspect),
    JS_CFUNC_DEF("at", 1, js_mat_at),
    JS_CFUNC_DEF("set", 2, js_mat_set),
    JS_CFUNC_DEF("setTo", 0, js_mat_set_to),
    JS_CFUNC_DEF("convertTo", 2, js_mat_convert_to),
    JS_CFUNC_DEF("copyTo", 1, js_mat_copy_to),
    JS_CFUNC_DEF("reshape", 1, js_mat_reshape),
    JS_CFUNC_MAGIC_DEF("keys", 0, js_mat_iterator_new, 0),
    JS_CFUNC_MAGIC_DEF("values", 0, js_mat_iterator_new, 1),
    JS_CFUNC_MAGIC_DEF("entries", 0, js_mat_iterator_new, 2),
    JS_ALIAS_DEF("[Symbol.iterator]", "entries"),
    JS_ALIAS_DEF("[Symbol.toPrimitive]", "toString"),

    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "Mat", JS_PROP_CONFIGURABLE)

};

const JSCFunctionListEntry js_mat_iterator_proto_funcs[] = {
    JS_ITERATOR_NEXT_DEF("next", 0, js_mat_iterator_next, 0),
    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "MatIterator", JS_PROP_CONFIGURABLE),
    JS_CFUNC_DEF("[Symbol.iterator]", 0, js_mat_iterator_dup),

};

const JSCFunctionListEntry js_mat_static_funcs[] = {
    JS_CFUNC_DEF("getRotationMatrix2D", 3, js_mat_getrotationmatrix2d),
    JS_CFUNC_MAGIC_DEF("add", 2, js_mat_class_func, 0),
    JS_CFUNC_MAGIC_DEF("sub", 2, js_mat_class_func, 1),
    JS_CFUNC_MAGIC_DEF("mul", 2, js_mat_class_func, 2),
    JS_CFUNC_MAGIC_DEF("div", 2, js_mat_class_func, 3),
    JS_CFUNC_MAGIC_DEF("and", 2, js_mat_class_func, 4),
    JS_CFUNC_MAGIC_DEF("or", 2, js_mat_class_func, 5),
    JS_CFUNC_MAGIC_DEF("xor", 3, js_mat_class_func, 6),
    JS_CFUNC_MAGIC_DEF("zeros", 1, js_mat_class_create, 0),
    JS_CFUNC_MAGIC_DEF("ones", 1, js_mat_class_create, 1),
    JS_PROP_INT32_DEF("CV_8U", CV_MAKETYPE(CV_8U, 1), JS_PROP_ENUMERABLE),
};

int
js_mat_init(JSContext* ctx, JSModuleDef* m) {
  if(js_mat_class_id == 0) {
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

    JS_FreeValue(ctx, g);
  }

  if(m)
    JS_SetModuleExport(ctx, m, "Mat", mat_class);
  return 0;
}

extern "C" VISIBLE JSModuleDef*
JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_mat_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "Mat");
  return m;
}
