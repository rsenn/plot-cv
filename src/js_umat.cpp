#include "jsbindings.hpp"
#include "js_umat.hpp"
#include "js_size.hpp"
#include "js_point.hpp"
#include "js_rect.hpp"
#include "js_array.hpp"
#include "js_typed_array.hpp"
#include "js_alloc.hpp"
#include "geometry.hpp"
#include "util.hpp"
#include "../quickjs/cutils.h"
#include <list>
#include <map>
#include <fstream>

#if defined(JS_UMAT_MODULE) || defined(quickjs_umat_EXPORTS)
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module
#else
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module_umat
#endif

enum { UMAT_EXPR_AND = 0, UMAT_EXPR_OR, UMAT_EXPR_XOR, UMAT_EXPR_MUL };
enum { UMAT_ITERATOR_KEYS, UMAT_ITERATOR_VALUES, UMAT_ITERATOR_ENTRIES };
extern "C" {
JSValue umat_proto = JS_UNDEFINED, umat_class = JS_UNDEFINED, umat_iterator_proto = JS_UNDEFINED, umat_iterator_class = JS_UNDEFINED;
JSClassID js_umat_class_id = 0, js_umat_iterator_class_id = 0;

static void
js_umat_free_func(JSRuntime* rt, void* opaque, void* ptr) {
  static_cast<JSUMatData*>(opaque)->release();
}
}

static std::vector<JSUMatData*> umat_list;
static std::list<JSUMatData*> umat_freed;

typedef struct JSUMatIteratorData {
  JSValue obj, buf;
  uint32_t row, col;
  int magic;
} JSUMatIteratorData;

VISIBLE JSUMatData*
js_umat_data(JSContext* ctx, JSValueConst val) {
  return static_cast<JSUMatData*>(JS_GetOpaque2(ctx, val, js_umat_class_id));
}

static inline std::vector<int>
js_umat_sizes(const JSUMatData& umat) {
  const cv::UMatSize size(umat.size);
  std::vector<int> sizes;
  if(umat.dims == 2) {
    sizes.push_back(umat.rows);
    sizes.push_back(umat.cols);
  } else {
    std::copy(&size[0], &size[size.dims()], std::back_inserter(sizes));
  }
  return sizes;
}

static inline std::vector<std::string>
js_umat_dimensions(const JSUMatData& umat) {
  std::vector<int> sizes = js_umat_sizes(umat);
  std::vector<std::string> dimensions;

  std::transform(sizes.cbegin(), sizes.cend(), std::back_inserter(dimensions), static_cast<std::string (*)(int)>(&std::to_string));
  return dimensions;
}

static inline JSUMatData*
js_umat_track(JSContext* ctx, JSUMatData* s) {
  std::vector<cv::UMat*> deallocate;

  for(;;) {
    auto it2 = std::find(umat_freed.cbegin(), umat_freed.cend(), s);
    if(it2 != umat_freed.cend()) {
      deallocate.push_back(s);

      // std::cerr << "allocated @" << static_cast<void*>(s) << " which is in free list" << std::endl;

      // umat_freed.erase(it2);
      s = js_allocate<cv::UMat>(ctx);
      memcpy(s, deallocate[deallocate.size() - 1], sizeof(JSUMatData));

    } else {
      break;
    }
  }

  umat_list.push_back(s);

  for(const auto& ptr : deallocate) js_deallocate(ctx, ptr);
  return s;
}

#ifdef DEBUG_UMAT
static inline std::map<void*, std::vector<JSUMatData*>>
js_umat_data(void* data = nullptr) {
  std::map<void*, std::vector<JSUMatData*>> ret;
  for(auto umat : umat_list) {
    const auto u = umat->u;
    if(u != nullptr && (data == nullptr || u == data)) {
      void* data = u;
      if(ret.find(data) == ret.end()) {
        std::vector<JSUMatData*> v{umat};
        ret.insert({data, v});
      } else {
        ret[data].push_back(umat);
      }
    }
  }
  return ret;
}

static inline void
js_umat_print_data(const std::map<void*, std::vector<JSUMatData*>>& data, size_t minSize = 1) {

  for(const auto& [key, value] : data) {

    if(value.size() >= minSize) {
      std::cerr << "data @" << key << " =";

      for(const auto& ptr : value) {
        size_t refcount = ptr->u ? ptr->u->refcount : 0;
        std::cerr << " umat @" << static_cast<void*>(ptr);
        if(refcount > 1)
          std::cerr << " (refcount=" << refcount << ")";
      }
      std::cerr << std::endl;
    }
  }
}

static inline void
js_umat_dump(JSUMatData* const s) {
  auto posList = std::find(umat_list.cbegin(), umat_list.cend(), s);
  bool inList = posList != umat_list.cend();
  bool inFreed = std::find(umat_freed.cbegin(), umat_freed.cend(), s) != umat_freed.cend();
  const auto u = s->u;
  std::cerr << " umat"
            << "[" << (posList - umat_list.cbegin()) << "]=" << static_cast<void*>(s);

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
js_umat_new(JSContext* ctx, uint32_t rows, uint32_t cols, int type) {
  JSValue ret;
  JSUMatData* s;

  if(JS_IsUndefined(umat_proto))
    js_umat_init(ctx, NULL);

  ret = JS_NewObjectProtoClass(ctx, umat_proto, js_umat_class_id);

  s = js_umat_track(ctx, js_allocate<cv::UMat>(ctx));

  if(cols || rows || type) {
    new(s) cv::UMat(rows, cols, type);
    // *s = cv::UMat::zeros(rows, cols,  type);
  } else {
    new(s) cv::UMat();
  }

  // s->addref();
#ifdef DEBUG_UMAT
  std::cerr << ((cols > 0 || rows > 0) ? "js_umat_new (h,w)" : "js_umat_new      ");
  js_umat_dump(s);
  std::cerr << std::endl;
#endif

  JS_SetOpaque(ret, s);
  return ret;
}

VISIBLE JSValue
js_umat_wrap(JSContext* ctx, const cv::UMat& umat) {
  JSValue ret;
  JSUMatData* s;
  ret = JS_NewObjectProtoClass(ctx, umat_proto, js_umat_class_id);

  s = js_umat_track(ctx, js_allocate<cv::UMat>(ctx));
  new(s) cv::UMat();
  *s = umat;
#ifdef DEBUG_UMAT
  std::cerr << "js_umat_wrap     ";
  auto posList = std::find(umat_list.cbegin(), umat_list.cend(), const_cast<JSUMatData*>(&umat));
  bool inList;
  if((inList = posList != umat_list.cend())) {
    std::cerr << "arg[" << (posList - umat_list.cbegin()) << "]=" << static_cast<const void*>(&umat);
    std::cerr << ", inList(arg)=" << (inList ? "true" : "false");
  } else {
    std::cerr << "arg=" << static_cast<const void*>(&umat);
  }
  js_umat_dump(s);
  std::cerr << std::endl;
#endif

  JS_SetOpaque(ret, s);
  return ret;
}

static std::pair<JSSizeData<uint32_t>, int>
js_umat_params(JSContext* ctx, int argc, JSValueConst* argv) {
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
js_umat_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {

  const auto& [size, type] = js_umat_params(ctx, argc, argv);

  return js_umat_new(ctx, size.height, size.width, type);
}

void
js_umat_finalizer(JSRuntime* rt, JSValue val) {
  JSUMatData* s = static_cast<JSUMatData*>(JS_GetOpaque(val, js_umat_class_id));

  auto it2 = std::find(umat_freed.cbegin(), umat_freed.cend(), s);
  auto it = std::find(umat_list.cbegin(), umat_list.cend(), s);

  if(it2 != umat_freed.cend()) {
#ifdef DEBUG_UMAT
    std::cerr << "js_umat_finalizer";
    js_umat_dump(s);
#endif

    std::cerr << " ERROR: already freed" << std::endl;
    return;
  }
  JS_FreeValueRT(rt, val);
  if(it != umat_list.cend()) {
    size_t refcount = s->u != nullptr ? s->u->refcount : 0;
    if(s->u) {
      auto data = s->u;
#ifdef DEBUG_UMAT
      std::cerr << "cv::UMat::release";
      std::cerr << " umat=" << static_cast<void*>(s);
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
      umat_list.erase(it);
      umat_freed.push_front(s);
    }
#ifdef DEBUG_UMAT
    std::cerr << "js_umat_finalizer";
    js_umat_dump(s);
    std::cerr << ", refcount=" << refcount;
    std::cerr << ", umat_list.size()=" << umat_list.size();
    std::cerr << ", umat_freed.size()=" << umat_freed.size() << std::endl;
#endif
  } else {
#ifdef DEBUG_UMAT
    std::cerr << "js_umat_finalizer";
    js_umat_dump(s);
#endif
    std::cerr << " ERROR: not found" << std::endl;
  }

  js_deallocate(rt, s);
}

static JSValue
js_umat_funcs(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValue ret = JS_UNDEFINED;
  int64_t i = -1, i2 = -1;
  JSPointData<double> pt;
  JSUMatData* m = js_umat_data(ctx, this_val);

  if(argc > 0) {
    JS_ToInt64(ctx, &i, argv[0]);
    pt = js_point_get(ctx, argv[0]);
    if(argc > 1) {
      JS_ToInt64(ctx, &i2, argv[1]);
    }
  }

  if(magic == 0)
    ret = js_umat_wrap(ctx, m->col(i));
  else if(magic == 1)
    ret = js_umat_wrap(ctx, m->row(i));
  else if(magic == 2)
    ret = js_umat_wrap(ctx, m->colRange(i, i2));
  else if(magic == 3)
    ret = js_umat_wrap(ctx, m->rowRange(i, i2));
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
    ret = js_umat_wrap(ctx, m->clone());
  } else if(magic == 6) {
    JSRectData<double> rect = {0, 0, 0, 0};

    if(argc > 0)
      rect = js_rect_get(ctx, argv[0]);

    ret = js_umat_wrap(ctx, (*m)(rect));

  } else if(magic == 7) {
    JSRectData<double> rect = {0, 0, 0, 0};

    if(argc > 0)
      rect = js_rect_get(ctx, argv[0]);

    ret = js_umat_wrap(ctx, (*m)(rect));

  } else if(magic == 8) {
    m->release();
  } else if(magic == 9) {
    ret = js_umat_wrap(ctx, *m);

  } else if(magic == 10) {
    *m = cv::UMat::zeros(m->rows, m->cols, m->type());
  } else if(magic == 11) {
    *m = cv::UMat();
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
js_umat_expr(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValue ret = JS_UNDEFINED;
  JSColorData<double> color;
  JSUMatData *src = nullptr, *dst = nullptr, *o = nullptr;
  double scale = 1.0;

  if((src = js_umat_data(ctx, this_val)) == nullptr)
    return JS_EXCEPTION;

  cv::UMat& umat = *src;

  if(argc < 1)
    return JS_EXCEPTION;

  if((o = js_umat_data(ctx, argv[0])) == nullptr)
    if(!js_color_read(ctx, argv[0], &color))
      return JS_EXCEPTION;

  if(magic == 3 && argc > 1) {
    JS_ToFloat64(ctx, &scale, argv[1]);
    argv++;
    argc--;
  }

  if(argc > 1) {
    dst = js_umat_data(ctx, argv[1]);
  }

  if(dst == nullptr)
    //  return JS_UNDEFINED;
    dst = src;

  //  cv::UMat& out = *dst;
  cv::UMatExpr expr;

  if(o == nullptr) {
    cv::Scalar& scalar = *reinterpret_cast<cv::Scalar*>(&color);
    switch(magic) {
      case UMAT_EXPR_AND: expr = umat & scalar; break;
      case UMAT_EXPR_OR: expr = umat | scalar; break;
      case UMAT_EXPR_XOR: expr = umat ^ scalar; break;
      case UMAT_EXPR_MUL: expr = umat.mul(scalar, scale); break;
    }
    *dst = static_cast<cv::UMat>(expr);

  } else {
    // cv::UMat const& other = *o;

    switch(magic) {
      case UMAT_EXPR_AND:
        expr = (*src) & (*o); /*cv::bitwise_and(*src, *o, *dst);*/
        break;
      case UMAT_EXPR_OR:
        expr = (*src) | (*o); /*cv::bitwise_or(*src, *o, *dst);*/
        break;
      case UMAT_EXPR_XOR:
        expr = (*src) ^ (*o); /*cv::bitwise_xor(*src, *o, *dst);*/
        break;
      case UMAT_EXPR_MUL:
        expr = (*src) * (*o); /**dst = umat.mul(*o, scale);*/
        break;
    }

    *dst = static_cast<cv::UMat>(expr);
  }

  return ret;
}

static JSValue
js_umat_init(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValue ret = JS_UNDEFINED;
  JSUMatData* m;

  auto [size, type] = js_umat_params(ctx, argc, argv);

  if((m = js_umat_data(ctx, this_val))) {
    if(size.width == 0 || size.height == 0) {
      if(m->rows && m->cols) {
        size.width = m->cols;
        size.height = m->rows;
        type = m->type();
      }
    }
    ret = JS_DupValue(ctx, this_val);
  } else if(size.width > 0 && size.height > 0) {
    ret = js_umat_new(ctx, size.height, size.width, type);

    m = js_umat_data(ctx, ret);
  } else {
    return JS_EXCEPTION;
  }

  cv::UMat& umat = *m;

  switch(magic) {
    case 0: {
      umat = cv::UMat::zeros(size, type);
      break;
    }
    case 1: {
      umat = cv::UMat::ones(size, type);
      break;
    }
  }

  return ret;
}

template<class T>
void
js_umat_get(JSContext* ctx, JSValueConst this_val, uint32_t row, uint32_t col, T& value) {
  cv::UMat* m = js_umat_data(ctx, this_val);

  if(m)
    value = (*m).at<T>(row, col);
  else
    value = T();
}

static JSValue
js_umat_get(JSContext* ctx, JSValueConst this_val, uint32_t row, uint32_t col) {
  JSValue ret = JS_EXCEPTION;
  cv::UMat* m = js_umat_data(ctx, this_val);

  if(m) {
    uint32_t bytes = (1 << m->depth()) * m->channels();
    size_t channels = umat_channels(*m);

    if(channels == 1) {
      switch(m->type()) {

        case CV_8UC1: {
          uint8_t value;
          js_umat_get(ctx, this_val, row, col, value);
          ret = JS_NewUint32(ctx, value);
          break;
        }

        case CV_16UC1: {
          uint16_t value;
          js_umat_get(ctx, this_val, row, col, value);
          ret = JS_NewUint32(ctx, value);
          break;
        }
        case CV_32SC1: {
          int32_t value;
          js_umat_get(ctx, this_val, row, col, value);
          ret = JS_NewInt32(ctx, value);
          break;
        }
        case CV_32FC1: {
          float value;
          js_umat_get(ctx, this_val, row, col, value);
          ret = JS_NewFloat64(ctx, value);
          break;
        }
        case CV_64FC1: {
          double value;
          js_umat_get(ctx, this_val, row, col, value);
          ret = JS_NewFloat64(ctx, value);
          break;
        }
        default: {
          ret = JS_ThrowTypeError(ctx, "Invalid Mat type %u", m->type());
          break;
        }
      }

      return ret;
    }
  }
  return JS_UNDEFINED;
}

static int
js_umat_get_wh(JSContext* ctx, JSUMatSizeData* size, JSValueConst obj) {
  cv::UMat* m = js_umat_data(ctx, obj);

  if(m) {
    size->rows = m->rows;
    size->cols = m->cols;
    return 1;
  }
  return 0;
}

static JSValue
js_umat_at(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSUMatData* m = js_umat_data(ctx, this_val);
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
    JSUMatSizeData dim = {static_cast<uint32_t>(m->rows), static_cast<uint32_t>(m->cols)};
    uint32_t idx;

    JS_ToUint32(ctx, &idx, argv[0]);
    row = idx / dim.cols;
    col = idx % dim.cols;
  }

  return js_umat_get(ctx, this_val, row, col);
}

static JSValue
js_umat_set(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  cv::UMat* m = js_umat_data(ctx, this_val);
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
    return JS_UNDEFINED;
  return JS_UNDEFINED;
}

template<class T>
typename std::enable_if<std::is_integral<T>::value, void>::type
js_umat_vector_get(JSContext* ctx, int argc, JSValueConst* argv, std::vector<T>& output, std::vector<bool>& defined) {
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
js_umat_vector_get(JSContext* ctx, int argc, JSValueConst* argv, std::vector<T>& output, std::vector<bool>& defined) {
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
js_umat_vector_get(JSContext* ctx, int argc, JSValueConst* argv, std::vector<T>& output, std::vector<bool>& defined) {
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
js_umat_set_vector(JSContext* ctx, JSUMatData* m, int argc, JSValueConst* argv) {
  JSUMatSizeData dim = {static_cast<uint32_t>(m->rows), static_cast<uint32_t>(m->cols)};
  uint32_t idx;
  std::vector<bool> defined;
  std::vector<T> v;
  js_umat_vector_get(ctx, argc, argv, v, defined);

  for(idx = 0; idx < v.size(); idx++)
    if(defined[idx])
      m->at<T>(idx / dim.cols, idx % dim.cols) = v[idx];
  return v;
}

static JSValue
js_umat_set_to(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSUMatData* m = js_umat_data(ctx, this_val);
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
      js_umat_set_vector<cv::Vec<uint16_t, 2>>(ctx, m, argc, argv);
    else if(m->channels() == 3)
      js_umat_set_vector<cv::Vec<uint16_t, 3>>(ctx, m, argc, argv);
    else if(m->channels() == 4)
      js_umat_set_vector<cv::Vec<uint16_t, 4>>(ctx, m, argc, argv);
  } else if(m->depth() == CV_32F) {
    if(m->channels() == 1)
      js_umat_set_vector<float>(ctx, m, argc, argv);
  } else if(bytes <= sizeof(uint)) {
    if(bytes <= 1) {
      std::vector<uint8_t> v;
      js_umat_vector_get(ctx, argc, argv, v, defined);
      m->setTo(cv::InputArray(v), defined);
    } else if(bytes <= 2) {
      std::vector<uint16_t> v;
      js_umat_vector_get(ctx, argc, argv, v, defined);
      m->setTo(cv::InputArray(v), defined);
    } else if(bytes <= 4) {
      js_umat_set_vector<uint32_t>(ctx, m, argc, argv);
    } else if(bytes <= 8) {
      js_umat_set_vector<uint64_t>(ctx, m, argc, argv);
    }
  }

  return JS_UNDEFINED;
}

static JSValue
js_umat_get_props(JSContext* ctx, JSValueConst this_val, int magic) {
  cv::UMat* m = js_umat_data(ctx, this_val);
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

  return JS_UNDEFINED;
}

static JSValue
js_umat_tostring(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  cv::UMat* m = js_umat_data(ctx, this_val);
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
    std::vector<std::string> sizeStrs = js_umat_dimensions(*m);
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
          os << std::setfill('0') << std::setbase(16) << std::setw(m->type() == CV_8UC4 ? 8 : m->type() == CV_8UC1 ? 2 : 6)
             << m->at<uint32_t>(y, x);
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
js_umat_inspect(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  cv::UMat* m = js_umat_data(ctx, this_val);
  int x, y;

  std::ostringstream os;
  std::string str;
  int i = 0;
  if(!m)
    return JS_EXCEPTION;

  int bytes = 1 << ((m->type() & 0x7) >> 1);
  char sign = (m->type() & 0x7) >= 5 ? 'F' : (m->type() & 1) ? 'S' : 'U';

  std::vector<std::string> sizeStrs = js_umat_dimensions(*m);
  ;

  os << "Mat "
     /*     << "@ "
          << reinterpret_cast<void*>(reinterpret_cast<char*>(m)  )*/
     << " [ ";
  if(sizeStrs.size() || m->type()) {
    os << "size: " COLOR_YELLOW "" << join(sizeStrs.cbegin(), sizeStrs.cend(), "" COLOR_NONE "*" COLOR_YELLOW "") << "" COLOR_NONE ", ";
    os << "type: " COLOR_YELLOW "CV_" << (bytes * 8) << sign << 'C' << m->channels() << "" COLOR_NONE ", ";
    os << "elemSize: " COLOR_YELLOW "" << m->elemSize() << "" COLOR_NONE ", ";
    os << "elemSize1: " COLOR_YELLOW "" << m->elemSize1() << "" COLOR_NONE ", ";
    os << "total: " COLOR_YELLOW "" << m->total() << "" COLOR_NONE ", ";
    os << "dims: " COLOR_YELLOW "" << m->dims << "" COLOR_NONE "";
  } else {
    os << "empty";
  }
  if(m->u)
    os << ", refcount: " COLOR_YELLOW "" << m->u->refcount;
  os << "" COLOR_NONE " ]";
  str = os.str();
  return JS_NewStringLen(ctx, str.data(), str.size());
}

static JSValue
js_mat_getrotationmatrix2d(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSPointData<double> s;

  double angle = 0, scale = 1;
  cv::UMat m;

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

  ret = js_umat_wrap(ctx, m);
  return ret;
}

static JSValue
js_umat_convert_to(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSUMatData *m, *output;
  int32_t rtype;
  double alpha = 1, beta = 0;

  m = js_umat_data(ctx, this_val);
  output = js_umat_data(ctx, argv[0]);

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
js_umat_copy_to(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSUMatData *m = nullptr, *output = nullptr, *mask = nullptr;

  m = js_umat_data(ctx, this_val);
  output = js_umat_data(ctx, argv[0]);

  if(argc > 1)
    mask = js_umat_data(ctx, argv[1]);

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
js_umat_reshape(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSUMatData *m, umat;
  int32_t cn, rows = 0;
  JSValue ret = JS_EXCEPTION;

  m = js_umat_data(ctx, this_val);

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
        umat = m->reshape(cn, ndims, &newshape[0]);
      } else {
        umat = m->reshape(cn, newshape);
      }
    } else if(JS_IsNumber(argv[0])) {
      JS_ToInt32(ctx, &rows, argv[0]);
      umat = m->reshape(cn, rows);
    }
    ret = js_umat_wrap(ctx, umat);
  }

  return ret;
}

static JSValue
js_umat_class_func(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValueConst *v = argv, *e = &argv[argc];
  JSUMatData result;
  JSUMatData *prev = nullptr, *umat = nullptr;

  while(v < e) {
    JSValueConst arg = *v++;

    if(nullptr == (umat = js_umat_data(ctx, arg)))
      return JS_EXCEPTION;

    if(prev) {
      JSUMatData const &a = *prev, &b = *umat;
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
      prev = umat;
      result = cv::UMat::zeros(umat->rows, umat->cols, umat->type());
      umat->copyTo(result);
    }
  }

  return js_umat_wrap(ctx, result);
}

static JSValue
js_umat_fill(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSUMatData* m;

  m = js_umat_data(ctx, this_val);

  if(m == nullptr)
    return JS_EXCEPTION;

  if(m->empty() || m->rows == 0 || m->cols == 0)
    return JS_EXCEPTION;

  cv::UMat& umat = *m;

  switch(magic) {
    case 0: {
      umat = cv::UMat::zeros(umat.rows, umat.cols, umat.type());
      break;
    }
    case 1: {
      umat = cv::UMat::ones(umat.rows, umat.cols, umat.type());
      break;
    }
  }

  return JS_DupValue(ctx, this_val);
}

static JSValue
js_umat_class_create(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValue ret = JS_UNDEFINED;

  const auto& [size, type] = js_umat_params(ctx, argc, argv);

  if(size.width == 0 || size.height == 0)
    return JS_EXCEPTION;

  ret = js_umat_new(ctx, uint32_t(0), uint32_t(0), int(0));
  JSUMatData& umat = *js_umat_data(ctx, ret);

  switch(magic) {
    case 0: {
      umat = cv::Scalar::all(0);
      break;
    }
    case 1: {
      umat = cv::Scalar::all(1);
      break;
    }
  }

  return ret;
}

/*static JSValue
js_umat_create_vec(JSContext* ctx, int len, JSValue* vec) {
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
}*/

static JSValue
js_umat_buffer(JSContext* ctx, JSValueConst this_val) {
  JSUMatData* m;
  cv::UMatData* data;
  size_t size;
  uint8_t* ptr;

  if((m = js_umat_data(ctx, this_val)) == nullptr)
    return JS_EXCEPTION;

data = m->u;
  ptr = data ? data->data : nullptr;
  size = data ? data->size : 0;

  if(ptr == nullptr)
    return JS_NULL;

  m->addref();
  // m->addref();

  return JS_NewArrayBuffer(ctx, ptr, size, &js_umat_free_func, m, FALSE /*TRUE*/);
}

static JSValue
js_umat_array(JSContext* ctx, JSValueConst this_val) {
  JSUMatData* m;
  JSValueConst global, typed_array, buffer;
  int elem_size;
  const char* ctor;

  if((m = js_umat_data(ctx, this_val)) == nullptr)
    return JS_EXCEPTION;

  elem_size = m->elemSize();

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
    default: JS_ThrowTypeError(ctx, "cv:Mat type=%02x channels=%02x", m->type(), m->channels()); return JS_EXCEPTION;
  }
  typed_array = JS_GetPropertyStr(ctx, global, ctor);
  buffer = js_umat_get_props(ctx, this_val, 9);

  return JS_CallConstructor(ctx, typed_array, 1, &buffer);
}

JSValue
js_umat_iterator_new(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSValue enum_obj, umat;
  JSUMatIteratorData* it;
  umat = JS_DupValue(ctx, this_val);
  if(!JS_IsException(umat)) {
    enum_obj = JS_NewObjectProtoClass(ctx, umat_iterator_proto, js_umat_iterator_class_id);
    if(!JS_IsException(enum_obj)) {
      it = js_allocate<JSUMatIteratorData>(ctx);

      it->obj = umat;
      it->buf = js_umat_buffer(ctx, this_val);
      it->row = 0;
      it->col = 0;
      it->magic = magic;

      JS_SetOpaque(/*ctx, */ enum_obj, it);
      return enum_obj;
    }
    JS_FreeValue(ctx, enum_obj);
  }
  JS_FreeValue(ctx, umat);
  return JS_EXCEPTION;
}

JSValue
js_umat_iterator_next(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, BOOL* pdone, int magic) {
  JSUMatIteratorData* it;
  JSValue ret = JS_UNDEFINED;

  *pdone = FALSE;

  it = static_cast<JSUMatIteratorData*>(JS_GetOpaque(this_val, js_umat_iterator_class_id));
  if(it) {
    JSUMatData* m;
    uint32_t row, col;
    size_t offset, channels;
    JSMatSizeData dim;

    if((m = js_umat_data(ctx, it->obj)) == nullptr)
      return JS_EXCEPTION;

    dim = umat_size(*m);

    /*if(!JS_IsUndefined(it->obj)) {
      if(js_umat_get_wh(ctx, &dim, it->obj))*/
    row = it->row;
    col = it->col;
    if(row >= m->rows) {
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

    channels = umat_channels(*m);
    offset = umat_offset(*m, row, col);

    // printf("channels=%zx row=%u col=%u dim.rows=%u dim.cols=%u\n", channels, row, col, dim.rows,
    // dim.cols);

    switch(it->magic) {
      case UMAT_ITERATOR_KEYS: {
        std::array<uint32_t, 2> pos = {row, col};
        ret = js_array_from(ctx, pos);
        break;
      }

      case UMAT_ITERATOR_VALUES: {

        if(channels == 1)
          return js_umat_get(ctx, it->obj, row, col);

        ret = js_typedarray_new(ctx, it->buf, offset, channels, TypedArrayProps(*m));
        break;
      }

      case UMAT_ITERATOR_ENTRIES: {
        JSValue value =
            channels == 1 ? js_umat_get(ctx, it->obj, row, col) : js_typedarray_new(ctx, it->buf, offset, channels, TypedArrayProps(*m));
        std::array<uint32_t, 2> pos = {row, col};
        std::array<JSValue, 2> entry = {js_array_from(ctx, pos), value};

        ret = js_array_from(ctx, entry);
        break;
      }
    }
    /* }*/
  }
  return ret;
}

void
js_umat_iterator_finalizer(JSRuntime* rt, JSValue val) {
  JSUMatIteratorData* it = static_cast<JSUMatIteratorData*>(JS_GetOpaque(val, js_umat_iterator_class_id));
  js_deallocate(rt, it);
}

static JSValue
js_umat_iterator_dup(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  return JS_DupValue(ctx, this_val);
}

JSClassDef js_umat_class = {
    .class_name = "Mat",
    .finalizer = js_umat_finalizer,
};

JSClassDef js_umat_iterator_class = {
    .class_name = "UMatIterator",
    .finalizer = js_umat_iterator_finalizer,
};

const JSCFunctionListEntry js_umat_proto_funcs[] = {JS_CGETSET_MAGIC_DEF("cols", js_umat_get_props, NULL, 0),
                                                   JS_CGETSET_MAGIC_DEF("rows", js_umat_get_props, NULL, 1),
                                                   JS_CGETSET_MAGIC_DEF("channels", js_umat_get_props, NULL, 2),
                                                   JS_CGETSET_MAGIC_DEF("type", js_umat_get_props, NULL, 3),
                                                   JS_CGETSET_MAGIC_DEF("depth", js_umat_get_props, NULL, 4),
                                                   JS_CGETSET_MAGIC_DEF("empty", js_umat_get_props, NULL, 5),
                                                   JS_CGETSET_MAGIC_DEF("total", js_umat_get_props, NULL, 6),
                                                   JS_CGETSET_MAGIC_DEF("size", js_umat_get_props, NULL, 7),
                                                   JS_CGETSET_MAGIC_DEF("continuous", js_umat_get_props, NULL, 8),
                                                   JS_CGETSET_MAGIC_DEF("submatrix", js_umat_get_props, NULL, 9),
                                                   JS_CGETSET_MAGIC_DEF("step", js_umat_get_props, NULL, 10),
                                                   JS_CGETSET_MAGIC_DEF("elemSize", js_umat_get_props, NULL, 11),
                                                   JS_CGETSET_MAGIC_DEF("elemSize1", js_umat_get_props, NULL, 12),
                                                   JS_CGETSET_DEF("buffer", js_umat_buffer, NULL),
                                                   JS_CGETSET_DEF("array", js_umat_array, NULL),
                                                   JS_CFUNC_MAGIC_DEF("col", 1, js_umat_funcs, 0),
                                                   JS_CFUNC_MAGIC_DEF("row", 1, js_umat_funcs, 1),
                                                   JS_CFUNC_MAGIC_DEF("colRange", 2, js_umat_funcs, 2),
                                                   JS_CFUNC_MAGIC_DEF("rowRange", 2, js_umat_funcs, 3),
                                                   JS_CFUNC_MAGIC_DEF("clone", 0, js_umat_funcs, 5),
                                                   JS_CFUNC_MAGIC_DEF("roi", 0, js_umat_funcs, 6),
                                                   JS_CFUNC_MAGIC_DEF("release", 0, js_umat_funcs, 8),
                                                   JS_CFUNC_MAGIC_DEF("dup", 0, js_umat_funcs, 9),
                                                   JS_CFUNC_MAGIC_DEF("clear", 0, js_umat_funcs, 10),
                                                   JS_CFUNC_MAGIC_DEF("reset", 0, js_umat_funcs, 11),
                                                   JS_CFUNC_MAGIC_DEF("resize", 1, js_umat_funcs, 12),
                                                   JS_CFUNC_MAGIC_DEF("step1", 0, js_umat_funcs, 13),
                                                   JS_CFUNC_MAGIC_DEF("locateROI", 0, js_umat_funcs, 14),

                                                   JS_CFUNC_MAGIC_DEF("and", 2, js_umat_expr, UMAT_EXPR_AND),
                                                   JS_CFUNC_MAGIC_DEF("or", 2, js_umat_expr, UMAT_EXPR_OR),
                                                   JS_CFUNC_MAGIC_DEF("xor", 3, js_umat_expr, UMAT_EXPR_XOR),
                                                   JS_CFUNC_MAGIC_DEF("mul", 3, js_umat_expr, UMAT_EXPR_MUL),

                                                   JS_CFUNC_MAGIC_DEF("zero", 2, js_umat_fill, 0),
                                                   JS_CFUNC_MAGIC_DEF("one", 2, js_umat_fill, 1),

                                                   JS_CFUNC_DEF("toString", 0, js_umat_tostring),
                                                   JS_CFUNC_DEF("inspect", 0, js_umat_inspect),
                                                   JS_CFUNC_DEF("at", 1, js_umat_at),
                                                   JS_CFUNC_DEF("set", 2, js_umat_set),
                                                   JS_CFUNC_DEF("setTo", 0, js_umat_set_to),
                                                   JS_CFUNC_DEF("convertTo", 2, js_umat_convert_to),
                                                   JS_CFUNC_DEF("copyTo", 1, js_umat_copy_to),
                                                   JS_CFUNC_DEF("reshape", 1, js_umat_reshape),
                                                   JS_CFUNC_MAGIC_DEF("keys", 0, js_umat_iterator_new, UMAT_ITERATOR_KEYS),
                                                   JS_CFUNC_MAGIC_DEF("values", 0, js_umat_iterator_new, UMAT_ITERATOR_VALUES),
                                                   JS_CFUNC_MAGIC_DEF("entries", 0, js_umat_iterator_new, UMAT_ITERATOR_ENTRIES),
                                                   JS_ALIAS_DEF("[Symbol.iterator]", "entries"),
                                                   JS_ALIAS_DEF("[Symbol.toPrimitive]", "toString"),

                                                   JS_PROP_STRING_DEF("[Symbol.toStringTag]", "Mat", JS_PROP_CONFIGURABLE)

};

const JSCFunctionListEntry js_umat_iterator_proto_funcs[] = {
    JS_ITERATOR_NEXT_DEF("next", 0, js_umat_iterator_next, 0),
    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "UMatIterator", JS_PROP_CONFIGURABLE),
    JS_CFUNC_DEF("[Symbol.iterator]", 0, js_umat_iterator_dup),

};

const JSCFunctionListEntry js_umat_static_funcs[] = {
    JS_CFUNC_DEF("getRotationMatrix2D", 3, js_mat_getrotationmatrix2d),
    JS_CFUNC_MAGIC_DEF("add", 2, js_umat_class_func, 0),
    JS_CFUNC_MAGIC_DEF("sub", 2, js_umat_class_func, 1),
    JS_CFUNC_MAGIC_DEF("mul", 2, js_umat_class_func, 2),
    JS_CFUNC_MAGIC_DEF("div", 2, js_umat_class_func, 3),
    JS_CFUNC_MAGIC_DEF("and", 2, js_umat_class_func, 4),
    JS_CFUNC_MAGIC_DEF("or", 2, js_umat_class_func, 5),
    JS_CFUNC_MAGIC_DEF("xor", 3, js_umat_class_func, 6),
    JS_CFUNC_MAGIC_DEF("zeros", 1, js_umat_class_create, 0),
    JS_CFUNC_MAGIC_DEF("ones", 1, js_umat_class_create, 1),
    JS_PROP_INT32_DEF("CV_8U", CV_MAKETYPE(CV_8U, 1), JS_PROP_ENUMERABLE),
};

int
js_umat_init(JSContext* ctx, JSModuleDef* m) {
  if(js_umat_class_id == 0) {
    /* create the Mat class */
    JS_NewClassID(&js_umat_class_id);
    JS_NewClassID(&js_umat_iterator_class_id);
    JS_NewClass(JS_GetRuntime(ctx), js_umat_class_id, &js_umat_class);
    JS_NewClass(JS_GetRuntime(ctx), js_umat_iterator_class_id, &js_umat_iterator_class);

    umat_proto = JS_NewObject(ctx);
    JS_SetPropertyFunctionList(ctx, umat_proto, js_umat_proto_funcs, countof(js_umat_proto_funcs));
    JS_SetClassProto(ctx, js_umat_class_id, umat_proto);

    umat_iterator_proto = JS_NewObject(ctx);
    JS_SetPropertyFunctionList(ctx, umat_iterator_proto, js_umat_iterator_proto_funcs, countof(js_umat_iterator_proto_funcs));
    JS_SetClassProto(ctx, js_umat_iterator_class_id, umat_iterator_proto);

    umat_class = JS_NewCFunction2(ctx, js_umat_ctor, "Mat", 2, JS_CFUNC_constructor, 0);
    /* set proto.constructor and ctor.prototype */
    JS_SetConstructor(ctx, umat_class, umat_proto);

    JS_SetPropertyFunctionList(ctx, umat_class, js_umat_static_funcs, countof(js_umat_static_funcs));

    JSValue g = JS_GetGlobalObject(ctx);
    int32array_ctor = JS_GetProperty(ctx, g, JS_ATOM_Int32Array);
    int32array_proto = JS_GetPrototype(ctx, int32array_ctor);

    JS_FreeValue(ctx, g);
  }

  if(m)
    JS_SetModuleExport(ctx, m, "Mat", umat_class);
  return 0;
}

extern "C" VISIBLE JSModuleDef*
JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_umat_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "Mat");
  return m;
}