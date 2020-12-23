#ifndef JS_ARRAY_H
#define JS_ARRAY_H

static inline int64_t
js_array_length(JSContext* ctx, const JSValueConst& arr) {
  int64_t ret = -1;
  if(JS_IsArray(ctx, arr)) {
    uint32_t len;
    JSValue v = JS_GetPropertyStr(ctx, arr, "length");
    JS_ToUint32(ctx, &len, v);
    JS_FreeValue(ctx, v);
    ret = len;
  }
  return ret;
}

extern "C" int JS_DeletePropertyInt64(JSContext* ctx, JSValueConst obj, int64_t idx, int flags);

static inline int64_t
js_array_truncate(JSContext* ctx, const JSValueConst& arr, int64_t len) {
  int64_t newlen = -1;
  if(JS_IsArray(ctx, arr)) {
    int64_t top = js_array_length(ctx, arr);
    newlen = std::min(top, len < 0 ? top + len : len);

    while(--top >= newlen) {
      JS_DeletePropertyInt64(ctx, arr, top, 0);
    }
    JS_SetPropertyStr(ctx, arr, "length", JS_NewInt64(ctx, newlen));
  }
  return newlen;
}

class js_array_iterator : public std::iterator<std::input_iterator_tag, JSValue> {
public:
  js_array_iterator(JSContext* c, const JSValueConst& a, const size_t i = 0) : ctx(c), array(&a), pos(i) {}
  value_type
  operator*() const {
    return JS_GetPropertyUint32(ctx, *array, pos);
  }
  js_array_iterator&
  operator++() {
    ++this->pos;
    return *this;
  }
  js_array_iterator
  operator++(int) {
    js_array_iterator temp(*this);
    ++(*this);
    return temp;
  }
  bool
  operator==(const js_array_iterator& rhs) {
    return array == rhs.array && pos == rhs.pos;
  }
  bool
  operator!=(const js_array_iterator& rhs) {
    return !operator==(rhs);
  }

private:
  JSContext* ctx;
  const JSValueConst* array;
  difference_type pos;
};

static inline js_array_iterator
js_begin(JSContext* c, const JSValueConst& a) {
  return js_array_iterator(c, a, 0);
}

static inline js_array_iterator
js_end(JSContext* c, const JSValueConst& a) {
  return js_array_iterator(c, a, js_array_length(c, a));
}

template<class T> class js_array {
public:
  static int64_t
  to_vector(JSContext* ctx, JSValueConst arr, std::vector<T>& out) {
    int64_t i, n;
    JSValue len;
    if(!JS_IsArray(ctx, arr))
      return -1;
    len = JS_GetPropertyStr(ctx, arr, "length");
    JS_ToInt64(ctx, &n, len);
    out.reserve(out.size() + n);
    for(i = 0; i < n; i++) {
      double value;
      JSValue item = JS_GetPropertyUint32(ctx, arr, (uint32_t)i);
      if(JS_ToFloat64(ctx, &value, item) == -1) {
        JS_FreeValue(ctx, item);
        out.clear();
        return -1;
      }
      out.push_back(value);
      JS_FreeValue(ctx, item);
    }
    return n;
  }

  template<class Container>
  static JSValue
  from(JSContext* ctx, const Container& in) {
    return from_sequence<typename Container::const_iterator>(ctx, in.cbegin(), in.cend());
  }

  static JSValue
  from_vector(JSContext* ctx, const std::vector<T>& in) {
    return from_sequence(ctx, in.cbegin(), in.cend());
  }

  template<class Iterator>
  static JSValue
  from_sequence(JSContext* ctx, const Iterator& start, const Iterator& end) {
    JSValue arr = JS_NewArray(ctx);
    size_t i = 0;
    for(Iterator it = start; it != end; ++it) {
      JSValue item = JS_NewFloat64(ctx, *it);
      JS_SetPropertyUint32(ctx, arr, i, item);
      ++i;
    }
    return arr;
  }

  template<size_t N> static int64_t to_array(JSContext* ctx, JSValueConst arr, std::array<T, N>& out);
  static int64_t to_scalar(JSContext* ctx, JSValueConst arr, cv::Scalar_<T>& out);
};

template<class T>
template<size_t N>
int64_t
js_array<T>::to_array(JSContext* ctx, JSValueConst arr, std::array<T, N>& out) {
  std::vector<T> tmp;
  to_vector(ctx, arr, tmp);
  if(tmp.size() < N)
    return -1;
  for(size_t i = 0; i < N; i++) out[i] = tmp[i];
  return N;
}

template<class T>
int64_t
js_array<T>::to_scalar(JSContext* ctx, JSValueConst arr, cv::Scalar_<T>& out) {
  size_t n;
  std::vector<T> tmp;
  to_vector(ctx, arr, tmp);
  if((n = tmp.size()) < 4)
    tmp.resize(4);
  for(size_t i = 0; i < 4; i++) out[i] = tmp[i];
  return n;
}

#include "js_point.h"

template<class T> class js_array<JSPointData<T>> {
public:
  static int64_t
  to_vector(JSContext* ctx, JSValueConst arr, JSContourData<T>& out) {
    int64_t i, n;
    JSValue len;
    if(!JS_IsArray(ctx, arr))
      return -1;
    len = JS_GetPropertyStr(ctx, arr, "length");
    JS_ToInt64(ctx, &n, len);
    out.reserve(out.size() + n);
    for(i = 0; i < n; i++) {
      JSPointData<double> value;
      JSValue item = JS_GetPropertyUint32(ctx, arr, (uint32_t)i);
      if(!js_point_read(ctx, item, &value)) {
        JS_FreeValue(ctx, item);
        out.clear();
        return -1;
      }
      out.push_back(value);
      JS_FreeValue(ctx, item);
    }
    return n;
  }
  template<size_t N> static int64_t to_array(JSContext* ctx, JSValueConst arr, std::array<cv::Mat, N>& out);
};

template<class T> class js_array<JSContourData<T>> {
public:
  typedef JSContourData<T> contour_type;
  typedef JSPointData<T> point_type;

  template<class Iterator>
  static JSValue
  from_sequence(JSContext* ctx, const Iterator& start, const Iterator& end) {
    JSValue arr = JS_NewArray(ctx);
    size_t i = 0;
    for(Iterator it = start; it != end; ++it) {
      JSValue item = js_contour_new(ctx, *it);
      JS_SetPropertyUint32(ctx, arr, i, item);
      ++i;
    }
    return arr;
  }

  template<class Container>
  static JSValue
  from(JSContext* ctx, const Container& in) {
    return from_sequence<typename Container::const_iterator>(ctx, in.cbegin(), in.cend());
  }
};

template<> class js_array<cv::Mat> {
public:
  static int64_t
  to_vector(JSContext* ctx, JSValueConst arr, std::vector<cv::Mat>& out) {
    int64_t i, n;
    JSValue len;
    if(!JS_IsArray(ctx, arr))
      return -1;
    len = JS_GetPropertyStr(ctx, arr, "length");
    JS_ToInt64(ctx, &n, len);
    out.reserve(out.size() + n);
    for(i = 0; i < n; i++) {
      JSMatData* value;
      JSValue item = JS_GetPropertyUint32(ctx, arr, (uint32_t)i);
      value = js_mat_data(ctx, item);
      if(value == nullptr) {
        JS_FreeValue(ctx, item);
        out.clear();
        return -1;
      }
      out.push_back(*value);
      JS_FreeValue(ctx, item);
    }
    return n;
  }
  template<size_t N> static int64_t to_array(JSContext* ctx, JSValueConst arr, std::array<cv::Mat, N>& out);
};

template<class T> class js_array<std::vector<T>> {
public:
  static int64_t
  to_vector(JSContext* ctx, JSValueConst arr, std::vector<std::vector<T>>& out) {
    int64_t i, n;
    JSValue len;
    if(!JS_IsArray(ctx, arr))
      return -1;
    len = JS_GetPropertyStr(ctx, arr, "length");
    JS_ToInt64(ctx, &n, len);
    out.reserve(out.size() + n);
    for(i = 0; i < n; i++) {
      std::vector<T> value;
      JSValue item = JS_GetPropertyUint32(ctx, arr, (uint32_t)i);
      if(js_array<T>::to_vector(ctx, arr, value) == -1) {
        JS_FreeValue(ctx, item);
        out.clear();
        return -1;
      }
      out.push_back(value);
      JS_FreeValue(ctx, item);
    }
    return n;
  }

  template<size_t N> static int64_t to_array(JSContext* ctx, JSValueConst arr, std::array<std::vector<T>, N>& out);
};

template<class T>
inline int64_t
js_array_to_vector(JSContext* ctx, JSValueConst arr, std::vector<T>& out) {
  return js_array<T>::to_vector(ctx, arr, out);
}

template<class T, size_t N>
inline int64_t
js_array_to_array(JSContext* ctx, JSValueConst arr, std::array<T, N>& out) {
  return js_array<T>::to_array<N>(ctx, arr, out);
}

template<class T>
inline int64_t
js_array_to_scalar(JSContext* ctx, JSValueConst arr, cv::Scalar_<T>& out) {
  return js_array<T>::to_scalar(ctx, arr, out);
}

template<class Iterator>
inline JSValue
js_array_from(JSContext* ctx, const Iterator& start, const Iterator& end) {
  return js_array<typename Iterator::value_type>::from_sequence(ctx, start, end);
}

template<class Container>
inline JSValue
js_array_from(JSContext* ctx, const Container& v) {
  return js_array<typename Container::value_type>::from_sequence(ctx, v.cbegin(), v.cend());
}

class js_object {
public:
  template<class T>
  static int64_t
  to_map(JSContext* ctx, JSValueConst obj, std::map<std::string, T>& out) {
    int64_t i = 0;
    jsrt js(ctx);
    auto names = js.property_names(obj);
    for(auto name : names) {
      T prop = js.to<T>(js.get_property(obj, name));
      out[name] = prop;
      ++i;
    }
    return i;
  }

  template<class T>
  static JSValue
  from_map(JSContext* ctx, const std::map<std::string, T>& in) {
    typedef std::pair<std::string, T> entry_type;
    jsrt js(ctx);
    JSValue obj = JS_NewObject(ctx);
    ;

    for(entry_type entry : in) js.set_property(obj, entry.first, js.create<T>(entry.second), JS_PROP_C_W_E);

    return obj;
  }
};

#endif /* defined(JS_ARRAY_H) */
