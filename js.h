#ifndef JS_H
#define JS_H

#include "../quickjs/quickjs.h"
#include <unordered_map>
#include <vector>
#include <type_traits>
#include <functional>

struct jsrt {
  typedef JSValue value;
  typedef JSValueConst const_value;

  jsrt() : global(*this) {}

  bool init(int argc, char* argv[]);
  ~jsrt();

  typedef value
  c_function(jsrt* rt, const_value this_val, int argc, const_value* argv);

  int
  eval_buf(const char* buf, int buf_len, const char* filename, int eval_flags);
  int eval_file(const char* filename, int module = -1);

  value add_function(const char* name, JSCFunction* fn, int args = 0);

  template<class T> void get_int_array(const_value val, T& ref);
  template<class T> void get_point(const_value val, T& ref);
  template<class T> void get_point_array(const_value val, std::vector<T>& ref);

  value create_array(int32_t size = -1);
  template<class T> value create(T arg);
  template<class T> value create_point(T x, T y);

  value get_property(const_value obj, const char* name);
  value get_property(const_value obj, uint32_t index);

  void set_property(const_value obj, const char* name, value val);
  void set_property(const_value obj, uint32_t index, value val);

  value get_global(const char* name);

  value call(const_value func, size_t argc, const_value* argv);
  value call(const_value func, std::vector<const_value>& args);
  value call(const char* name, size_t argc, const_value* argv);

  value* get_function(const char* name);

  std::string to_str(const_value val);

  const_value prototype(const_value obj) const;

  void property_names(const_value obj,
                      std::vector<const char*>& out,
                      bool enum_only = false,
                      bool recursive = false) const;

  std::vector<const char*> property_names(const_value obj,
                                          bool enum_only = true,
                                          bool recursive = true) const;

public:
  struct global_object {
    global_object(jsrt& js);
    global_object(global_object&& o) noexcept;
    ~global_object();

    operator const_value() const { return val; }
    operator value() { return val; }

  private:
    friend class jsrt;
    bool get();

    value val;
    jsrt& js;
  } global;

public:
  global_object&
  get_global_object() {
    global.get();
    return global;
  }

  value get_undefined() const;
  value get_null() const;
  value get_true() const;
  value get_false() const;

private:
  JSRuntime* rt;
  JSContext* ctx;

  std::unordered_map<const char*, std::pair<JSCFunction*, value>> funcmap;
};

template<class T>
inline void
jsrt::get_int_array(const_value val, T& ref) {
  int32_t i, n, length = 0, arr = JS_IsArray(ctx, val);
  if(arr) {
    value lval = JS_GetPropertyStr(ctx, val, "length");
    JS_ToInt32(ctx, &length, lval);
    for(i = 0; i < length; i++) {
      value v = JS_GetPropertyUint32(ctx, val, i);
      JS_ToInt32(ctx, &n, v);
      ref[i] = n;
    }
  }
}

template<class T>
inline void
jsrt::get_point(const_value val, T& ref) {
  bool arr = JS_IsArray(ctx, val);
  value vx, vy;
  double x = 0, y = 0;
  if(arr) {
    int32_t length;
    value lval = JS_GetPropertyStr(ctx, val, "length");
    JS_ToInt32(ctx, &length, lval);
    if(length >= 2) {
      vx = JS_GetPropertyUint32(ctx, val, 0);
      vy = JS_GetPropertyUint32(ctx, val, 1);
    }
  } else {
    vx = JS_GetPropertyStr(ctx, val, "x");
    vy = JS_GetPropertyStr(ctx, val, "y");
  }
  JS_ToFloat64(ctx, &x, vx);
  JS_ToFloat64(ctx, &y, vy);

  ref.x = x;
  ref.y = y;
}

template<class T>
inline void
jsrt::get_point_array(const_value val, std::vector<T>& ref) {
  int32_t i, n, length = 0, arr = JS_IsArray(ctx, val);
  if(arr) {
    value lval = JS_GetPropertyStr(ctx, val, "length");
    JS_ToInt32(ctx, &length, lval);
    ref.resize(length);

    for(i = 0; i < length; i++) {
      value v = JS_GetPropertyUint32(ctx, val, i);

      get_point(v, ref[i]);
    }
  }
}

template<>
inline jsrt::value
jsrt::create<double>(double num) {
  return JS_NewFloat64(ctx, num);
}
template<>
inline jsrt::value
jsrt::create<float>(float num) {
  return create(static_cast<double>(num));
}
template<>
inline jsrt::value
jsrt::create<int>(int num) {
  return JS_NewInt32(ctx, num);
}
template<>
inline jsrt::value
jsrt::create<const char*>(const char* str) {
  return JS_NewString(ctx, str);
}
template<>
inline jsrt::value
jsrt::create<bool>(bool b) {
  return b ? get_true() : get_false();
}

inline jsrt::value
jsrt::create_array(int32_t size) {
  value ret = JS_NewArray(ctx);
  if(size >= 0)
    JS_SetPropertyStr(ctx, ret, "length", JS_NewInt32(ctx, size));

  return ret;
}

template<class T>
inline jsrt::value
jsrt::create(T arg) {
  return std::is_pointer<T>::value && arg == nullptr ? get_null()
                                                     : get_undefined();
}

template<class T>
inline jsrt::value
jsrt::create_point(T x, T y) {
  value obj = JS_NewObject(ctx);
  JS_SetPropertyStr(ctx, obj, "x", create(x));
  JS_SetPropertyStr(ctx, obj, "y", create(y));
  return obj;
}

inline jsrt::value
jsrt::get_property(const_value obj, uint32_t index) {
  return JS_GetPropertyUint32(ctx, obj, index);
}

inline jsrt::value
jsrt::get_property(const_value obj, const char* name) {
  return JS_GetPropertyStr(ctx, obj, name);
}

inline void
jsrt::set_property(const_value obj, const char* name, value val) {
  JS_SetPropertyStr(ctx, obj, name, val);
}

inline void
jsrt::set_property(const_value obj, uint32_t index, value val) {
  JS_SetPropertyUint32(ctx, obj, index, val);
}

template<class T>
inline jsrt::value
vector_to_js(
    jsrt& js,
    const T& v,
    size_t n,
    const std::function<jsrt::value(const typename T::value_type&)>& fn) {
  using std::placeholders::_1;
  jsrt::value ret = js.create_array(n);
  for(uint32_t i = 0; i < n; i++) js.set_property(ret, i, fn(v[i]));
  return ret;
}

template<class T>
inline jsrt::value
vector_to_js(jsrt& js, const T& v, size_t n) {
  return vector_to_js(v,
                      n,
                      std::bind(&jsrt::create<typename T::value_type>,
                                &js,
                                std::placeholders::_1));
}

template<class T>
inline jsrt::value
vector_to_js(jsrt& js, const T& v) {
  return vector_to_js(js, v, v.size());
}

template<class P>
inline jsrt::value
vector_to_js(jsrt& js,
             const std::vector<P>& v,
             const std::function<jsrt::value(const P&)>& fn) {
  return vector_to_js(js, v, v.size(), fn);
}

template<class P>
inline jsrt::value
pointer_to_js(jsrt& js,
              const P* v,
              size_t n,
              const std::function<jsrt::value(const P&)>& fn) {
  jsrt::value ret = js.create_array(n);
  for(uint32_t i = 0; i < n; i++) js.set_property(ret, i, fn(v[i]));
  return ret;
}

template<class P>
inline jsrt::value
pointer_to_js(jsrt& js, const P* v, size_t n) {
  std::function<jsrt::value(const P&)> fn(
      [&](const P& v) -> jsrt::value { return js.create(v); });

  return pointer_to_js(js, v, n, fn);
}

template<class P>
inline jsrt::value
vector_to_js(jsrt& js, const std::vector<P>& v, jsrt::value (*fn)(const P&)) {
  uint32_t i, n = v.size();
  jsrt::value ret = js.create_array(n);
  for(i = 0; i < n; i++) js.set_property(ret, i, fn(v[i]));
  return ret;
}

template<class P>
inline jsrt::value
vector_to_js(jsrt& js, const std::vector<P>& v) {
  using std::placeholders::_1;
  return vector_to_js<P>(v, std::bind(&jsrt::create<P>, &js, _1));
}

inline std::string
to_string(const char* s) {
  return std::string(s);
}

#endif // defined JS_H
