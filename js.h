#ifndef JS_H
#define JS_H

#include "../quickjs/quickjs.h"
#include <unordered_map>
#include <vector>
#include <type_traits>
#include <functional>
#include <string>
#include <cstring>
#include <iostream>

struct jsrt {
  typedef JSValue value;
  typedef JSValueConst const_value;
  struct global;
  const_value _true, _false, _null, _undefined;

  jsrt() : global(*this) {}

  bool init(int argc, char* argv[]);
  ~jsrt();

  typedef value c_function(jsrt* rt, const_value this_val, int argc, const_value* argv);

  int eval_buf(const char* buf, int buf_len, const char* filename, int eval_flags);
  int eval_file(const char* filename, int module = -1);

  value add_function(const char* name, JSCFunction* fn, int args = 0);

  template<class T> void get_number(const_value val, T& ref) const;
  template<class T>
  void
  get_string(const_value val, T& str) const {}

  template<class T> void get_int_array(const_value val, T& ref) const;
  template<class T> void get_point(const_value val, T& ref) const;
  template<class T> void get_point_array(const_value val, std::vector<T>& ref) const;
  template<class T> void get_rect(const_value val, T& ref) const;

  void
  get_boolean(const_value val, bool& ref) {
    bool b = JS_ToBool(ctx, val);
    ref = b;
  }
  bool
  get_boolean(const_value val) {
    bool b;
    get_boolean(val, b);
    return b;
  }

  value create_array(int32_t size = -1);
  template<class T> value create(T arg);
  template<class T> value create_point(T x, T y);

  template<class T>
  value
  get_property(const_value obj, T prop) const {
    return get_undefined();
  }

  template<class T>
  void
  set_property(const_value obj, T prop, value val) {}
  /*  void set_property(const_value obj, const char* name, value val);
    void set_property(const_value obj, uint32_t index, value val);
  */
  value get_global(const char* name);
  value
  global_object() {
    global.get();
    return global;
  }

  value call(const_value func, size_t argc, const_value* argv);
  value call(const_value func, std::vector<const_value>& args);
  value call(const char* name, size_t argc, const_value* argv);

  value* get_function(const char* name);

  std::string to_str(const_value val);

  const_value prototype(const_value obj) const;

  void property_names(const_value obj, std::vector<const char*>& out, bool enum_only = false, bool recursive = false) const;

  std::vector<const char*> property_names(const_value obj, bool enum_only = true, bool recursive = true) const;

  void dump_error();

  void dump_exception(JSValueConst exception_val, bool is_throw);

  bool is_number(const_value val) const;
  bool is_undefined(const_value val) const;
  bool is_array(const_value val) const;
  bool is_object(const_value val) const;
  bool is_boolean(const_value val) const;
  bool is_point(const_value val) const;
  bool is_rect(const_value val) const;

protected:
  struct global {
    global(jsrt& js);
    global(global&& o) noexcept;
    ~global();

    operator const_value() const { return val; }
    operator value() { return val; }

  private:
    friend class jsrt;
    bool get();

    value val;
    jsrt& js;
  } global;

  value get_undefined() const;
  value get_null() const;
  value get_true() const;
  value get_false() const;

private:
  JSRuntime* rt;
  JSContext* ctx;

  static char* normalize_module(JSContext* ctx, const char* module_base_name, const char* module_name, void* opaque);

  std::unordered_map<const char*, std::pair<JSCFunction*, value>> funcmap;
};

template<>
inline void
jsrt::get_string<std::string>(const_value val, std::string& str) const {
  const char* s = JS_ToCString(ctx, val);
  str = std::string(s);
  JS_FreeCString(ctx, s);
}

template<>
inline void
jsrt::get_string<const char*>(const_value val, const char*& str) const {
  const char* s = JS_ToCString(ctx, val);
  str = strdup(s);
  JS_FreeCString(ctx, s);
}

template<>
inline void
jsrt::get_number<int32_t>(const_value val, int32_t& ref) const {
  int32_t i = -1;
  JS_ToInt32(ctx, &i, val);
  ref = i;
}

template<>
inline void
jsrt::get_number<int64_t>(const_value val, int64_t& ref) const {
  int64_t i = 0;
  JS_ToInt64(ctx, &i, val);
  ref = i;
}

template<>
inline void
jsrt::get_number<double>(const_value val, double& ref) const {
  double f = 0;
  JS_ToFloat64(ctx, &f, val);
  ref = f;
}

template<>
inline void
jsrt::get_number<float>(const_value val, float& ref) const {
  double f = 0;
  get_number(val, f);
  ref = f;
}

template<>
inline void
jsrt::get_number<uint32_t>(const_value val, uint32_t& ref) const {
  int64_t i = 0;
  get_number<int64_t>(val, i);
  ref = static_cast<uint32_t>(i);
}

template<class T>
inline void
jsrt::get_int_array(const_value val, T& ref) const {
  uint32_t i, n, length = 0, arr = JS_IsArray(ctx, val);
  if(arr) {
    get_number(get_property(val, "length"), length);
    for(i = 0; i < length; i++) {
      value v = get_property<uint32_t>(val, i);
      get_number(v, n);
      ref[i] = n;
    }
  }
}

template<class T>
inline void
jsrt::get_point(const_value val, T& ref) const {
  bool arr = JS_IsArray(ctx, val);
  value vx = _undefined, vy = _undefined;

  if(arr) {
    uint32_t length;
    get_number(get_property(val, "length"), length);
    if(length >= 2) {
      vx = get_property<uint32_t>(val, 0);
      vy = get_property<uint32_t>(val, 1);
    } else {
      return;
    }
  } else {
    vx = get_property(val, "x");
    vy = get_property(val, "y");
  }

  get_number(vx, ref.x);
  get_number(vy, ref.y);
}

template<class T>
inline void
jsrt::get_rect(const_value val, T& ref) const {
  if(is_object(val)) {
    double vw, vh;

    get_point(val, ref);

    get_number(get_property(val, "width"), vw);
    get_number(get_property(val, "height"), vh);

    ref.width = vw;
    ref.height = vh;
  }
}

template<class T>
inline void
jsrt::get_point_array(const_value val, std::vector<T>& ref) const {
  uint32_t i, n, length = 0, arr = JS_IsArray(ctx, val);
  if(arr) {
    get_number(get_property(val, "length"), length);
    ref.resize(length);

    for(i = 0; i < length; i++) {
      JSValueConst prop = get_property<uint32_t>(val, i);
      std::string s;

      get_string(prop, s);

      std::cerr << "point prop: " << s << std::endl;

      get_point(prop, ref[i]);
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
  return std::is_pointer<T>::value && arg == nullptr ? get_null() : get_undefined();
}

template<class T>
inline jsrt::value
jsrt::create_point(T x, T y) {
  value obj = JS_NewObject(ctx);
  JS_SetPropertyStr(ctx, obj, "x", create(x));
  JS_SetPropertyStr(ctx, obj, "y", create(y));
  return obj;
}

template<>
inline jsrt::value
jsrt::get_property<uint32_t>(const_value obj, uint32_t index) const {
  return JS_GetPropertyUint32(ctx, obj, index);
}

template<>
inline jsrt::value
jsrt::get_property<const char*>(const_value obj, const char* name) const {
  return JS_GetPropertyStr(ctx, obj, name);
}

template<>
inline void
jsrt::set_property<const char*>(const_value obj, const char* name, value val) {
  JS_SetPropertyStr(ctx, obj, name, val);
}

template<>
inline void
jsrt::set_property<uint32_t>(const_value obj, uint32_t index, value val) {
  JS_SetPropertyUint32(ctx, obj, index, val);
}

template<class T>
inline jsrt::value
vector_to_js(jsrt& js, const T& v, size_t n, const std::function<jsrt::value(const typename T::value_type&)>& fn) {
  using std::placeholders::_1;
  jsrt::value ret = js.create_array(n);
  for(uint32_t i = 0; i < n; i++) js.set_property(ret, i, fn(v[i]));
  return ret;
}

template<class T>
inline jsrt::value
vector_to_js(jsrt& js, const T& v, size_t n) {
  return vector_to_js(v, n, std::bind(&jsrt::create<typename T::value_type>, &js, std::placeholders::_1));
}

template<class T>
inline jsrt::value
vector_to_js(jsrt& js, const T& v) {
  return vector_to_js(js, v, v.size());
}

template<class P>
inline jsrt::value
vector_to_js(jsrt& js, const std::vector<P>& v, const std::function<jsrt::value(const P&)>& fn) {
  return vector_to_js(js, v, v.size(), fn);
}

template<class P>
inline jsrt::value
pointer_to_js(jsrt& js, const P* v, size_t n, const std::function<jsrt::value(const P&)>& fn) {
  jsrt::value ret = js.create_array(n);
  for(uint32_t i = 0; i < n; i++) js.set_property(ret, i, fn(v[i]));
  return ret;
}

template<class P>
inline jsrt::value
pointer_to_js(jsrt& js, const P* v, size_t n) {
  std::function<jsrt::value(const P&)> fn([&](const P& v) -> jsrt::value { return js.create(v); });

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

inline bool
jsrt::is_number(const_value val) const {
  return JS_IsNumber(val);
}

inline bool
jsrt::is_undefined(const_value val) const {
  return JS_IsUndefined(val);
}

inline bool
jsrt::is_array(const_value val) const {
  return JS_IsArray(ctx, val);
}

inline bool
jsrt::is_object(const_value val) const {
  return JS_IsObject(val);
}

inline bool
jsrt::is_boolean(const_value val) const {
  return JS_IsBool(val);
}

#endif // defined JS_H
