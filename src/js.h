
#ifndef JS_H
#define JS_H

#include "quickjs/quickjs.h"
extern "C" {
#include "quickjs/quickjs-atom.h"
}
#include <unordered_map>
#include <vector>
#include <type_traits>
#include <functional>
#include <string>
#include <cstring>
#include <iostream>
#include <iterator>

struct jsiter;

struct jsrt {
  typedef JSValue value;
  typedef JSValueConst const_value;

  const_value _true, _false, _null, _undefined;

  jsrt() : global(*this) {}

  bool init(int argc, char* argv[]);
  ~jsrt();

  typedef value c_function(jsrt* rt, const_value this_val, int argc, const_value* argv);

  int eval_buf(const char* buf, int buf_len, const char* filename, int eval_flags);
  int eval_file(const char* filename, int module = -1);

  value add_function(const char* name, JSCFunction* fn, int args = 0);

  template<class T> void get_number(const_value val, T& ref) const;
  void get_string(const_value val, std::string& str) const;
  void get_string(const_value val, const char*& cstr) const;

  template<class T> void get_int_array(const_value val, T& ref) const;
  template<class T> void get_point(const_value val, T& ref) const;
  template<class T> void get_point_array(const_value val, std::vector<T>& ref) const;
  template<class T> void get_rect(const_value val, T& ref) const;
  template<class T> void get_color(const_value val, T& ref) const;

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
  bool
  has_property(const_value obj, T prop) const {
    return false;
  }

  template<class T>
  void
  set_property(const_value obj, T prop, value val) {}

  value get_constructor(const_value obj) const;
  bool has_constructor(const_value obj) const;

  value get_prototype(const_value obj) const;
  bool has_prototype(const_value obj) const;

  std::string function_name(const_value fn) const;

  value get_global(const char* name);
  void set_global(const char* name, const_value v);
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

  void property_names(const_value obj,
                      std::vector<const char*>& out,
                      bool enum_only = false,
                      bool recursive = false) const;

  std::vector<const char*> property_names(const_value obj,
                                          bool enum_only = true,
                                          bool recursive = true) const;

  void dump_error();

  void dump_exception(JSValueConst exception_val, bool is_throw);

  bool is_number(const_value val) const;
  bool is_undefined(const_value val) const;
  bool is_array(const_value val) const;
  bool is_object(const_value val) const;
  bool is_boolean(const_value val) const;
  bool is_point(const_value val) const;
  bool is_rect(const_value val) const;
  bool is_color(const_value val) const;
  bool is_array_like(const_value val) const;

  int
  tag(const_value val) const {
    return JS_VALUE_GET_TAG(val);
  }
  std::string
  typestr(const_value val) const {
    if(is_number(val))
      return "number";
    else if(is_undefined(val))
      return "undefined";
    else if(is_array(val))
      return "array";
    else if(is_object(val))
      return "object";
    else if(is_boolean(val))
      return "boolean";
    else if(is_point(val))
      return "point";
    else if(is_rect(val))
      return "rect";
    else if(is_color(val))
      return "color";
    return "unknown";
  }

  std::string
  to_string(const_value arg) const {
    std::string ret;
    get_string(arg, ret);
    return ret;
  }

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

public:
  JSContext* ctx;

private:
  JSRuntime* rt;
  static char* normalize_module(JSContext* ctx,
                                const char* module_base_name,
                                const char* module_name,
                                void* opaque);
  std::unordered_map<const char*, std::pair<JSCFunction*, value>> funcmap;

public:
  int32_t get_length(const const_value& v) const;
  jsiter begin(JSValue& v);
  jsiter end(JSValue& v);

private:
  friend class jsiter;

  std::function<JSValue(JSValue, uint32_t)> index() const;
  std::function<JSValue(uint32_t)> index(const JSValueConst&) const;
};

inline void
jsrt::get_string(const_value val, std::string& str) const {
  const char* s = JS_ToCString(ctx, val);
  str = std::string(s);
  JS_FreeCString(ctx, s);
}

inline void
jsrt::get_string(const_value val, const char*& str) const {
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
  value vx = _undefined, vy = _undefined;

  if(is_array(val)) {
    uint32_t length;
    get_number(get_property(val, "length"), length);
    if(length >= 2) {
      vx = get_property<uint32_t>(val, 0);
      vy = get_property<uint32_t>(val, 1);
    } else {
      return;
    }
  } else if(is_object(val) && has_property(val, "x") && has_property(val, "y")) {
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
jsrt::get_color(const_value val, T& ref) const {
  value vr = _undefined, vg = _undefined, vb = _undefined, va = _undefined;

  if(is_object(val) && has_property(val, "r") && has_property(val, "g") && has_property(val, "b")) {
    vr = get_property(val, "r");
    vg = get_property(val, "g");
    vb = get_property(val, "b");
    va = get_property(val, "a");
  } else if(is_array_like(val)) {
    uint32_t length;
    get_number(get_property(val, "length"), length);
    if(length >= 3) {
      vr = get_property<uint32_t>(val, 0);
      vg = get_property<uint32_t>(val, 1);
      vb = get_property<uint32_t>(val, 2);
      va = get_property<uint32_t>(val, 3);
    } else {
      return;
    }
  }
  get_number(vb, ref[0]);
  get_number(vg, ref[1]);
  get_number(vr, ref[2]);
  get_number(va, ref[3]);
}

template<class T>
inline void
jsrt::get_point_array(const_value val, std::vector<T>& ref) const {
  if(is_array_like(val)) {
    uint32_t i, length;

    get_number(get_property(val, "length"), length);
    ref.resize(length);

    for(i = 0; i < length; i++) {
      JSValueConst pt = get_property<uint32_t>(val, i);

      get_point(pt, ref[i]);
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

inline jsrt::value
jsrt::get_constructor(jsrt::const_value obj) const {
  return get_property(get_prototype(obj), "constructor");
}

inline bool
jsrt::has_constructor(jsrt::const_value obj) const {
  return !is_undefined(get_constructor(obj));
}

inline jsrt::value
jsrt::get_prototype(jsrt::const_value obj) const {
  return JS_GetPrototype(ctx, obj);
}

inline bool
jsrt::has_prototype(jsrt::const_value obj) const {
  return !is_undefined(get_prototype(obj));
}

inline std::string
jsrt::function_name(jsrt::const_value fn) const {
  return to_string(get_property(fn, "name"));
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
vector_to_js(jsrt& js,
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
                      std::bind(&jsrt::create<typename T::value_type>, &js, std::placeholders::_1));
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

template<>
inline bool
jsrt::has_property<const char*>(const_value obj, const char* name) const {
  JSAtom atom = JS_NewAtom(ctx, name);
  bool present = !!JS_HasProperty(ctx, obj, atom);
  JS_FreeAtom(ctx, atom);
  return present;
}

template<>
inline bool
jsrt::has_property<uint32_t>(const_value obj, uint32_t index) const {
  JSAtom atom = JS_NewAtomUInt32(ctx, index);
  bool present = !!JS_HasProperty(ctx, obj, atom);
  JS_FreeAtom(ctx, atom);
  return present;
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

inline bool
jsrt::is_array_like(const_value val) const {
  if(is_array(val))
    return true;

  if(has_property(val, "length")) {
    if(is_number(get_property(val, "length")))
      return true;
  }
  return false;
}

/**
 * Array iterator
 */
struct jsiter {
  JSValue operator*() const {
    if(p < n)
      return i((uint32_t)p);
    else
      return JS_UNDEFINED;
  }
  jsiter
  operator++() {
    jsiter ret = *this;
    if(p < n)
      p++;
    return ret;
  }
  jsiter&
  operator++(int) {
    if(p < n)
      ++p;
    return *this;
  }
  bool
  operator==(const jsiter& o) const {
    return p == o.p && n == o.n;
  }
  bool
  operator<(const jsiter& o) const {
    return p < o.p;
  }
  bool
  operator>(const jsiter& o) const {
    return p > o.p;
  }
  bool
  operator<=(const jsiter& o) const {
    return !(*this > o);
  }
  bool
  operator>=(const jsiter& o) const {
    return !(*this < o);
  }
  bool
  operator!=(const jsiter& o) const {
    return !(*this == o);
  }

  ptrdiff_t
  operator-(const jsiter& o) const {
    return p - o.p;
  }

  jsiter
  operator-(size_t o) const {
    return jsiter(i, n, p - o);
  }

  jsiter
  operator+(size_t o) const {
    return jsiter(i, n, p + o);
  }

protected:
  std::function<JSValue(uint32_t)> i;
  uint32_t n;
  int32_t p;

private:
  friend class jsrt;

  jsiter(std::function<JSValue(uint32_t)> index, size_t len, size_t pos)
      : i(index), n(len), p(pos) {}
  jsiter(jsrt& js, const JSValue& arr, size_t len) : i(js.index(arr)), n(len), p(0) {}
  jsiter(jsrt& js, const JSValue& arr, size_t len, size_t pos) : i(js.index(arr)), n(len), p(pos) {}
};

inline int32_t
jsrt::get_length(const jsrt::const_value& a) const {
  uint32_t length;
  JSValue v = get_property(a, "length");
  if(is_undefined(v))
    return -1;
  get_number(v, length);
  return length;
}

inline jsiter
jsrt::begin(JSValue& v) {
  uint32_t n = get_length(v);
  return jsiter(*this, v, n, 0);
}

inline jsiter
jsrt::end(JSValue& v) {
  uint32_t n = get_length(v);
  return jsiter(*this, v, n, n);
}

inline std::function<JSValue(JSValue, uint32_t)>
jsrt::index() const {
  return std::bind(&jsrt::get_property<uint32_t>,
                   this,
                   std::placeholders::_1,
                   std::placeholders::_2);
}
inline std::function<JSValue(uint32_t)>
jsrt::index(const JSValue& a) const {
  return std::bind(&jsrt::get_property<uint32_t>, this, a, std::placeholders::_1);
}

inline void
jsrt::set_global(const char* name, const_value v) {
  value g = global_object();
  set_property(g, name, v);
}

extern "C" jsrt js;

#endif // defined JS_H
