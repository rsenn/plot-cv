#ifndef JS_H
#define JS_H

#include "../quickjs/quickjs.h"
#include <unordered_map>
#include <vector>
#include <type_traits>
#include <functional>

struct jsrt {
  bool init(int argc, char* argv[]);
  ~jsrt();

  typedef JSValue c_function(jsrt* rt, JSValueConst this_val, int argc, JSValueConst* argv);

  int eval_buf(const char* buf, int buf_len, const char* filename, int eval_flags);
  int eval_file(const char* filename, int module = -1);

  JSValue add_function(const char* name, JSCFunction* fn, int args = 0);

  template<class T> void get_int_array(JSValueConst val, T& ref);
  template<class T> void get_point(JSValueConst val, T& ref);
  template<class T> void get_point_array(JSValueConst val, std::vector<T>& ref);

  JSValue create_array(int32_t size = -1);

  template<class T> JSValue create(T arg);

  template<class T> JSValue create_point(T x, T y);

  void set_property(JSValueConst obj, const char* name, JSValue val);
  void set_property(JSValueConst obj, uint32_t index, JSValue val);

  JSValue get_global_property(const char* name);

  JSValue call(JSValueConst func, std::vector<JSValueConst>& args);

protected:
  struct global_object {
    global_object(JSContext* __ctx) : ctx(__ctx) { val = JS_GetGlobalObject(ctx); }
    global_object(global_object&& o) noexcept : val(std::move(o.val)), ctx(std::move(o.ctx)) {}

    ~global_object() {
      if(ctx)
        JS_FreeValue(ctx, val);
    }

    JSValue val;
    JSContext* ctx;
  };

  global_object
  get_global_object() {
    return global_object(ctx);
  }

  JSValue get_undefined() const;
  JSValue get_null() const;
  JSValue get_true() const;
  JSValue get_false() const;

private:
  JSRuntime* rt;
  JSContext* ctx;

  std::unordered_map<const char*, std::pair<JSCFunction*, JSValue>> funcmap;
};

template<class T>
inline void
jsrt::get_int_array(JSValueConst val, T& ref) {
  int32_t i, n, length = 0, arr = JS_IsArray(ctx, val);
  if(arr) {
    JSValue lval = JS_GetPropertyStr(ctx, val, "length");
    JS_ToInt32(ctx, &length, lval);
    for(i = 0; i < length; i++) {
      JSValue v = JS_GetPropertyUint32(ctx, val, i);
      JS_ToInt32(ctx, &n, v);
      ref[i] = n;
    }
  }
}

template<class T>
inline void
jsrt::get_point(JSValueConst val, T& ref) {
  bool arr = JS_IsArray(ctx, val);
  JSValue vx, vy;
  double x = 0, y = 0;
  if(arr) {
    int32_t length;
    JSValue lval = JS_GetPropertyStr(ctx, val, "length");
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
jsrt::get_point_array(JSValueConst val, std::vector<T>& ref) {
  int32_t i, n, length = 0, arr = JS_IsArray(ctx, val);
  if(arr) {
    JSValue lval = JS_GetPropertyStr(ctx, val, "length");
    JS_ToInt32(ctx, &length, lval);
    ref.resize(length);

    for(i = 0; i < length; i++) {
      JSValue v = JS_GetPropertyUint32(ctx, val, i);

      get_point(v, ref[i]);
    }
  }
}

template<>
inline JSValue
jsrt::create<double>(double num) {
  return JS_NewFloat64(ctx, num);
}
template<>
inline JSValue
jsrt::create<float>(float num) {
  return create(static_cast<double>(num));
}
template<>
inline JSValue
jsrt::create<int>(int num) {
  return JS_NewInt32(ctx, num);
}
template<>
inline JSValue
jsrt::create<const char*>(const char* str) {
  return JS_NewString(ctx, str);
}
template<>
inline JSValue
jsrt::create<bool>(bool b) {
  return b ? get_true() : get_false();
}

inline JSValue
jsrt::create_array(int32_t size) {
  JSValue ret = JS_NewArray(ctx);
  if(size >= 0)
    JS_SetPropertyStr(ctx, ret, "length", JS_NewInt32(ctx, size));

  return ret;
}

template<class T>
inline JSValue
jsrt::create(T arg) {
  return std::is_pointer<T>::value && arg == nullptr ? get_null() : get_undefined();
}

template<class T>
inline JSValue
jsrt::create_point(T x, T y) {
  JSValue obj = JS_NewObject(ctx);
  JS_SetPropertyStr(ctx, obj, "x", create(x));
  JS_SetPropertyStr(ctx, obj, "y", create(y));
  return obj;
}

inline void
jsrt::set_property(JSValueConst obj, const char* name, JSValue val) {
  JS_SetPropertyStr(ctx, obj, name, val);
}

inline void
jsrt::set_property(JSValueConst obj, uint32_t index, JSValue val) {
  JS_SetPropertyUint32(ctx, obj, index, val);
}

inline JSValue
jsrt::get_global_property(const char* name) {
  global_object global = get_global_object();
  JSValue ret = JS_GetPropertyStr(ctx, global.val, name);
  return ret;
}

inline JSValue
jsrt::call(JSValueConst func, std::vector<JSValueConst>& args) {
  global_object global = get_global_object();
  JSValue ret = JS_Call(ctx, func, global.val, args.size(), args.data());
  return ret;
}

template<class T>
inline JSValue
vector_to_js(jsrt& js, const T& v, size_t n, const std::function<JSValue(const typename T::value_type&)>& fn) {
  using std::placeholders::_1;
  JSValue ret = js.create_array(n);
  for(uint32_t i = 0; i < n; i++) js.set_property(ret, i, fn(v[i]));
  return ret;
}

template<class T>
inline JSValue
vector_to_js(jsrt& js, const T& v, size_t n) {
  return vector_to_js(v, n, std::bind(&jsrt::create<typename T::value_type>, &js, std::placeholders::_1));
}

template<class T>
inline JSValue
vector_to_js(jsrt& js,const T& v) {
  return vector_to_js(js, v, v.size());
}

template<class P>
inline JSValue
vector_to_js(jsrt& js, const std::vector<P>& v, const std::function<JSValue(const P&)>& fn) {
  return vector_to_js(js, v, v.size(), fn);
}

template<class P>
inline JSValue
pointer_to_js(jsrt& js, const P* v, size_t n, const std::function<JSValue(const P&)>& fn) {
  JSValue ret = js.create_array(n);
  for(uint32_t i = 0; i < n; i++) js.set_property(ret, i, fn(v[i]));
  return ret;
}


template<class P>
inline JSValue
pointer_to_js(jsrt& js, const P* v, size_t n) {
std::function<JSValue(const P&)> fn([&](const P& v) -> JSValue {
  return js.create(v);
});

 return pointer_to_js(js, v, n, fn);
}

template<class P>
inline JSValue
vector_to_js(jsrt& js, const std::vector<P>& v, JSValue (*fn)(const P&)) {
  uint32_t i, n = v.size();
  JSValue ret = js.create_array(n);
  for(i = 0; i < n; i++) js.set_property(ret, i, fn(v[i]));
  return ret;
}
template<class P>
inline JSValue
vector_to_js(jsrt& js, const std::vector<P>& v) {
  using std::placeholders::_1;
  return vector_to_js<P>(v, std::bind(&jsrt::create<P>, &js, _1));
}

#endif // defined JS_H