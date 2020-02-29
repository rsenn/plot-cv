#ifndef JS_H
#define JS_H

#include "../quickjs/quickjs.h"
#include <unordered_map>
#include <vector>

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

  JSValue
  create_array(int32_t size = -1) {
    JSValue ret = JS_NewArray(ctx);
    if(size >= 0)
      JS_SetPropertyStr(ctx, ret, "length", JS_NewInt32(ctx, size));
    return ret;
  }
  JSValue
  create_number(double num) {
    return JS_NewFloat64(ctx, num);
  }
  JSValue
  create_number(int num) {
    return JS_NewInt32(ctx, num);
  }

  template<class T>
  JSValue
  create_point(T x, T y) {
    JSValue obj = JS_NewObject(ctx);

    JS_SetPropertyStr(ctx, obj, "x", create_number(x));
    JS_SetPropertyStr(ctx, obj, "y", create_number(y));
    return obj;
  }

  void
  set_property(JSValueConst obj, const char* name, JSValue val) {
    JS_SetPropertyStr(ctx, obj, name, val);
  }
  void
  set_property(JSValueConst obj, uint32_t index, JSValue val) {
    JS_SetPropertyUint32(ctx, obj, index, val);
  }

  JSValue
  get_global_property(const char* name) {
    global_object global = get_global_object();
    JSValue ret = JS_GetPropertyStr(ctx, global.val, name);
    return ret;
  }

  JSValue
  call(JSValueConst func, std::vector<JSValueConst>& args) {
    global_object global = get_global_object();
    JSValue ret = JS_Call(ctx, func, global.val, args.size(), args.data());
    return ret;
  }

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

#endif // defined JS_H