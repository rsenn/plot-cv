#include "js.h"
#include <cstring>
#include <iostream>
#include <cstring>
extern "C" {
#include "../quickjs/quickjs-libc.h"
#include "../quickjs/cutils.h"
};

bool
jsrt::init(int argc, char* argv[]) {
  int load_std = 0;

  if((rt = JS_NewRuntime())) {
    if((ctx = JS_NewContext(rt))) {
      // JS_AddIntrinsicBaseObjects(ctx);
      js_std_add_helpers(ctx, argc, argv);

      /* system modules */
      js_init_module_std(ctx, "std");
      js_init_module_os(ctx, "os");

      /* make 'std' and 'os' visible to non module code */
      if(load_std) {
        const char* str = "import * as std from 'std';\n"
                          "import * as os from 'os';\n"
                          "globalThis.std = std;\n"
                          "globalThis.os = os;\n";
        eval_buf(str, strlen(str), "<input>", JS_EVAL_TYPE_MODULE);
      }

      {
        value global_obj = JS_GetGlobalObject(ctx);

        JS_SetPropertyStr(ctx,
                          global_obj,
                          "global",
                          JS_DupValue(ctx, global_obj));
        JS_FreeValue(ctx, global_obj);
      }

      /* loader for ES6 modules */
      JS_SetModuleLoaderFunc(rt, NULL, js_module_loader, NULL);
    }
  }

  global.get();

  return ctx != nullptr;
}

jsrt::value*
jsrt::get_function(const char* name) {
  auto it = funcmap.find(name);
  if(it != funcmap.end()) {
    std::pair<JSCFunction*, value>& val = it->second;
    return &val.second;
  }
  return nullptr;
}

std::string
jsrt::to_str(const_value val) {
  std::string ret;
  if(JS_IsFunction(ctx, val))
    ret = "Function";
  else if(JS_IsNumber(val))
    ret = "Number";
  else if(JS_IsBool(val))
    ret = "Boolean";
  else if(JS_IsString(val))
    ret = "String";
  else if(JS_IsArray(ctx, val))
    ret = "Array";
  else if(JS_IsObject(val))
    ret = "Object";
  else if(JS_IsSymbol(val))
    ret = "Symbol";
  else if(JS_IsException(val))
    ret = "Exception";
  else if(JS_IsUninitialized(val))
    ret = "Uninitialized";
  else if(JS_IsUndefined(val))
    ret = "undefined";

  return ret;
}

jsrt::value
jsrt::prototype(const_value obj) const {
  return JS_GetPrototype(ctx, obj);
}

std::vector<const char*>
jsrt::property_names(const_value obj, bool enum_only, bool recursive) const {
  std::vector<const char*> ret;
  property_names(obj, ret, enum_only);
  return ret;
}

void
jsrt::property_names(const_value obj,
                     std::vector<const char*>& out,
                     bool enum_only,
                     bool recursive) const {
  JSPropertyEnum* props;
  uint32_t nprops;
  while(JS_IsObject(obj)) {
    props = nullptr;
    nprops = 0;
    JS_GetOwnPropertyNames(ctx,
                           &props,
                           &nprops,
                           obj,
                           JS_GPN_STRING_MASK | JS_GPN_SYMBOL_MASK |
                               (enum_only ? JS_GPN_ENUM_ONLY : 0));
    for(uint32_t i = 0; i < nprops; i++) {
      const char* s = JS_AtomToCString(ctx, props[i].atom);
      out.push_back(s);
    }
    if(!recursive)
      break;

    obj = prototype(obj);
  }
}

jsrt::global_object::global_object(jsrt& rt) : js(rt) { get(); }

bool
jsrt::global_object::get() {
  if(js.ctx) {
    val = JS_GetGlobalObject(js.ctx);
    return true;
  }
  return false;
}

jsrt::global_object::global_object(global_object&& o) noexcept
    : val(std::move(o.val)), js(o.js) {}

jsrt::global_object::~global_object() {
  if(js.ctx)
    JS_FreeValue(js.ctx, val);
}

jsrt::value
jsrt::get_undefined() const {
  return JS_UNDEFINED;
}
jsrt::value
jsrt::get_null() const {
  return JS_NULL;
}
jsrt::value
jsrt::get_true() const {
  return JS_TRUE;
}
jsrt::value
jsrt::get_false() const {
  return JS_FALSE;
}

jsrt::~jsrt() {
  if(ctx)
    JS_FreeContext(ctx);
  if(rt)
    JS_FreeRuntime(rt);
  ctx = nullptr;
  rt = nullptr;
}

int
jsrt::eval_buf(const char* buf,
               int buf_len,
               const char* filename,
               int eval_flags) {
  value val;
  int ret;
  if((eval_flags & JS_EVAL_TYPE_MASK) == JS_EVAL_TYPE_MODULE) {
    val = JS_Eval(
        ctx, buf, buf_len, filename, eval_flags | JS_EVAL_FLAG_COMPILE_ONLY);
    if(!JS_IsException(val)) {
      js_module_set_import_meta(ctx, val, TRUE, TRUE);
      val = JS_EvalFunction(ctx, val);
    }
  } else {
    val = JS_Eval(ctx, buf, buf_len, filename, eval_flags);
  }
  if(JS_IsException(val)) {
    js_std_dump_error(ctx);
    ret = -1;
  } else {
    ret = 0;
  }
  JS_FreeValue(ctx, val);
  return ret;
}

int
jsrt::eval_file(const char* filename, int module) {
  char* buf;
  int ret, eval_flags;
  size_t buf_len;
  buf = reinterpret_cast<char*>(js_load_file(ctx, &buf_len, filename));
  if(!buf) {
    perror(filename);
    exit(1);
  }
  if(module < 0)
    module = (has_suffix(filename, ".mjs") ||
              JS_DetectModule((const char*)buf, buf_len));

  eval_flags = module ? JS_EVAL_TYPE_MODULE : JS_EVAL_TYPE_GLOBAL;

  /* std::string script(buf, buf_len);
   std::cerr << "Script: " << script << std::endl;*/

  ret = eval_buf(buf, buf_len, filename, eval_flags);
  js_free(ctx, buf);
  return ret;
}

jsrt::value
jsrt::add_function(const char* name, JSCFunction* fn, int args) {
  global_object& global = get_global_object();
  value function = JS_NewCFunction(ctx, fn, name, args);

  funcmap[name] = std::make_pair(fn, function);

  JS_SetPropertyStr(ctx, global, name, function);
  return function;
}

jsrt::value
jsrt::get_global(const char* name) {
  const_value global = JS_GetGlobalObject(ctx);
  value ret = JS_GetPropertyStr(ctx, global, name);
  return ret;
}

jsrt::value
jsrt::call(const char* name, size_t argc, const_value* argv) {
  const_value func = get_global(name);
  return call(func, argc, argv);
}

jsrt::value
jsrt::call(const_value func, std::vector<const_value>& args) {
  return call(func, args.size(), args.data());
}

jsrt::value
jsrt::call(const_value func, size_t argc, const_value* argv) {
  global_object& global = get_global_object();
  value ret = JS_Call(ctx, func, global, argc, argv);
  return ret;
}
