#include "js.h"
#include <cstring>
#include <iostream>
#include <cstring>
#include <filesystem>
#include <cassert>

extern "C" {
#include "quickjs/quickjs-libc.h"
#include "quickjs/cutils.h"

jsrt js;


char* normalize_module(JSContext* ctx, const char* module_base_name, const char* module_name, void* opaque);
};

static JSValue
js_print(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  int i;
  const char* str;

  for(i = 0; i < argc; i++) {
    if(i != 0)
      putchar(' ');
    str = JS_ToCString(ctx, argv[i]);
    if(!str)
      return JS_EXCEPTION;
    fputs(str, stdout);
    JS_FreeCString(ctx, str);
  }
  putchar('\n');
  return JS_UNDEFINED;
}

struct JSProperty;
struct JSShapeProperty;

bool
jsrt::init(int argc, char* argv[]) {
  int load_std = 0;

  if(ctx == nullptr)
    if(!create())
      return false;

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

  /* loader for ES6 modules */
  JS_SetModuleLoaderFunc(rt, &normalize_module, js_module_loader, this);

  global.get();

  this->_undefined = get_undefined();
  this->_null = get_null();
  this->_true = get_true();
  this->_false = get_false();

  return ctx != nullptr;
}

bool
jsrt::create(JSContext* _ctx) {
  assert(ctx == nullptr);
  if(_ctx) {
    rt = JS_GetRuntime(ctx = _ctx);
  } else {
    if((rt = JS_NewRuntime()))
      ctx = JS_NewContext(rt);
  }
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
jsrt::property_names(const_value obj, std::vector<const char*>& out, bool enum_only, bool recursive) const {
  JSPropertyEnum* props;
  uint32_t nprops;
  while(JS_IsObject(obj)) {
    props = nullptr;
    nprops = 0;
    JS_GetOwnPropertyNames(ctx, &props, &nprops, obj, JS_GPN_STRING_MASK | JS_GPN_SYMBOL_MASK | (enum_only ? JS_GPN_ENUM_ONLY : 0));
    for(uint32_t i = 0; i < nprops; i++) {
      const char* s = JS_AtomToCString(ctx, props[i].atom);
      out.push_back(s);
    }
    if(!recursive)
      break;

    obj = prototype(obj);
  }
}

bool
jsrt::is_point(const_value val) const {
  if(is_array(val)) {
    int32_t length = -1;
    get_number(get_property(val, "length"), length);
    if(length == 2)
      return true;
  } else if(is_object(val)) {
    JSValue x, y;
    x = get_property(val, "x");
    y = get_property(val, "y");
    if(is_number(x) && is_number(y))
      return true;
  }

  return false;
}

bool
jsrt::is_rect(const_value val) const {
  if(is_object(val)) {
    JSValue x, y, w, h;
    x = get_property(val, "x");
    y = get_property(val, "y");
    w = get_property(val, "width");
    h = get_property(val, "height");
    if(is_number(x) && is_number(y))
      if(is_number(w) && is_number(h))
        return true;
  }

  return false;
}

bool
jsrt::is_color(const_value val) const {
  JSValue b = _undefined, g = _undefined, r = _undefined, a = _undefined;

  if(is_array_like(val)) {
    uint32_t length;
    get_number(get_property(val, "length"), length);

    if(length == 3 || length == 4) {
      b = get_property<uint32_t>(val, 0);
      g = get_property<uint32_t>(val, 1);
      r = get_property<uint32_t>(val, 2);
      a = length > 3 ? get_property<uint32_t>(val, 3) : const_cast<jsrt*>(this)->create<int32_t>(255);
    } else {
      return false;
    }
  } else if(is_object(val)) {
    b = get_property(val, "b");
    g = get_property(val, "g");
    r = get_property(val, "r");
    a = get_property(val, "a");
  }

  if(is_number(b) && is_number(g) && is_number(r) && is_number(a))
    return true;
  return false;
}

jsrt::global::global(jsrt& rt) : js(rt) { get(); }

bool
jsrt::global::get() {
  if(js.ctx) {
    value global = JS_GetGlobalObject(js.ctx);

    JS_SetPropertyStr(js.ctx, global, "global", JS_DupValue(js.ctx, global));
    JS_FreeValue(js.ctx, global);
  }

  if(js.ctx) {
    val = JS_GetGlobalObject(js.ctx);
    return true;
  }
  return false;
}

jsrt::global::global(global&& o) noexcept : val(std::move(o.val)), js(o.js) {}

jsrt::global::~global() {
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
  if(ctx) {
    JS_FreeContext(ctx);

    /*  if(rt)
        JS_FreeRuntime(rt);*/
  }
  ctx = nullptr;
  rt = nullptr;
}

int
jsrt::eval_buf(const char* buf, int buf_len, const char* filename, int eval_flags) {
  value val;
  int ret;
  if((eval_flags & JS_EVAL_TYPE_MASK) == JS_EVAL_TYPE_MODULE) {
    val = JS_Eval(ctx, buf, buf_len, filename, eval_flags | JS_EVAL_FLAG_COMPILE_ONLY);
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
    module = (has_suffix(filename, ".mjs") || JS_DetectModule((const char*)buf, buf_len));

  eval_flags = module ? JS_EVAL_TYPE_MODULE : JS_EVAL_TYPE_GLOBAL;

 /*  std::string script(buf, buf_len);
   std::cerr << "Script: " << script << std::endl;
*/
  ret = eval_buf(buf, buf_len, filename, eval_flags);
  js_free(ctx, buf);
  return ret;
}

jsrt::value
jsrt::add_function(const char* name, JSCFunction* fn, int args) {
  value function = JS_NewCFunction(ctx, fn, name, args);

  funcmap[name] = std::make_pair(fn, function);

  JS_SetPropertyStr(ctx, global_object(), name, function);
  return function;
}

void
jsrt::set_global(const char* name, JSValue val) {

  JS_SetPropertyStr(ctx, global_object(), name, val);
}

jsrt::value
jsrt::get_global(const char* name) {
  value ret = JS_GetPropertyStr(ctx, global_object(), name);
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
  value ret = JS_Call(ctx, func, global_object(), argc, argv);
  if(JS_IsException(ret))
    dump_error();
  return ret;
}

extern "C" char*
normalize_module(JSContext* ctx, const char* module_base_name, const char* module_name, void* opaque) {
  using std::filesystem::path;
  using std::filesystem::weakly_canonical;

  char* name;
  jsrt* js = static_cast<jsrt*>(opaque);
  /*
    std::cerr << "module_base_name: " << module_base_name << std::endl;
    std::cerr << "module_name: " << module_name << std::endl;
  */
  if(module_name[0] == '.' && module_name[1] == '/')
    module_name += 2;

  path module_path = path(module_base_name).replace_filename(path(module_name, module_name + strlen(module_name)));

  // std::cerr << "module_path: " << module_path.string() << std::endl;

  bool exists = std::filesystem::exists(module_path);
  // std::cerr << "exists module_path: " << exists << std::endl;

  if(!exists) {
    module_path = weakly_canonical(module_path);

    exists = std::filesystem::exists(module_path);
  }

  if(exists) {
    std::string module_pathstr;

    module_pathstr = module_path.string();
    /*
    module_pathstr.resize(module_pathstr.size()+1);
    */
    name = static_cast<char*>(js_malloc(ctx, module_pathstr.size() + 1));
    strcpy(name, module_pathstr.c_str());

    // std::cerr << "name: " << name << std::endl;
  }
  return name;
}

void
jsrt::dump_error() {
  JSValue exception_val;

  exception_val = JS_GetException(ctx);
  dump_exception(exception_val, true);
  JS_FreeValue(ctx, exception_val);
}

void
jsrt::dump_exception(JSValueConst exception_val, bool is_throw) {
  JSValue val;
  const char* stack;
  bool is_error;

  is_error = JS_IsError(ctx, exception_val);
  /*  if(is_throw && !is_error)
      printf("Throw: ");*/
  js_print(ctx, JS_NULL, 1, (JSValueConst*)&exception_val);
  if(is_error) {
    val = JS_GetPropertyStr(ctx, exception_val, "stack");
    if(!JS_IsUndefined(val)) {
      stack = JS_ToCString(ctx, val);
      printf("%s\n", stack);
      JS_FreeCString(ctx, stack);
    }
    JS_FreeValue(ctx, val);
  }
}
