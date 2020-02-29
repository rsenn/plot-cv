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
        JSValue global_obj = JS_GetGlobalObject(ctx);

        JS_SetPropertyStr(ctx, global_obj, "global", JS_DupValue(ctx, global_obj));
        JS_FreeValue(ctx, global_obj);
      }
    }
  }
  return ctx != nullptr;
}

jsrt::~jsrt() {
  JS_FreeContext(ctx);
  JS_FreeRuntime(rt);
  ctx = nullptr;
  rt = nullptr;
}

int
jsrt::eval_buf(const char* buf, int buf_len, const char* filename, int eval_flags) {
  JSValue val;
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

  {
    std::string script(buf, buf_len);

    std::cerr << "Script: " << script << std::endl;

    ret = eval_buf(buf, buf_len, filename, eval_flags);
    js_free(ctx, buf);
    return ret;
  }
}

JSValue
jsrt::add_function(const char* name, JSCFunction* fn, int args) {
  global_object global = get_global_object();
  JSValue function = JS_NewCFunction(ctx, fn, name, args);

  funcmap[name] = std::make_pair(fn, function);

  JS_SetPropertyStr(ctx, global.val, name, function);
  return function;
}
