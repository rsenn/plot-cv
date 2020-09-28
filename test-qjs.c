#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <inttypes.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>

#include "cutils.h"
#include "quickjs.h"
#include "quickjs-libc.h"

static int
eval_buf(JSContext* ctx, const void* buf, int buf_len, const char* filename, int eval_flags) {
  JSValue val;
  int ret;

  if((eval_flags & JS_EVAL_TYPE_MASK) == JS_EVAL_TYPE_MODULE) {
    /* for the modules, we compile then run to be able to set
       import.meta */
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

static int
eval_file(JSContext* ctx, const char* filename, int module) {
  uint8_t* buf;
  int ret, eval_flags;
  size_t buf_len;

  buf = js_load_file(ctx, &buf_len, filename);
  if(!buf) {
    perror(filename);
    exit(1);
  }

  if(module < 0) {
    module = (has_suffix(filename, ".mjs") || JS_DetectModule((const char*)buf, buf_len));
  }
  if(module)
    eval_flags = JS_EVAL_TYPE_MODULE;
  else
    eval_flags = JS_EVAL_TYPE_GLOBAL;
  ret = eval_buf(ctx, buf, buf_len, filename, eval_flags);
  js_free(ctx, buf);
  return ret;
}

/* also used to initialize the worker context */
static JSContext*
JS_NewCustomContext(JSRuntime* rt) {
  JSContext* ctx;
  ctx = JS_NewContext(rt);
  if(!ctx)
    return NULL;
#ifdef CONFIG_BIGNUM
  if(1) {
    JS_AddIntrinsicBigFloat(ctx);
    JS_AddIntrinsicBigDecimal(ctx);
    JS_AddIntrinsicOperators(ctx);
    JS_EnableBignumExt(ctx, TRUE);
  }
#endif
  /* system modules */
  js_init_module_std(ctx, "std");
  js_init_module_os(ctx, "os");
  return ctx;
}

int
main(int argc, char** argv) {
  JSRuntime* rt;
  JSContext* ctx;
  typedef JSContext* ContextFunction(JSRuntime*);

  ContextFunction* newContext = &JS_NewContext;
  rt = JS_NewRuntime();

  if(!rt) {
    fprintf(stderr, "qjs: cannot allocate JS runtime\n");
    exit(2);
  }

  js_std_set_worker_new_context_func(newContext);
  js_std_init_handlers(rt);
  ctx = newContext(rt);
  if(!ctx) {
    fprintf(stderr, "qjs: cannot allocate JS context\n");
    exit(2);
  }

  /* loader for ES6 modules */
  JS_SetModuleLoaderFunc(rt, NULL, js_module_loader, NULL);

  js_std_add_helpers(ctx, argc, argv);

  if(eval_file(ctx, "quickjs/examples/hello.js", 0))
    goto fail;

  js_std_loop(ctx);

  js_std_free_handlers(rt);
  JS_FreeContext(ctx);
  JS_FreeRuntime(rt);

  return 0;
fail:
  js_std_free_handlers(rt);
  JS_FreeContext(ctx);
  JS_FreeRuntime(rt);
  return 1;
}
