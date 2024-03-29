#define CONFIG_VERSION "2021-03-27"
#define CONFIG_BIGNUM 1

#include <quickjs/quickjs-config.h>

#include <stdio.h>
#include "quickjs/libbf.h"
#include "quickjs/quickjs.h"
#include "quickjs/cutils.c"
#include "quickjs/qjs-modules/quickjs-internal.h"

int
main(int argc, char* argv[]) {
  printf("sizeof(JSModuleDef) = %d\n", sizeof(JSModuleDef));
  printf("sizeof(JSObject) = %d\n", sizeof(JSObject));

  printf("JS_CLASS_OBJECT = %d\n", JS_CLASS_OBJECT);
  printf("JS_CLASS_ARRAY = %d\n", JS_CLASS_ARRAY);
  printf("JS_CLASS_ERROR = %d\n", JS_CLASS_ERROR);
  printf("JS_CLASS_NUMBER = %d\n", JS_CLASS_NUMBER);
  printf("JS_CLASS_STRING = %d\n", JS_CLASS_STRING);
  printf("JS_CLASS_BOOLEAN = %d\n", JS_CLASS_BOOLEAN);
  printf("JS_CLASS_SYMBOL = %d\n", JS_CLASS_SYMBOL);
  printf("JS_CLASS_ARGUMENTS = %d\n", JS_CLASS_ARGUMENTS);
  printf("JS_CLASS_MAPPED_ARGUMENTS = %d\n", JS_CLASS_MAPPED_ARGUMENTS);
  printf("JS_CLASS_DATE = %d\n", JS_CLASS_DATE);
  printf("JS_CLASS_MODULE_NS = %d\n", JS_CLASS_MODULE_NS);
  printf("JS_CLASS_C_FUNCTION = %d\n", JS_CLASS_C_FUNCTION);
  printf("JS_CLASS_BYTECODE_FUNCTION = %d\n", JS_CLASS_BYTECODE_FUNCTION);
  printf("JS_CLASS_BOUND_FUNCTION = %d\n", JS_CLASS_BOUND_FUNCTION);
  printf("JS_CLASS_C_FUNCTION_DATA = %d\n", JS_CLASS_C_FUNCTION_DATA);
  printf("JS_CLASS_GENERATOR_FUNCTION = %d\n", JS_CLASS_GENERATOR_FUNCTION);
  printf("JS_CLASS_FOR_IN_ITERATOR = %d\n", JS_CLASS_FOR_IN_ITERATOR);
  printf("JS_CLASS_REGEXP = %d\n", JS_CLASS_REGEXP);
  printf("JS_CLASS_ARRAY_BUFFER = %d\n", JS_CLASS_ARRAY_BUFFER);
  printf("JS_CLASS_SHARED_ARRAY_BUFFER = %d\n", JS_CLASS_SHARED_ARRAY_BUFFER);
  printf("JS_CLASS_UINT8C_ARRAY = %d\n", JS_CLASS_UINT8C_ARRAY);
  printf("JS_CLASS_INT8_ARRAY = %d\n", JS_CLASS_INT8_ARRAY);
  printf("JS_CLASS_UINT8_ARRAY = %d\n", JS_CLASS_UINT8_ARRAY);
  printf("JS_CLASS_INT16_ARRAY = %d\n", JS_CLASS_INT16_ARRAY);
  printf("JS_CLASS_UINT16_ARRAY = %d\n", JS_CLASS_UINT16_ARRAY);
  printf("JS_CLASS_INT32_ARRAY = %d\n", JS_CLASS_INT32_ARRAY);
  printf("JS_CLASS_UINT32_ARRAY = %d\n", JS_CLASS_UINT32_ARRAY);
  printf("JS_CLASS_BIG_INT64_ARRAY = %d\n", JS_CLASS_BIG_INT64_ARRAY);
  printf("JS_CLASS_BIG_UINT64_ARRAY = %d\n", JS_CLASS_BIG_UINT64_ARRAY);
  printf("JS_CLASS_FLOAT32_ARRAY = %d\n", JS_CLASS_FLOAT32_ARRAY);
  printf("JS_CLASS_FLOAT64_ARRAY = %d\n", JS_CLASS_FLOAT64_ARRAY);
  printf("JS_CLASS_DATAVIEW = %d\n", JS_CLASS_DATAVIEW);
  printf("JS_CLASS_BIG_INT = %d\n", JS_CLASS_BIG_INT);
  printf("JS_CLASS_BIG_FLOAT = %d\n", JS_CLASS_BIG_FLOAT);
  printf("JS_CLASS_FLOAT_ENV = %d\n", JS_CLASS_FLOAT_ENV);
  printf("JS_CLASS_BIG_DECIMAL = %d\n", JS_CLASS_BIG_DECIMAL);
  printf("JS_CLASS_OPERATOR_SET = %d\n", JS_CLASS_OPERATOR_SET);
  printf("JS_CLASS_MAP = %d\n", JS_CLASS_MAP);
  printf("JS_CLASS_SET = %d\n", JS_CLASS_SET);
  printf("JS_CLASS_WEAKMAP = %d\n", JS_CLASS_WEAKMAP);
  printf("JS_CLASS_WEAKSET = %d\n", JS_CLASS_WEAKSET);
  printf("JS_CLASS_MAP_ITERATOR = %d\n", JS_CLASS_MAP_ITERATOR);
  printf("JS_CLASS_SET_ITERATOR = %d\n", JS_CLASS_SET_ITERATOR);
  printf("JS_CLASS_ARRAY_ITERATOR = %d\n", JS_CLASS_ARRAY_ITERATOR);
  printf("JS_CLASS_STRING_ITERATOR = %d\n", JS_CLASS_STRING_ITERATOR);
  printf("JS_CLASS_REGEXP_STRING_ITERATOR = %d\n", JS_CLASS_REGEXP_STRING_ITERATOR);
  printf("JS_CLASS_GENERATOR = %d\n", JS_CLASS_GENERATOR);
  printf("JS_CLASS_PROXY = %d\n", JS_CLASS_PROXY);
  printf("JS_CLASS_PROMISE = %d\n", JS_CLASS_PROMISE);
  printf("JS_CLASS_PROMISE_RESOLVE_FUNCTION = %d\n", JS_CLASS_PROMISE_RESOLVE_FUNCTION);
  printf("JS_CLASS_PROMISE_REJECT_FUNCTION = %d\n", JS_CLASS_PROMISE_REJECT_FUNCTION);
  printf("JS_CLASS_ASYNC_FUNCTION = %d\n", JS_CLASS_ASYNC_FUNCTION);
  printf("JS_CLASS_ASYNC_FUNCTION_RESOLVE = %d\n", JS_CLASS_ASYNC_FUNCTION_RESOLVE);
  printf("JS_CLASS_ASYNC_FUNCTION_REJECT = %d\n", JS_CLASS_ASYNC_FUNCTION_REJECT);
  printf("JS_CLASS_ASYNC_FROM_SYNC_ITERATOR = %d\n", JS_CLASS_ASYNC_FROM_SYNC_ITERATOR);
  printf("JS_CLASS_ASYNC_GENERATOR_FUNCTION = %d\n", JS_CLASS_ASYNC_GENERATOR_FUNCTION);
  printf("JS_CLASS_ASYNC_GENERATOR = %d\n", JS_CLASS_ASYNC_GENERATOR);
  int divi = -9, quot = 10, result;

  result = ((divi % quot) + quot) % quot;
  printf("%d %% %d = %d\n", divi, quot, result);

  return 0;
}
