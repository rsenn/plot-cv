#define CONFIG_VERSION "2021-03-27"

#include <stdio.h>
#include "quickjs/quickjs.h"
#include "quickjs/cutils.c"
#include "quickjs/quickjs.c"

int
main(int argc, char* argv[]) {
  printf("sizeof(JSModuleDef) = %d\n", sizeof(JSModuleDef));
  printf("sizeof(JSObject) = %d\n", sizeof(JSObject));

  return 0;
}
