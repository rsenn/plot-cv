#include <dlfcn.h>

int
main() {
  void* dll = dlopen("build/x86_64-w64-mingw32/contour.dll", RTLD_NOW);

  int* ptr = dlsym(dll, "js_contour_class_id");

  printf("js_contour_class_id = %d\n", *ptr);
}
