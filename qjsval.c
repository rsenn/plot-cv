#include <quickjs/quickjs-config.h>
#include <quickjs/quickjs.h>
#include "quickjs/qjs-modules/quickjs-internal.h"

int
main() {
  printf("\nJSContext %zu\n", sizeof(JSContext));
  printf(".loaded_modules %zu %zu\n", offsetof(JSContext, loaded_modules), sizeof(((JSContext*)0)->loaded_modules));
  printf(".user_opaque %zu %zu\n", offsetof(JSContext, user_opaque), sizeof(((JSContext*)0)->user_opaque));

  printf("\nJSModuleDef %zu\n", sizeof(JSModuleDef));
  printf(".header.ref_count %zu %zu\n", offsetof(JSModuleDef, header.ref_count), sizeof(((JSModuleDef*)0)->header.ref_count));
  printf(".module_name %zu %zu\n", offsetof(JSModuleDef, module_name), sizeof(((JSModuleDef*)0)->module_name));
  printf(".link %zu %zu\n", offsetof(JSModuleDef, link), sizeof(((JSModuleDef*)0)->link));
  printf(".req_module_entries %zu %zu\n",
         offsetof(JSModuleDef, req_module_entries),
         sizeof(((JSModuleDef*)0)->req_module_entries));
  printf(".req_module_entries_count %zu %zu\n",
         offsetof(JSModuleDef, req_module_entries_count),
         sizeof(((JSModuleDef*)0)->req_module_entries_count));
  printf(".req_module_entries_size %zu %zu\n",
         offsetof(JSModuleDef, req_module_entries_size),
         sizeof(((JSModuleDef*)0)->req_module_entries_size));

  printf(".export_entries %zu %zu\n", offsetof(JSModuleDef, export_entries), sizeof(((JSModuleDef*)0)->export_entries));
  printf(".export_entries_count %zu %zu\n",
         offsetof(JSModuleDef, export_entries_count),
         sizeof(((JSModuleDef*)0)->export_entries_count));
  printf(".export_entries_size %zu %zu\n",
         offsetof(JSModuleDef, export_entries_size),
         sizeof(((JSModuleDef*)0)->export_entries_size));

  printf(".module_ns %zu %zu\n", offsetof(JSModuleDef, module_ns), sizeof(((JSModuleDef*)0)->module_ns));

  printf("\nJSExportEntry %zu\n", sizeof(JSExportEntry));
  printf(".local_name %zu %zu\n", offsetof(JSExportEntry, local_name), sizeof(((JSExportEntry*)0)->local_name));
  printf(".export_name %zu %zu\n", offsetof(JSExportEntry, export_name), sizeof(((JSExportEntry*)0)->export_name));

  printf("\nJSVarRef %zu\n", sizeof(JSVarRef));
  printf(".pvalue %zu %zu\n", offsetof(JSVarRef, pvalue), sizeof(((JSVarRef*)0)->pvalue));
  printf(".value %zu %zu\n", offsetof(JSVarRef, value), sizeof(((JSVarRef*)0)->value));
  return 0;
}
