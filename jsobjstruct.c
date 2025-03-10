#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "quickjs/quickjs.h"
#include "quickjs/cutils.h"
#include "quickjs/list.h"

typedef enum {
  JS_GC_OBJ_TYPE_JS_OBJECT,
  JS_GC_OBJ_TYPE_FUNCTION_BYTECODE,
  JS_GC_OBJ_TYPE_SHAPE,
  JS_GC_OBJ_TYPE_VAR_REF,
  JS_GC_OBJ_TYPE_ASYNC_FUNCTION,
  JS_GC_OBJ_TYPE_JS_CONTEXT,
} JSGCObjectTypeEnum;

/* header for GC objects. GC objects are C data structures with a
   reference count that can reference other GC objects. JS Objects are
   a particular type of GC object. */
struct JSGCObjectHeader {
  int ref_count; /* must come first, 32-bit */
  JSGCObjectTypeEnum gc_obj_type : 4;
  uint8_t mark : 4; /* used by the GC */
  uint8_t dummy1;   /* not used by the GC */
  uint16_t dummy2;  /* not used by the GC */
  struct list_head link;
};
struct JSObject {
  union {
    JSGCObjectHeader header;
    struct {
      int __gc_ref_count; /* corresponds to header.ref_count */
      uint8_t __gc_mark;  /* corresponds to header.mark/gc_obj_type */
      union {
        struct {
          uint8_t extensible : 1;
          uint8_t free_mark : 1;  /* only used when freeing objects with cycles */
          uint8_t is_exotic : 1;  /* TRUE if object has exotic property handlers */
          uint8_t fast_array : 1; /* TRUE if u.array is used for get/put (for JS_CLASS_ARRAY,
                                     JS_CLASS_ARGUMENTS and typed arrays) */
          uint8_t is_constructor : 1;       /* TRUE if object is a constructor function */
          uint8_t is_uncatchable_error : 1; /* if TRUE, error is not catchable */
          uint8_t tmp_mark : 1;             /* used in JS_WriteObjectRec() */
          uint8_t is_HTMLDDA : 1;           /* specific annex B IsHtmlDDA behavior */
        };
        uint8_t byte;
      } __flags;
      uint16_t class_id; /* see JS_CLASS_x */
    };
  };
};

int
main() {
  struct JSObject obj;
  memset(&obj, 0, sizeof(obj));
  obj.__flags.tmp_mark = 1;
  int32_t idx = -2;
  uint32_t len = 100;

  printf("JSObject.__gc_ref_count %lu %lu\n",
         offsetof(struct JSObject, __gc_ref_count),
         sizeof(obj.__gc_ref_count));
  printf("JSObject.__flags %lu %lu\n", offsetof(struct JSObject, __flags), sizeof(obj.__flags));
  printf("JSObject.class_id %lu %lu\n",
         offsetof(struct JSObject, class_id),
         sizeof(obj.class_id));
  printf("JSObject.__flags.byte %02x\n", obj.__flags.byte);
  printf("idx %% len = %i\n", idx % (int32_t)len);
}
