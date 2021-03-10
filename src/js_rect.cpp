#include "jsbindings.hpp"
#include "js_rect.hpp"
#include "js_size.hpp"
#include "js_point.hpp"
#include "js_alloc.hpp"
#include "js_array.hpp"
#include "js_typed_array.hpp"
#include "util.hpp"

#if defined(JS_RECT_MODULE) || defined(quickjs_rect_EXPORTS)
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module
#else
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module_rect
#endif

extern "C" {
JSValue rect_proto = JS_UNDEFINED, rect_class = JS_UNDEFINED;
JSClassID js_rect_class_id = 0;
}

VISIBLE JSValue
js_rect_new(JSContext* ctx, double x, double y, double w, double h) {
  JSValue ret;
  JSRectData<double>* s;

  if(JS_IsUndefined(rect_proto))
    js_rect_init(ctx, NULL);

  ret = JS_NewObjectProtoClass(ctx, rect_proto, js_rect_class_id);

  s = js_allocate<JSRectData<double>>(ctx);

  new(s) JSRectData<double>();
  s->x = x;
  s->y = y;
  s->width = w;
  s->height = h;

  JS_SetOpaque(ret, s);
  return ret;
}

VISIBLE JSValue
js_rect_wrap(JSContext* ctx, const JSRectData<double>& rect) {
  return js_rect_new(ctx, rect.x, rect.y, rect.width, rect.height);
}

static JSValue
js_rect_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  double x, y, w, h;
  JSRectData<double> rect;

  if(argc > 0) {
    if(!js_rect_read(ctx, argv[0], &rect)) {
      JSPointData<double> point;
      if(argc >= 2 && js_point_read(ctx, argv[0], &point)) {
        JSSizeData<double> size;
        JSPointData<double> point2;

        x = point.x;
        y = point.y;
        if(js_size_read(ctx, argv[1], &size)) {
          w = size.width;
          h = size.height;
        } else if(js_point_read(ctx, argv[1], &point2)) {
          w = point2.x - point.x;
          h = point2.y - point.y;
        } else {
          return JS_EXCEPTION;
        }
      } else {
        if(JS_ToFloat64(ctx, &x, argv[0]))
          return JS_EXCEPTION;
        if(argc < 2 || JS_ToFloat64(ctx, &y, argv[1]))
          return JS_EXCEPTION;
        if(argc < 3 || JS_ToFloat64(ctx, &w, argv[2]))
          return JS_EXCEPTION;
        if(argc < 4 || JS_ToFloat64(ctx, &h, argv[3]))
          return JS_EXCEPTION;
      }
    }
  }

  return js_rect_new(ctx, x, y, w, h);
}

VISIBLE JSRectData<double>*
js_rect_data(JSContext* ctx, JSValueConst val) {
  return static_cast<JSRectData<double>*>(JS_GetOpaque2(ctx, val, js_rect_class_id));
}

void
js_rect_finalizer(JSRuntime* rt, JSValue val) {
  JSRectData<double>* s = static_cast<JSRectData<double>*>(JS_GetOpaque(val, js_rect_class_id));
  /* Note: 's' can be NULL in case JS_SetOpaque() was not called */

  s->~JSRectData<double>();
  js_deallocate(rt, s);
}

static JSValue
js_rect_get(JSContext* ctx, JSValueConst this_val, int magic) {
  JSValue ret = JS_UNDEFINED;
  JSRectData<double>* s = static_cast<JSRectData<double>*>(JS_GetOpaque2(ctx, this_val, js_rect_class_id));
  if(!s)
    ret = JS_EXCEPTION;
  else if(magic == 0)
    ret = JS_NewFloat64(ctx, s->x);
  else if(magic == 1)
    ret = JS_NewFloat64(ctx, s->y);
  else if(magic == 2)
    ret = JS_NewFloat64(ctx, s->width);
  else if(magic == 3)
    ret = JS_NewFloat64(ctx, s->height);
  else if(magic == 4)
    ret = JS_NewFloat64(ctx, s->x + s->width);
  else if(magic == 5)
    ret = JS_NewFloat64(ctx, s->y + s->height);
  else if(magic == 6)
    ret = js_point_new(ctx, s->x, s->y);
  else if(magic == 7)
    ret = js_size_new(ctx, s->width, s->height);
  return ret;
}

static JSValue
js_rect_set(JSContext* ctx, JSValueConst this_val, JSValueConst val, int magic) {
  JSRectData<double>* s = static_cast<JSRectData<double>*>(JS_GetOpaque2(ctx, this_val, js_rect_class_id));
  double v;
  if(!s)
    return JS_EXCEPTION;
  if(JS_ToFloat64(ctx, &v, val))
    return JS_EXCEPTION;
  if(magic == 0)
    s->x = v;
  else if(magic == 1)
    s->y = v;
  else if(magic == 2)
    s->width = v;
  else if(magic == 3)
    s->height = v;
  else if(magic == 4)
    s->width = v - s->x;
  else if(magic == 5)
    s->height = v - s->y;
  else if(magic == 6) {
    JSPointData<double> point;
    js_point_read(ctx, val, &point);
    s->x = point.x;
    s->y = point.y;
  } else if(magic == 7) {
    JSSizeData<double> size;
    js_size_read(ctx, val, &size);
    s->width = size.width;
    s->height = size.height;
  }
  return JS_UNDEFINED;
}

static JSValue
js_rect_to_string(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSRectData<double> rect, *s;
  std::ostringstream os;
  std::array<const char*, 3> delims = {
      ",",
      "∣" /*｜⧸⦁⎮∥∣⸾⼁❘❙⟊⍿⎸⏐｜│￨︲︱❘|｜*/,
      "×" /*"𝅃🅧𝚡🅧🅇𝘹𝚡𝘹𝐱ꭗ𝐗𝑿𝅃𝅃xˣₓ⒳Ⓧⓧ✕✘✗⨉⨯⨂✖⨻⦁⋅⊗⊠∗×⨯×"*/};

  for(size_t i = 0; i < argc; i++) { delims[i] = JS_ToCString(ctx, argv[i]); }

  if((s = static_cast<JSRectData<double>*>(JS_GetOpaque2(ctx, this_val, js_rect_class_id))) != nullptr) {
    rect = *s;
  } else {
    js_rect_read(ctx, this_val, &rect);
  }

  os << rect.x << delims[0] << rect.y << delims[1] << rect.width << delims[2] << rect.height;

  return JS_NewString(ctx, os.str().c_str());
}
static JSValue
js_rect_to_source(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSRectData<double> rect, *s;
  std::ostringstream os;

  if((s = static_cast<JSRectData<double>*>(JS_GetOpaque2(ctx, this_val, js_rect_class_id))) != nullptr) {
    rect = *s;
  } else {
    js_rect_read(ctx, this_val, &rect);
  }

  os << "{x:" << rect.x << ",y:" << rect.y << ",width:" << rect.width << ",height:" << rect.height << "}";

  return JS_NewString(ctx, os.str().c_str());
}

static JSValue
js_rect_funcs(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv, int magic) {
  JSRectData<double> rect, other, *s, *a;
  JSValue ret = JS_EXCEPTION;

  if((s = js_rect_data(ctx, this_val)) == nullptr)
    return ret;

  rect = *s;

  if(magic == 0) {
    js_rect_read(ctx, argv[0], &other);
    bool equals = rect.x == other.x && rect.y == other.y && rect.width == other.width && rect.height == other.height;
    ret = JS_NewBool(ctx, equals);
  } else if(magic == 1) {
    double x, y, width, height, f;
    int32_t precision = 0;
    if(argc > 0)
      JS_ToInt32(ctx, &precision, argv[0]);
    f = std::pow(10, precision);
    x = std::round(rect.x * f) / f;
    y = std::round(rect.y * f) / f;
    width = std::round(rect.width * f) / f;
    height = std::round(rect.height * f) / f;
    ret = js_rect_new(ctx, x, y, width, height);
  } else if(magic == 2) {
    ret = JS_NewObject(ctx);

    JS_SetPropertyStr(ctx, ret, "x", JS_NewFloat64(ctx, rect.x));
    JS_SetPropertyStr(ctx, ret, "y", JS_NewFloat64(ctx, rect.y));
    JS_SetPropertyStr(ctx, ret, "width", JS_NewFloat64(ctx, rect.width));
    JS_SetPropertyStr(ctx, ret, "height", JS_NewFloat64(ctx, rect.height));
  } else if(magic == 3) {
    std::array<double, 4> array{rect.x, rect.y, rect.width, rect.height};

    ret = js_array_from(ctx, array.cbegin(), array.cend());
  }
  return ret;
}

static JSValue
js_rect_inspect(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSRectData<double> rect, *s;
  std::ostringstream os;

  if((s = static_cast<JSRectData<double>*>(JS_GetOpaque2(ctx, this_val, js_rect_class_id))) != nullptr) {
    rect = *s;
  } else {
    js_rect_read(ctx, this_val, &rect);
  }

  os << "{ x: " COLOR_YELLOW "" << rect.x << "" COLOR_NONE ", y: " COLOR_YELLOW "" << rect.y << "" COLOR_NONE ", width: " COLOR_YELLOW ""
     << rect.width << "" COLOR_NONE ", height: " COLOR_YELLOW "" << rect.height << "" COLOR_NONE " }";

  return JS_NewString(ctx, os.str().c_str());
}

static JSValue
js_rect_method(JSContext* ctx, JSValueConst rect, int argc, JSValueConst* argv, int magic) {
  JSRectData<double>* s = static_cast<JSRectData<double>*>(JS_GetOpaque2(ctx, rect, js_rect_class_id));
  JSValue ret = JS_UNDEFINED;
  JSPointData<double> point = js_point_get(ctx, argv[0]);
  if(magic == 0)
    ret = JS_NewBool(ctx, s->contains(point));
  if(magic == 1)
    ret = JS_NewBool(ctx, s->empty());
  if(magic == 2)
    ret = JS_NewFloat64(ctx, s->area());

  if(magic == 3 || magic == 4) {
    JSPointData<double> pt = magic == 3 ? s->br() : s->tl();

    ret = js_point_new(ctx, pt.x, pt.y);
  }
  if(magic == 5) {
    cv::Size2d sz = s->size();
    ret = js_size_new(ctx, sz.width, sz.height);
  }
  return ret;
}

static JSValue iterator_symbol = JS_UNDEFINED;

static JSValue
js_rect_symbol_iterator(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  JSValue arr, iter;
  jsrt js(ctx);
  arr = js_rect_funcs(ctx, this_val, argc, argv, 3);

  if(JS_IsUndefined(iterator_symbol))
    iterator_symbol = js.get_symbol("iterator");

  if(!JS_IsFunction(ctx, (iter = js.get_property(arr, iterator_symbol))))
    return JS_EXCEPTION;
  return JS_Call(ctx, iter, arr, 0, argv);
}

static JSValue
js_rect_from(JSContext* ctx, JSValueConst rect, int argc, JSValueConst* argv) {
  std::array<double, 4> array;
  JSValue ret = JS_EXCEPTION;

  if(JS_IsString(argv[0])) {
    const char* str = JS_ToCString(ctx, argv[0]);
    char* endptr = nullptr;
    for(size_t i = 0; i < 4; i++) {
      while(!isdigit(*str) && *str != '-' && *str != '+' && !(*str == '.' && isdigit(str[1]))) str++;
      if(*str == '\0')
        break;
      array[i] = strtod(str, &endptr);
      str = endptr;
    }
  } else if(JS_IsArray(ctx, argv[0])) {
    js_array_to(ctx, argv[0], array);
  }
  if(array[2] > 0 && array[3] > 0)
    ret = js_rect_new(ctx, array[0], array[1], array[2], array[3]);
  return ret;
}

JSClassDef js_rect_class = {
    .class_name = "Rect",
    .finalizer = js_rect_finalizer,
};

const JSCFunctionListEntry js_rect_proto_funcs[] = {JS_CGETSET_ENUMERABLE_DEF("x", js_rect_get, js_rect_set, 0),
                                                    JS_CGETSET_ENUMERABLE_DEF("y", js_rect_get, js_rect_set, 1),
                                                    JS_CGETSET_ENUMERABLE_DEF("width", js_rect_get, js_rect_set, 2),
                                                    JS_CGETSET_ENUMERABLE_DEF("height", js_rect_get, js_rect_set, 3),
                                                    JS_CGETSET_MAGIC_DEF("x2", js_rect_get, js_rect_set, 4),
                                                    JS_CGETSET_MAGIC_DEF("y2", js_rect_get, js_rect_set, 5),
                                                    JS_CGETSET_MAGIC_DEF("point", js_rect_get, js_rect_set, 6),
                                                    JS_CGETSET_MAGIC_DEF("size", js_rect_get, js_rect_set, 7),
                                                    JS_ALIAS_DEF("x1", "x"),
                                                    JS_ALIAS_DEF("y1", "y"),
                                                    JS_CFUNC_MAGIC_DEF("contains", 0, js_rect_method, 0),
                                                    JS_CFUNC_MAGIC_DEF("empty", 0, js_rect_method, 1),
                                                    JS_CFUNC_MAGIC_DEF("area", 0, js_rect_method, 2),
                                                    JS_CFUNC_MAGIC_DEF("br", 0, js_rect_method, 3),
                                                    JS_CFUNC_MAGIC_DEF("tl", 0, js_rect_method, 4),
                                                    // JS_CFUNC_MAGIC_DEF("size", 0, js_rect_method, 5),
                                                    JS_CFUNC_DEF("toString", 0, js_rect_to_string),
                                                    JS_CFUNC_DEF("toSource", 0, js_rect_to_source),
                                                    JS_CFUNC_DEF("inspect", 0, js_rect_inspect),
                                                    JS_CFUNC_MAGIC_DEF("equals", 1, js_rect_funcs, 0),
                                                    JS_CFUNC_MAGIC_DEF("round", 0, js_rect_funcs, 1),
                                                    JS_CFUNC_MAGIC_DEF("toObject", 0, js_rect_funcs, 2),
                                                    JS_CFUNC_MAGIC_DEF("toArray", 0, js_rect_funcs, 3),
                                                    JS_PROP_STRING_DEF("[Symbol.toStringTag]", "Rect", JS_PROP_CONFIGURABLE)

};
const JSCFunctionListEntry js_rect_static_funcs[] = {JS_CFUNC_DEF("from", 1, js_rect_from)};

int
js_rect_init(JSContext* ctx, JSModuleDef* m) {

  if(js_rect_class_id == 0) {
    /* create the Rect class */
    JS_NewClassID(&js_rect_class_id);
    JS_NewClass(JS_GetRuntime(ctx), js_rect_class_id, &js_rect_class);

    rect_proto = JS_NewObject(ctx);
    JS_SetPropertyFunctionList(ctx, rect_proto, js_rect_proto_funcs, countof(js_rect_proto_funcs));
    JS_SetClassProto(ctx, js_rect_class_id, rect_proto);

    rect_class = JS_NewCFunction2(ctx, js_rect_ctor, "Rect", 0, JS_CFUNC_constructor, 0);
    /* set proto.constructor and ctor.prototype */
    JS_SetConstructor(ctx, rect_class, rect_proto);
    JS_SetPropertyFunctionList(ctx, rect_class, js_rect_static_funcs, countof(js_rect_static_funcs));
  }

  if(m)
    JS_SetModuleExport(ctx, m, "Rect", rect_class);

  return 0;
}

void
js_rect_constructor(JSContext* ctx, JSValue parent, const char* name) {
  if(JS_IsUndefined(rect_class))
    js_rect_init(ctx, 0);

  JS_SetPropertyStr(ctx, parent, name ? name : "Rect", rect_class);
}

#ifdef JS_RECT_MODULE
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module
#else
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module_rect
#endif

extern "C" JSModuleDef*
JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_rect_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "Rect");
  return m;
}
