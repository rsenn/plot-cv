#ifndef JS_UMAT_HPP
#define JS_UMAT_HPP

#include "jsbindings.hpp"

VISIBLE JSValue js_umat_new(JSContext*, int, int, int);
int js_umat_init(JSContext*, JSModuleDef*);
JSModuleDef* js_init_umat_module(JSContext* ctx, const char* module_name);
void js_umat_constructor(JSContext* ctx, JSValue parent, const char* name);

VISIBLE JSUMatData* js_umat_data(JSContext* ctx, JSValueConst val);

#endif /* defined(JS_UMAT_HPP) */