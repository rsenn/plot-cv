#ifndef JS_MAT_HPP
#define JS_MAT_HPP

#include "jsbindings.hpp"

extern "C" {

VISIBLE JSValue js_mat_new(JSContext*, uint32_t, uint32_t, int);
int js_mat_init(JSContext*, JSModuleDef*);
JSModuleDef* js_init_mat_module(JSContext* ctx, const char* module_name);
void js_mat_constructor(JSContext* ctx, JSValue parent, const char* name);

VISIBLE JSMatData* js_mat_data(JSContext* ctx, JSValueConst val);
}

#endif /* defined(JS_MAT_HPP) */