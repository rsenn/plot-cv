#ifndef JS_POINT_ITERATOR_HPP
#define JS_POINT_ITERATOR_HPP

#include "jsbindings.hpp"

extern "C" {

extern JSValue point_iterator_proto, point_iterator_class;
extern JSClassID js_point_iterator_class_id;

VISIBLE JSValue js_point_iterator_new(JSContext* ctx,
                                      const std::pair<JSPointData<double>*, JSPointData<double>*>& range,
                                      int magic);

int js_point_iterator_init(JSContext*, JSModuleDef* m);
void js_point_iterator_ctor(JSContext* ctx, JSValue parent, const char* name);

JSModuleDef* js_init_point_iterator_module(JSContext*, const char* module_name);
}

#endif /* defined(JS_POINT_ITERATOR_HPP) */