#ifndef JS_CV_HPP
#define JS_CV_HPP

#include <opencv2/core.hpp>
#include "jsbindings.hpp"

extern "C" JSClassID js_mat_class_id, js_umat_class_id, js_contour_class_id;

static inline JSInputOutputArray
js_cv_inputoutputarray(JSContext* ctx, JSValueConst value) {
  cv::Mat* mat;
  cv::UMat* umat;
  JSContourData<double>* contour;
  JSContoursData<double>* contours;

  if((mat = static_cast<cv::Mat*>(JS_GetOpaque(value, js_mat_class_id))))
    return JSInputOutputArray(*mat);
  if((umat = static_cast<cv::UMat*>(JS_GetOpaque(value, js_umat_class_id))))
    return JSInputOutputArray(*umat);
  if((contour = static_cast<JSContourData<double>*>(JS_GetOpaque(value, js_contour_class_id))))
    return JSInputOutputArray(*contour);

  if(js_is_arraybuffer(ctx, value)) {
    uint8_t* ptr;
    size_t size;
    ptr = JS_GetArrayBuffer(ctx, &size, value);

    return JSInputOutputArray(ptr, size);
  }

  if(js_is_typedarray(ctx, value))
    return js_typedarray_inputoutputarray(ctx, value);

  return cv::noArray();
}

template<class T>
static inline bool
js_is_noarray(const T& array) {
  return &array == &cv::noArray();
}

#endif /* defined(JS_CV_HPP) */