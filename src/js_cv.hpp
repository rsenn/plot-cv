#ifndef JS_CV_HPP
#define JS_CV_HPP

#include <opencv2/core.hpp>
#include "jsbindings.hpp"

extern "C" JSClassID js_mat_class_id, js_umat_class_id, js_contour_class_id;

static inline cv::_InputArray
js_cv_inputarray(JSContext* ctx, JSValueConst value) {
  cv::Mat* mat;
  cv::UMat* umat;
  JSContourData<double>* contour;

  if((mat = static_cast<cv::Mat*>(JS_GetOpaque(value, js_mat_class_id))))
    return cv::_InputArray(*mat);
  if((umat = static_cast<cv::UMat*>(JS_GetOpaque(value, js_umat_class_id))))
    return cv::_InputArray(*umat);
  if((contour = static_cast<JSContourData<double>*>(JS_GetOpaque(value, js_contour_class_id))))
    return cv::_InputArray(*contour);

  if(js_is_arraybuffer(ctx, value)) {
    uint8_t* ptr;
    size_t size;
    ptr = JS_GetArrayBuffer(ctx, &size, value);

    return cv::_InputArray(ptr, size);
  }

  if(js_is_typedarray(ctx, value)) {
    TypedArrayProps props = js_typedarray_props(ctx, value);

    return cv::_InputArray(props.ptr<float>(), props.size<float>());
  }

  return cv::_InputArray();
}

#endif /* defined(JS_CV_HPP) */