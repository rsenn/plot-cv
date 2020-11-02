#include "./jsbindings.h"
#include "./geometry.h"
#include "../quickjs/cutils.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#if defined(JS_CV_MODULE) || defined(quickjs_cv_EXPORTS)
#define JS_INIT_MODULE /*VISIBLE*/ js_init_module
#else
#define JS_INIT_MODULE VISIBLE js_init_module_cv
#endif

static JSValue
js_cv_ctor(JSContext* ctx, JSValueConst new_target, int argc, JSValueConst* argv) {
  JSValue obj = JS_UNDEFINED;
  JSValue proto;
  JSSizeData size;

  return obj;
}

static JSValue
js_cv_hough_lines(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  cv::Mat* image;
  JSValueConst array;
  double rho, theta;
  int32_t threshold;
  double srn = 0, stn = 0, min_theta = 0, max_theta = CV_PI;
  std::vector<cv::Vec2f> lines;
  size_t i;

  double angle = 0, scale = 1;
  cv::Mat m;

  JSValue ret;
  if(argc < 5)
    return JS_EXCEPTION;

  image = js_mat_data(ctx, argv[0]);

  if(image == nullptr || !JS_IsArray(ctx, argv[1]))
    return JS_EXCEPTION;

  array = argv[1];
  JS_ToFloat64(ctx, &rho, argv[2]);
  JS_ToFloat64(ctx, &theta, argv[3]);
  JS_ToInt32(ctx, &threshold, argv[4]);

  if(argc >= 6)
    JS_ToFloat64(ctx, &srn, argv[5]);
  if(argc >= 7)
    JS_ToFloat64(ctx, &stn, argv[6]);

  if(argc >= 8)
    JS_ToFloat64(ctx, &min_theta, argv[7]);
  if(argc >= 9)
    JS_ToFloat64(ctx, &max_theta, argv[8]);

  cv::HoughLines(*image, lines, rho, theta, threshold, srn, stn, min_theta, max_theta);

  i = 0;

  for(const cv::Vec2f& line : lines) {
    JSValue v = JS_NewArray(ctx);

    JS_SetPropertyUint32(ctx, v, 0, JS_NewFloat64(ctx, line[0]));
    JS_SetPropertyUint32(ctx, v, 1, JS_NewFloat64(ctx, line[1]));

    JS_SetPropertyUint32(ctx, argv[1], i++, v);
  }

  return JS_UNDEFINED;
}

static JSValue
js_cv_hough_lines_p(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  cv::Mat* image;
  JSValueConst array;
  double rho, theta;
  int32_t threshold;
  double minLineLength = 0, maxLineGap = 0;

  std::vector<cv::Vec4i> lines;
  size_t i;

  double angle = 0, scale = 1;
  cv::Mat m;

  JSValue ret;
  if(argc < 5)
    return JS_EXCEPTION;

  image = js_mat_data(ctx, argv[0]);

  if(image == nullptr || !JS_IsArray(ctx, argv[1]))
    return JS_EXCEPTION;

  array = argv[1];
  JS_ToFloat64(ctx, &rho, argv[2]);
  JS_ToFloat64(ctx, &theta, argv[3]);
  JS_ToInt32(ctx, &threshold, argv[4]);

  if(argc >= 6)
    JS_ToFloat64(ctx, &minLineLength, argv[5]);
  if(argc >= 7)
    JS_ToFloat64(ctx, &maxLineGap, argv[6]);

  cv::HoughLinesP(*image, lines, rho, theta, threshold, minLineLength, maxLineGap);

  i = 0;

  for(const cv::Vec4i& line : lines) {
    JSValue v = js_line_new(ctx, line[0], line[1], line[2], line[3]);

    JS_SetPropertyUint32(ctx, argv[1], i++, v);
  }

  return JS_UNDEFINED;
}

static JSValue
js_cv_canny(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  cv::Mat *image, *edges;
  double threshold1, threshold2;
  int32_t apertureSize = 3;
  bool L2gradient = false;

  image = js_mat_data(ctx, argv[0]);
  edges = js_mat_data(ctx, argv[1]);

  if(image == nullptr || edges == nullptr)
    return JS_EXCEPTION;

  JS_ToFloat64(ctx, &threshold1, argv[2]);
  JS_ToFloat64(ctx, &threshold2, argv[3]);

  if(argc >= 5)
    JS_ToInt32(ctx, &apertureSize, argv[4]);
  if(argc >= 6 && JS_IsBool(argv[4]))
    L2gradient = JS_ToBool(ctx, argv[5]);

  cv::Canny(*image, *edges, threshold1, threshold2, apertureSize, L2gradient);

  return JS_UNDEFINED;
}

static JSValue
js_cv_imread(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {

  const char* filename = JS_ToCString(ctx, argv[0]);

  cv::Mat mat = cv::imread(filename);

  return js_mat_wrap(ctx, mat);
}

static JSValue
js_cv_imwrite(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {

  const char* filename = JS_ToCString(ctx, argv[0]);
  cv::Mat* image = js_mat_data(ctx, argv[1]);

  if(image == nullptr)
    return JS_EXCEPTION;

  cv::imwrite(filename, *image);

  return JS_UNDEFINED;
}

static JSValue
js_cv_imshow(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {

  const char* filename = JS_ToCString(ctx, argv[0]);
  cv::Mat* image = js_mat_data(ctx, argv[1]);

  if(image == nullptr)
    return JS_EXCEPTION;

  cv::imshow(filename, *image);

  return JS_UNDEFINED;
}

static JSValue
js_cv_cvt_color(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {

  cv::Mat *src, *dst;
  int code, dstCn = 0;

  src = js_mat_data(ctx, argv[0]);
  dst = js_mat_data(ctx, argv[1]);

  if(src == nullptr || dst == nullptr)
    return JS_EXCEPTION;

  JS_ToInt32(ctx, &code, argv[2]);

  if(argc >= 4)
    JS_ToInt32(ctx, &dstCn, argv[3]);

  cv::cvtColor(*src, *dst, code, dstCn);
  return JS_UNDEFINED;
}

static JSValue
js_cv_split(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {

  cv::Mat* src;
  std::vector<cv::Mat> dst;
  int code, dstCn = 0;
  int32_t length;

  src = js_mat_data(ctx, argv[0]);

  if(src == nullptr)
    return JS_EXCEPTION;

  length = js_array_length(ctx, argv[1]);

  for(int32_t i = 0; i < src->channels(); i++) {
    dst.push_back(cv::Mat(src->size(), src->type() & 0x7));
  }

  // dst.resize(src->channels());

  if(dst.size() >= src->channels()) {

    /*  std::transform(js_begin(ctx, argv[1]), js_end(ctx, argv[1]), std::back_inserter(dst), [ctx,
      src](const JSValue& v) -> cv::Mat { cv::Mat* mat = js_mat_data(ctx, v); return mat == nullptr
      ? cv::Mat::zeros(src->rows, src->cols, src->type()) : *mat;
      });
  */

    cv::split(*src, dst.data());

    for(int32_t i = 0; i < src->channels(); i++) {
      JS_SetPropertyUint32(ctx, argv[1], i, js_mat_wrap(ctx, dst[i]));
    }

    return JS_UNDEFINED;
  }
  return JS_EXCEPTION;
}

static JSValue
js_cv_normalize(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {

  cv::Mat *src, *dst;
  double alpha = 1, beta = 0;
  int32_t norm_type = cv::NORM_L2, dtype = -1;

  src = js_mat_data(ctx, argv[0]);
  dst = js_mat_data(ctx, argv[1]);

  if(src == nullptr || dst == nullptr)
    return JS_EXCEPTION;

  if(argc >= 3)
    JS_ToFloat64(ctx, &alpha, argv[2]);
  if(argc >= 4)
    JS_ToFloat64(ctx, &beta, argv[3]);
  if(argc >= 5)
    JS_ToInt32(ctx, &norm_type, argv[4]);
  if(argc >= 6)
    JS_ToInt32(ctx, &dtype, argv[5]);

  cv::normalize(*src, *dst, alpha, beta, norm_type, dtype);
  return JS_UNDEFINED;
}

static JSValue
js_cv_named_window(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  const char* name;
  int32_t flags = cv::WINDOW_AUTOSIZE;
  name = JS_ToCString(ctx, argv[0]);

  if(argc > 1)
    JS_ToInt32(ctx, &flags, argv[1]);

  cv::namedWindow(name, flags);
  return JS_UNDEFINED;
}

static JSValue
js_cv_create_trackbar(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  const char *name, *window;
  int32_t ret, count;
  struct Trackbar {
    int32_t value;
    JSValue name, window, count;
    JSValueConst handler;
    JSContext* ctx;
  };
  Trackbar* userdata;

  name = JS_ToCString(ctx, argv[0]);
  window = JS_ToCString(ctx, argv[1]);

  if(name == nullptr || window == nullptr)
    return JS_EXCEPTION;

  userdata = static_cast<Trackbar*>(js_mallocz(ctx, sizeof(Trackbar)));

  JS_ToInt32(ctx, &userdata->value, argv[2]);
  JS_ToInt32(ctx, &count, argv[3]);

  userdata->name = JS_NewString(ctx, name);
  userdata->window = JS_NewString(ctx, window);
  userdata->count = JS_NewInt32(ctx, count);
  userdata->handler = JS_DupValue(ctx, argv[4]);
  userdata->ctx = ctx;

  /*JSValue str = JS_ToString(ctx, userdata->handler);
  std::cout << "handler: " << JS_ToCString(ctx, str) << std::endl;*/

  ret = cv::createTrackbar(
      name,
      window,
      &userdata->value,
      count,
      [](int newValue, void* ptr) {
        Trackbar const& data = *static_cast<Trackbar*>(ptr);

        if(JS_IsFunction(data.ctx, data.handler)) {
          JSValueConst argv[] = {JS_NewInt32(data.ctx, newValue),
                                 data.count,
                                 data.name,
                                 data.window};

          JS_Call(data.ctx, data.handler, JS_UNDEFINED, 4, argv);
        }
      },
      userdata);

  return JS_NewInt32(ctx, ret);
}

static JSValue
js_cv_wait_key(JSContext* ctx, JSValueConst this_val, int argc, JSValueConst* argv) {
  int32_t delay = 0;
  union {
    int32_t i;
    char c;
  } key;
  JSValue ret;

  if(argc > 0)
    JS_ToInt32(ctx, &delay, argv[0]);

  key.i = cv::waitKey(delay);

  if(key.i >= 0 && key.i <= 255) {
    char ch[2] = {key.c, 0};

    ret = JS_NewString(ctx, ch);
  } else {
    ret = JS_NewInt32(ctx, key.i);
  }
  return ret;
}

JSValue cv_proto, cv_class;
JSClassID js_cv_class_id;

JSClassDef js_cv_class = {"cv"};

const JSCFunctionListEntry js_cv_static_funcs[] = {
    JS_CFUNC_DEF("HoughLines", 5, js_cv_hough_lines),
    JS_CFUNC_DEF("HoughLinesP", 5, js_cv_hough_lines_p),
    JS_CFUNC_DEF("Canny", 4, js_cv_canny),
    JS_CFUNC_DEF("imread", 1, js_cv_imread),
    JS_CFUNC_DEF("imwrite", 2, js_cv_imwrite),
    JS_CFUNC_DEF("imshow", 2, js_cv_imshow),
    JS_CFUNC_DEF("cvtColor", 3, js_cv_cvt_color),
    JS_CFUNC_DEF("split", 2, js_cv_split),
    JS_CFUNC_DEF("normalize", 2, js_cv_normalize),
    JS_CFUNC_DEF("namedWindow", 1, js_cv_named_window),
    JS_CFUNC_DEF("createTrackbar", 5, js_cv_create_trackbar),
    JS_CFUNC_DEF("waitKey", 0, js_cv_wait_key),
    JS_PROP_INT32_DEF("CV_VERSION_MAJOR", CV_VERSION_MAJOR, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_VERSION_MINOR", CV_VERSION_MINOR, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_VERSION_REVISION", CV_VERSION_REVISION, JS_PROP_ENUMERABLE),
    JS_PROP_STRING_DEF("CV_VERSION_STATUS", CV_VERSION_STATUS, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_CMP_EQ", CV_CMP_EQ, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_CMP_GT", CV_CMP_GT, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_CMP_GE", CV_CMP_GE, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_CMP_LT", CV_CMP_LT, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_CMP_LE", CV_CMP_LE, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_CMP_NE", CV_CMP_NE, JS_PROP_ENUMERABLE),
    JS_PROP_DOUBLE_DEF("CV_PI", CV_PI, JS_PROP_ENUMERABLE),
    JS_PROP_DOUBLE_DEF("CV_2PI", CV_2PI, JS_PROP_ENUMERABLE),
    JS_PROP_DOUBLE_DEF("CV_LOG2", CV_LOG2, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_8U", CV_8U, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_8S", CV_8S, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_16U", CV_16U, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_16S", CV_16S, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_32S", CV_32S, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_32F", CV_32F, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_64F", CV_64F, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_8UC1", CV_8UC1, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_8UC2", CV_8UC2, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_8UC3", CV_8UC3, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_8UC4", CV_8UC4, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_8SC1", CV_8SC1, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_8SC2", CV_8SC2, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_8SC3", CV_8SC3, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_8SC4", CV_8SC4, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_16UC1", CV_16UC1, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_16UC2", CV_16UC2, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_16UC3", CV_16UC3, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_16UC4", CV_16UC4, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_16SC1", CV_16SC1, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_16SC2", CV_16SC2, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_16SC3", CV_16SC3, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_16SC4", CV_16SC4, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_32SC1", CV_32SC1, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_32SC2", CV_32SC2, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_32SC3", CV_32SC3, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_32SC4", CV_32SC4, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_32FC1", CV_32FC1, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_32FC2", CV_32FC2, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_32FC3", CV_32FC3, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_32FC4", CV_32FC4, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_64FC1", CV_64FC1, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_64FC2", CV_64FC2, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_64FC3", CV_64FC3, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("CV_64FC4", CV_64FC4, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("NORM_HAMMING", cv::NORM_HAMMING, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("NORM_HAMMING2", cv::NORM_HAMMING2, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("NORM_INF", cv::NORM_INF, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("NORM_L1", cv::NORM_L1, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("NORM_L2", cv::NORM_L2, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("NORM_L2SQR", cv::NORM_L2SQR, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("NORM_MINMAX", cv::NORM_MINMAX, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("NORM_RELATIVE", cv::NORM_RELATIVE, JS_PROP_ENUMERABLE),
    JS_PROP_INT32_DEF("NORM_TYPE_MASK", cv::NORM_TYPE_MASK, JS_PROP_ENUMERABLE),

    JS_PROP_INT32_DEF("COLOR_BGR2BGRA", cv::COLOR_BGR2BGRA, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2RGBA", cv::COLOR_RGB2RGBA, 0),
    JS_PROP_INT32_DEF("COLOR_BGRA2BGR", cv::COLOR_BGRA2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_RGBA2RGB", cv::COLOR_RGBA2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2RGBA", cv::COLOR_BGR2RGBA, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2BGRA", cv::COLOR_RGB2BGRA, 0),
    JS_PROP_INT32_DEF("COLOR_RGBA2BGR", cv::COLOR_RGBA2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_BGRA2RGB", cv::COLOR_BGRA2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2RGB", cv::COLOR_BGR2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2BGR", cv::COLOR_RGB2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_BGRA2RGBA", cv::COLOR_BGRA2RGBA, 0),
    JS_PROP_INT32_DEF("COLOR_RGBA2BGRA", cv::COLOR_RGBA2BGRA, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2GRAY", cv::COLOR_BGR2GRAY, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2GRAY", cv::COLOR_RGB2GRAY, 0),
    JS_PROP_INT32_DEF("COLOR_GRAY2BGR", cv::COLOR_GRAY2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_GRAY2RGB", cv::COLOR_GRAY2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_GRAY2BGRA", cv::COLOR_GRAY2BGRA, 0),
    JS_PROP_INT32_DEF("COLOR_GRAY2RGBA", cv::COLOR_GRAY2RGBA, 0),
    JS_PROP_INT32_DEF("COLOR_BGRA2GRAY", cv::COLOR_BGRA2GRAY, 0),
    JS_PROP_INT32_DEF("COLOR_RGBA2GRAY", cv::COLOR_RGBA2GRAY, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2BGR565", cv::COLOR_BGR2BGR565, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2BGR565", cv::COLOR_RGB2BGR565, 0),
    JS_PROP_INT32_DEF("COLOR_BGR5652BGR", cv::COLOR_BGR5652BGR, 0),
    JS_PROP_INT32_DEF("COLOR_BGR5652RGB", cv::COLOR_BGR5652RGB, 0),
    JS_PROP_INT32_DEF("COLOR_BGRA2BGR565", cv::COLOR_BGRA2BGR565, 0),
    JS_PROP_INT32_DEF("COLOR_RGBA2BGR565", cv::COLOR_RGBA2BGR565, 0),
    JS_PROP_INT32_DEF("COLOR_BGR5652BGRA", cv::COLOR_BGR5652BGRA, 0),
    JS_PROP_INT32_DEF("COLOR_BGR5652RGBA", cv::COLOR_BGR5652RGBA, 0),
    JS_PROP_INT32_DEF("COLOR_GRAY2BGR565", cv::COLOR_GRAY2BGR565, 0),
    JS_PROP_INT32_DEF("COLOR_BGR5652GRAY", cv::COLOR_BGR5652GRAY, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2BGR555", cv::COLOR_BGR2BGR555, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2BGR555", cv::COLOR_RGB2BGR555, 0),
    JS_PROP_INT32_DEF("COLOR_BGR5552BGR", cv::COLOR_BGR5552BGR, 0),
    JS_PROP_INT32_DEF("COLOR_BGR5552RGB", cv::COLOR_BGR5552RGB, 0),
    JS_PROP_INT32_DEF("COLOR_BGRA2BGR555", cv::COLOR_BGRA2BGR555, 0),
    JS_PROP_INT32_DEF("COLOR_RGBA2BGR555", cv::COLOR_RGBA2BGR555, 0),
    JS_PROP_INT32_DEF("COLOR_BGR5552BGRA", cv::COLOR_BGR5552BGRA, 0),
    JS_PROP_INT32_DEF("COLOR_BGR5552RGBA", cv::COLOR_BGR5552RGBA, 0),
    JS_PROP_INT32_DEF("COLOR_GRAY2BGR555", cv::COLOR_GRAY2BGR555, 0),
    JS_PROP_INT32_DEF("COLOR_BGR5552GRAY", cv::COLOR_BGR5552GRAY, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2XYZ", cv::COLOR_BGR2XYZ, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2XYZ", cv::COLOR_RGB2XYZ, 0),
    JS_PROP_INT32_DEF("COLOR_XYZ2BGR", cv::COLOR_XYZ2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_XYZ2RGB", cv::COLOR_XYZ2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2YCrCb", cv::COLOR_BGR2YCrCb, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2YCrCb", cv::COLOR_RGB2YCrCb, 0),
    JS_PROP_INT32_DEF("COLOR_YCrCb2BGR", cv::COLOR_YCrCb2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_YCrCb2RGB", cv::COLOR_YCrCb2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2HSV", cv::COLOR_BGR2HSV, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2HSV", cv::COLOR_RGB2HSV, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2Lab", cv::COLOR_BGR2Lab, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2Lab", cv::COLOR_RGB2Lab, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2Luv", cv::COLOR_BGR2Luv, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2Luv", cv::COLOR_RGB2Luv, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2HLS", cv::COLOR_BGR2HLS, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2HLS", cv::COLOR_RGB2HLS, 0),
    JS_PROP_INT32_DEF("COLOR_HSV2BGR", cv::COLOR_HSV2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_HSV2RGB", cv::COLOR_HSV2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_Lab2BGR", cv::COLOR_Lab2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_Lab2RGB", cv::COLOR_Lab2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_Luv2BGR", cv::COLOR_Luv2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_Luv2RGB", cv::COLOR_Luv2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_HLS2BGR", cv::COLOR_HLS2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_HLS2RGB", cv::COLOR_HLS2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2HSV_FULL", cv::COLOR_BGR2HSV_FULL, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2HSV_FULL", cv::COLOR_RGB2HSV_FULL, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2HLS_FULL", cv::COLOR_BGR2HLS_FULL, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2HLS_FULL", cv::COLOR_RGB2HLS_FULL, 0),
    JS_PROP_INT32_DEF("COLOR_HSV2BGR_FULL", cv::COLOR_HSV2BGR_FULL, 0),
    JS_PROP_INT32_DEF("COLOR_HSV2RGB_FULL", cv::COLOR_HSV2RGB_FULL, 0),
    JS_PROP_INT32_DEF("COLOR_HLS2BGR_FULL", cv::COLOR_HLS2BGR_FULL, 0),
    JS_PROP_INT32_DEF("COLOR_HLS2RGB_FULL", cv::COLOR_HLS2RGB_FULL, 0),
    JS_PROP_INT32_DEF("COLOR_LBGR2Lab", cv::COLOR_LBGR2Lab, 0),
    JS_PROP_INT32_DEF("COLOR_LRGB2Lab", cv::COLOR_LRGB2Lab, 0),
    JS_PROP_INT32_DEF("COLOR_LBGR2Luv", cv::COLOR_LBGR2Luv, 0),
    JS_PROP_INT32_DEF("COLOR_LRGB2Luv", cv::COLOR_LRGB2Luv, 0),
    JS_PROP_INT32_DEF("COLOR_Lab2LBGR", cv::COLOR_Lab2LBGR, 0),
    JS_PROP_INT32_DEF("COLOR_Lab2LRGB", cv::COLOR_Lab2LRGB, 0),
    JS_PROP_INT32_DEF("COLOR_Luv2LBGR", cv::COLOR_Luv2LBGR, 0),
    JS_PROP_INT32_DEF("COLOR_Luv2LRGB", cv::COLOR_Luv2LRGB, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2YUV", cv::COLOR_BGR2YUV, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2YUV", cv::COLOR_RGB2YUV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGR", cv::COLOR_YUV2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGB", cv::COLOR_YUV2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGB_NV12", cv::COLOR_YUV2RGB_NV12, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGR_NV12", cv::COLOR_YUV2BGR_NV12, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGB_NV21", cv::COLOR_YUV2RGB_NV21, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGR_NV21", cv::COLOR_YUV2BGR_NV21, 0),
    JS_PROP_INT32_DEF("COLOR_YUV420sp2RGB", cv::COLOR_YUV420sp2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_YUV420sp2BGR", cv::COLOR_YUV420sp2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGBA_NV12", cv::COLOR_YUV2RGBA_NV12, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGRA_NV12", cv::COLOR_YUV2BGRA_NV12, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGBA_NV21", cv::COLOR_YUV2RGBA_NV21, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGRA_NV21", cv::COLOR_YUV2BGRA_NV21, 0),
    JS_PROP_INT32_DEF("COLOR_YUV420sp2RGBA", cv::COLOR_YUV420sp2RGBA, 0),
    JS_PROP_INT32_DEF("COLOR_YUV420sp2BGRA", cv::COLOR_YUV420sp2BGRA, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGB_YV12", cv::COLOR_YUV2RGB_YV12, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGR_YV12", cv::COLOR_YUV2BGR_YV12, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGB_IYUV", cv::COLOR_YUV2RGB_IYUV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGR_IYUV", cv::COLOR_YUV2BGR_IYUV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGB_I420", cv::COLOR_YUV2RGB_I420, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGR_I420", cv::COLOR_YUV2BGR_I420, 0),
    JS_PROP_INT32_DEF("COLOR_YUV420p2RGB", cv::COLOR_YUV420p2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_YUV420p2BGR", cv::COLOR_YUV420p2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGBA_YV12", cv::COLOR_YUV2RGBA_YV12, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGRA_YV12", cv::COLOR_YUV2BGRA_YV12, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGBA_IYUV", cv::COLOR_YUV2RGBA_IYUV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGRA_IYUV", cv::COLOR_YUV2BGRA_IYUV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGBA_I420", cv::COLOR_YUV2RGBA_I420, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGRA_I420", cv::COLOR_YUV2BGRA_I420, 0),
    JS_PROP_INT32_DEF("COLOR_YUV420p2RGBA", cv::COLOR_YUV420p2RGBA, 0),
    JS_PROP_INT32_DEF("COLOR_YUV420p2BGRA", cv::COLOR_YUV420p2BGRA, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2GRAY_420", cv::COLOR_YUV2GRAY_420, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2GRAY_NV21", cv::COLOR_YUV2GRAY_NV21, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2GRAY_NV12", cv::COLOR_YUV2GRAY_NV12, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2GRAY_YV12", cv::COLOR_YUV2GRAY_YV12, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2GRAY_IYUV", cv::COLOR_YUV2GRAY_IYUV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2GRAY_I420", cv::COLOR_YUV2GRAY_I420, 0),
    JS_PROP_INT32_DEF("COLOR_YUV420sp2GRAY", cv::COLOR_YUV420sp2GRAY, 0),
    JS_PROP_INT32_DEF("COLOR_YUV420p2GRAY", cv::COLOR_YUV420p2GRAY, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGB_UYVY", cv::COLOR_YUV2RGB_UYVY, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGR_UYVY", cv::COLOR_YUV2BGR_UYVY, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGB_Y422", cv::COLOR_YUV2RGB_Y422, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGR_Y422", cv::COLOR_YUV2BGR_Y422, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGB_UYNV", cv::COLOR_YUV2RGB_UYNV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGR_UYNV", cv::COLOR_YUV2BGR_UYNV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGBA_UYVY", cv::COLOR_YUV2RGBA_UYVY, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGRA_UYVY", cv::COLOR_YUV2BGRA_UYVY, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGBA_Y422", cv::COLOR_YUV2RGBA_Y422, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGRA_Y422", cv::COLOR_YUV2BGRA_Y422, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGBA_UYNV", cv::COLOR_YUV2RGBA_UYNV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGRA_UYNV", cv::COLOR_YUV2BGRA_UYNV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGB_YUY2", cv::COLOR_YUV2RGB_YUY2, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGR_YUY2", cv::COLOR_YUV2BGR_YUY2, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGB_YVYU", cv::COLOR_YUV2RGB_YVYU, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGR_YVYU", cv::COLOR_YUV2BGR_YVYU, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGB_YUYV", cv::COLOR_YUV2RGB_YUYV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGR_YUYV", cv::COLOR_YUV2BGR_YUYV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGB_YUNV", cv::COLOR_YUV2RGB_YUNV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGR_YUNV", cv::COLOR_YUV2BGR_YUNV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGBA_YUY2", cv::COLOR_YUV2RGBA_YUY2, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGRA_YUY2", cv::COLOR_YUV2BGRA_YUY2, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGBA_YVYU", cv::COLOR_YUV2RGBA_YVYU, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGRA_YVYU", cv::COLOR_YUV2BGRA_YVYU, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGBA_YUYV", cv::COLOR_YUV2RGBA_YUYV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGRA_YUYV", cv::COLOR_YUV2BGRA_YUYV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2RGBA_YUNV", cv::COLOR_YUV2RGBA_YUNV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2BGRA_YUNV", cv::COLOR_YUV2BGRA_YUNV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2GRAY_UYVY", cv::COLOR_YUV2GRAY_UYVY, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2GRAY_YUY2", cv::COLOR_YUV2GRAY_YUY2, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2GRAY_Y422", cv::COLOR_YUV2GRAY_Y422, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2GRAY_UYNV", cv::COLOR_YUV2GRAY_UYNV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2GRAY_YVYU", cv::COLOR_YUV2GRAY_YVYU, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2GRAY_YUYV", cv::COLOR_YUV2GRAY_YUYV, 0),
    JS_PROP_INT32_DEF("COLOR_YUV2GRAY_YUNV", cv::COLOR_YUV2GRAY_YUNV, 0),
    JS_PROP_INT32_DEF("COLOR_RGBA2mRGBA", cv::COLOR_RGBA2mRGBA, 0),
    JS_PROP_INT32_DEF("COLOR_mRGBA2RGBA", cv::COLOR_mRGBA2RGBA, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2YUV_I420", cv::COLOR_RGB2YUV_I420, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2YUV_I420", cv::COLOR_BGR2YUV_I420, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2YUV_IYUV", cv::COLOR_RGB2YUV_IYUV, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2YUV_IYUV", cv::COLOR_BGR2YUV_IYUV, 0),
    JS_PROP_INT32_DEF("COLOR_RGBA2YUV_I420", cv::COLOR_RGBA2YUV_I420, 0),
    JS_PROP_INT32_DEF("COLOR_BGRA2YUV_I420", cv::COLOR_BGRA2YUV_I420, 0),
    JS_PROP_INT32_DEF("COLOR_RGBA2YUV_IYUV", cv::COLOR_RGBA2YUV_IYUV, 0),
    JS_PROP_INT32_DEF("COLOR_BGRA2YUV_IYUV", cv::COLOR_BGRA2YUV_IYUV, 0),
    JS_PROP_INT32_DEF("COLOR_RGB2YUV_YV12", cv::COLOR_RGB2YUV_YV12, 0),
    JS_PROP_INT32_DEF("COLOR_BGR2YUV_YV12", cv::COLOR_BGR2YUV_YV12, 0),
    JS_PROP_INT32_DEF("COLOR_RGBA2YUV_YV12", cv::COLOR_RGBA2YUV_YV12, 0),
    JS_PROP_INT32_DEF("COLOR_BGRA2YUV_YV12", cv::COLOR_BGRA2YUV_YV12, 0),
    JS_PROP_INT32_DEF("COLOR_BayerBG2BGR", cv::COLOR_BayerBG2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGB2BGR", cv::COLOR_BayerGB2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_BayerRG2BGR", cv::COLOR_BayerRG2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGR2BGR", cv::COLOR_BayerGR2BGR, 0),
    JS_PROP_INT32_DEF("COLOR_BayerBG2RGB", cv::COLOR_BayerBG2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGB2RGB", cv::COLOR_BayerGB2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_BayerRG2RGB", cv::COLOR_BayerRG2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGR2RGB", cv::COLOR_BayerGR2RGB, 0),
    JS_PROP_INT32_DEF("COLOR_BayerBG2GRAY", cv::COLOR_BayerBG2GRAY, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGB2GRAY", cv::COLOR_BayerGB2GRAY, 0),
    JS_PROP_INT32_DEF("COLOR_BayerRG2GRAY", cv::COLOR_BayerRG2GRAY, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGR2GRAY", cv::COLOR_BayerGR2GRAY, 0),
    JS_PROP_INT32_DEF("COLOR_BayerBG2BGR_VNG", cv::COLOR_BayerBG2BGR_VNG, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGB2BGR_VNG", cv::COLOR_BayerGB2BGR_VNG, 0),
    JS_PROP_INT32_DEF("COLOR_BayerRG2BGR_VNG", cv::COLOR_BayerRG2BGR_VNG, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGR2BGR_VNG", cv::COLOR_BayerGR2BGR_VNG, 0),
    JS_PROP_INT32_DEF("COLOR_BayerBG2RGB_VNG", cv::COLOR_BayerBG2RGB_VNG, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGB2RGB_VNG", cv::COLOR_BayerGB2RGB_VNG, 0),
    JS_PROP_INT32_DEF("COLOR_BayerRG2RGB_VNG", cv::COLOR_BayerRG2RGB_VNG, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGR2RGB_VNG", cv::COLOR_BayerGR2RGB_VNG, 0),
    JS_PROP_INT32_DEF("COLOR_BayerBG2BGR_EA", cv::COLOR_BayerBG2BGR_EA, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGB2BGR_EA", cv::COLOR_BayerGB2BGR_EA, 0),
    JS_PROP_INT32_DEF("COLOR_BayerRG2BGR_EA", cv::COLOR_BayerRG2BGR_EA, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGR2BGR_EA", cv::COLOR_BayerGR2BGR_EA, 0),
    JS_PROP_INT32_DEF("COLOR_BayerBG2RGB_EA", cv::COLOR_BayerBG2RGB_EA, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGB2RGB_EA", cv::COLOR_BayerGB2RGB_EA, 0),
    JS_PROP_INT32_DEF("COLOR_BayerRG2RGB_EA", cv::COLOR_BayerRG2RGB_EA, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGR2RGB_EA", cv::COLOR_BayerGR2RGB_EA, 0),
    JS_PROP_INT32_DEF("COLOR_BayerBG2BGRA", cv::COLOR_BayerBG2BGRA, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGB2BGRA", cv::COLOR_BayerGB2BGRA, 0),
    JS_PROP_INT32_DEF("COLOR_BayerRG2BGRA", cv::COLOR_BayerRG2BGRA, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGR2BGRA", cv::COLOR_BayerGR2BGRA, 0),
    JS_PROP_INT32_DEF("COLOR_BayerBG2RGBA", cv::COLOR_BayerBG2RGBA, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGB2RGBA", cv::COLOR_BayerGB2RGBA, 0),
    JS_PROP_INT32_DEF("COLOR_BayerRG2RGBA", cv::COLOR_BayerRG2RGBA, 0),
    JS_PROP_INT32_DEF("COLOR_BayerGR2RGBA", cv::COLOR_BayerGR2RGBA, 0),

};

int
js_cv_init(JSContext* ctx, JSModuleDef* m) {
  cv_class = JS_NewObject(ctx);
  JS_SetPropertyFunctionList(ctx, cv_class, js_cv_static_funcs, countof(js_cv_static_funcs));
  JSValue g = JS_GetGlobalObject(ctx);

  int32array_ctor = JS_GetProperty(ctx, g, JS_ATOM_Int32Array);
  int32array_proto = JS_GetPrototype(ctx, int32array_ctor);

  if(m)
    JS_SetModuleExport(ctx, m, "cv", cv_class);

  return 0;
}

extern "C" JSModuleDef*
JS_INIT_MODULE(JSContext* ctx, const char* module_name) {
  JSModuleDef* m;
  m = JS_NewCModule(ctx, module_name, &js_cv_init);
  if(!m)
    return NULL;
  JS_AddModuleExport(ctx, m, "cv");
  return m;
}
