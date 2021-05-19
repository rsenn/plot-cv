#define LOG_TAG "org.opencv.highgui.cv::VideoCapture"
#include <common.h>

#include <opencv2/opencv_modules.hpp>
#ifdef HAVE_OPENCV_HIGHGUI

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
// using namespace cv;

/// throw java exception
static void
throwJavaException(JNIEnv* env, const std::exception* e, const char* method) {
  std::string what = "unknown exception";
  jclass je = 0;

  if(e) {
    std::string exception_type = "std::exception";

    if(dynamic_cast<const cv::Exception*>(e)) {
      exception_type = "cv::Exception";
      je = env->FindClass("org/opencv/core/CvException");
    }

    what = exception_type + ": " + e->what();
  }

  if(!je)
    je = env->FindClass("java/lang/Exception");
  env->ThrowNew(je, what.c_str());

  LOGE("%s caught %s", method, what.c_str());
  (void)method; // avoid "unused" warning
}

extern "C" {

//
//   cv::VideoCapture::VideoCapture()
//

JNIEXPORT jlong JNICALL Java_org_opencv_highgui_VideoCapture_n_1VideoCapture__(JNIEnv* env, jclass);

JNIEXPORT jlong JNICALL
Java_org_opencv_highgui_VideoCapture_n_1VideoCapture__(JNIEnv* env, jclass) {
  static const char method_name[] = "highgui::cv::VideoCapture::VideoCapture()";
  try {
    LOGD("%s", method_name);
    cv::VideoCapture* _retval_ = new cv::VideoCapture();
    return (jlong)_retval_;
  } catch(const std::exception& e) { throwJavaException(env, &e, method_name); } catch(...) {
    throwJavaException(env, 0, method_name);
  }
  return 0;
}

//
//   cv::VideoCapture::VideoCapture(int device)
//

JNIEXPORT jlong JNICALL Java_org_opencv_highgui_VideoCapture_n_1VideoCapture__I(JNIEnv* env, jclass, jint device);

JNIEXPORT jlong JNICALL
Java_org_opencv_highgui_VideoCapture_n_1VideoCapture__I(JNIEnv* env, jclass, jint device) {
  static const char method_name[] = "highgui::cv::VideoCapture::VideoCapture(int device)";
  try {
    LOGD("%s", method_name);
    cv::VideoCapture* _retval_ = new cv::VideoCapture(device);
    return (jlong)_retval_;
  } catch(const std::exception& e) { throwJavaException(env, &e, method_name); } catch(...) {
    throwJavaException(env, 0, method_name);
  }
  return 0;
}

//
//  double cv::VideoCapture::get(int propId)
//

JNIEXPORT jdouble JNICALL Java_org_opencv_highgui_VideoCapture_n_1get(JNIEnv* env, jclass, jlong self, jint propId);

JNIEXPORT jdouble JNICALL
Java_org_opencv_highgui_VideoCapture_n_1get(JNIEnv* env, jclass, jlong self, jint propId) {
  static const char method_name[] = "highgui::cv::VideoCapture::get(int propId)";
  try {
    LOGD("%s", method_name);
    cv::VideoCapture* me = (VideoCapture*)self; // TODO: check for NULL
    double _retval_ = me->get(propId);
    return _retval_;
  } catch(const std::exception& e) { throwJavaException(env, &e, method_name); } catch(...) {
    throwJavaException(env, 0, method_name);
  }
  return 0;
}

//
//  bool cv::VideoCapture::grab()
//

JNIEXPORT jboolean JNICALL Java_org_opencv_highgui_VideoCapture_n_1grab(JNIEnv* env, jclass, jlong self);

JNIEXPORT jboolean JNICALL
Java_org_opencv_highgui_VideoCapture_n_1grab(JNIEnv* env, jclass, jlong self) {
  static const char method_name[] = "highgui::cv::VideoCapture::grab()";
  try {
    LOGD("%s", method_name);
    cv::VideoCapture* me = (VideoCapture*)self; // TODO: check for NULL
    bool _retval_ = me->grab();
    return _retval_;
  } catch(const std::exception& e) { throwJavaException(env, &e, method_name); } catch(...) {
    throwJavaException(env, 0, method_name);
  }
  return false;
}

//
//  bool cv::VideoCapture::isOpened()
//

JNIEXPORT jboolean JNICALL Java_org_opencv_highgui_VideoCapture_n_1isOpened(JNIEnv* env, jclass, jlong self);

JNIEXPORT jboolean JNICALL
Java_org_opencv_highgui_VideoCapture_n_1isOpened(JNIEnv* env, jclass, jlong self) {
  static const char method_name[] = "highgui::cv::VideoCapture::isOpened()";
  try {
    LOGD("%s", method_name);
    cv::VideoCapture* me = (VideoCapture*)self; // TODO: check for NULL
    bool _retval_ = me->isOpened();
    return _retval_;
  } catch(const std::exception& e) { throwJavaException(env, &e, method_name); } catch(...) {
    throwJavaException(env, 0, method_name);
  }
  return false;
}

//
//  bool cv::VideoCapture::open(int device)
//

JNIEXPORT jboolean JNICALL Java_org_opencv_highgui_VideoCapture_n_1open__JI(JNIEnv* env, jclass, jlong self, jint device);

JNIEXPORT jboolean JNICALL
Java_org_opencv_highgui_VideoCapture_n_1open__JI(JNIEnv* env, jclass, jlong self, jint device) {
  static const char method_name[] = "highgui::cv::VideoCapture::open(int device)";
  try {
    LOGD("%s", method_name);
    cv::VideoCapture* me = (VideoCapture*)self; // TODO: check for NULL
    bool _retval_ = me->open(device);
    return _retval_;
  } catch(const std::exception& e) { throwJavaException(env, &e, method_name); } catch(...) {
    throwJavaException(env, 0, method_name);
  }
  return false;
}

//
//  bool cv::VideoCapture::cv::read(cv::Mat image)
//

JNIEXPORT jboolean JNICALL Java_org_opencv_highgui_VideoCapture_n_1read(JNIEnv* env, jclass, jlong self, jlong image_nativeObj);

JNIEXPORT jboolean JNICALL
Java_org_opencv_highgui_VideoCapture_n_1read(JNIEnv* env, jclass, jlong self, jlong image_nativeObj) {
  static const char method_name[] = "highgui::cv::VideoCapture::cv::read(cv::Mat image)";
  try {
    LOGD("%s", method_name);
    cv::VideoCapture* me = (VideoCapture*)self; // TODO: check for NULL
    cv::Mat& image = *((Mat*)image_nativeObj);
    bool _retval_ = me->cv::read(image);
    return _retval_;
  } catch(const std::exception& e) { throwJavaException(env, &e, method_name); } catch(...) {
    throwJavaException(env, 0, method_name);
  }
  return false;
}

//
//  void cv::VideoCapture::release()
//

JNIEXPORT void JNICALL Java_org_opencv_highgui_VideoCapture_n_1release(JNIEnv* env, jclass, jlong self);

JNIEXPORT void JNICALL
Java_org_opencv_highgui_VideoCapture_n_1release(JNIEnv* env, jclass, jlong self) {
  static const char method_name[] = "highgui::cv::VideoCapture::release()";
  try {
    LOGD("%s", method_name);
    cv::VideoCapture* me = (VideoCapture*)self; // TODO: check for NULL
    me->release();
    return;
  } catch(const std::exception& e) { throwJavaException(env, &e, method_name); } catch(...) {
    throwJavaException(env, 0, method_name);
  }
  return;
}

//
//  bool cv::VideoCapture::retrieve(cv::Mat image, int channel = 0)
//

JNIEXPORT jboolean JNICALL
Java_org_opencv_highgui_VideoCapture_n_1retrieve__JJI(JNIEnv* env, jclass, jlong self, jlong image_nativeObj, jint channel);

JNIEXPORT jboolean JNICALL
Java_org_opencv_highgui_VideoCapture_n_1retrieve__JJI(JNIEnv* env, jclass, jlong self, jlong image_nativeObj, jint channel) {
  static const char method_name[] = "highgui::cv::VideoCapture::retrieve(cv::Mat image, int channel)";
  try {
    LOGD("%s", method_name);
    cv::VideoCapture* me = (VideoCapture*)self; // TODO: check for NULL
    cv::Mat& image = *((Mat*)image_nativeObj);
    bool _retval_ = me->retrieve(image, channel);
    return _retval_;
  } catch(const std::exception& e) { throwJavaException(env, &e, method_name); } catch(...) {
    throwJavaException(env, 0, method_name);
  }
  return false;
}

JNIEXPORT jboolean JNICALL Java_org_opencv_highgui_VideoCapture_n_1retrieve__JJ(JNIEnv* env,
                                                                                jclass,
                                                                                jlong self,
                                                                                jlong image_nativeObj);

JNIEXPORT jboolean JNICALL
Java_org_opencv_highgui_VideoCapture_n_1retrieve__JJ(JNIEnv* env, jclass, jlong self, jlong image_nativeObj) {
  static const char method_name[] = "highgui::cv::VideoCapture::retrieve(cv::Mat image)";
  try {
    LOGD("%s", method_name);
    cv::VideoCapture* me = (VideoCapture*)self; // TODO: check for NULL
    cv::Mat& image = *((Mat*)image_nativeObj);
    bool _retval_ = me->retrieve(image);
    return _retval_;
  } catch(const std::exception& e) { throwJavaException(env, &e, method_name); } catch(...) {
    throwJavaException(env, 0, method_name);
  }
  return false;
}

//
//  bool cv::VideoCapture::set(int propId, double value)
//

JNIEXPORT jboolean JNICALL
Java_org_opencv_highgui_VideoCapture_n_1set(JNIEnv* env, jclass, jlong self, jint propId, jdouble value);

JNIEXPORT jboolean JNICALL
Java_org_opencv_highgui_VideoCapture_n_1set(JNIEnv* env, jclass, jlong self, jint propId, jdouble value) {
  static const char method_name[] = "highgui::cv::VideoCapture::set(int propId, double value)";
  try {
    LOGD("%s", method_name);
    cv::VideoCapture* me = (VideoCapture*)self; // TODO: check for NULL
    bool _retval_ = me->set(propId, value);
    return _retval_;
  } catch(const std::exception& e) { throwJavaException(env, &e, method_name); } catch(...) {
    throwJavaException(env, 0, method_name);
  }
  return false;
}

//
//  string cv::VideoCapture::getSupportedPreviewSizes(...)
//

JNIEXPORT jstring JNICALL Java_org_opencv_highgui_VideoCapture_n_1getSupportedPreviewSizes(JNIEnv* env, jclass, jlong self);

JNIEXPORT jstring JNICALL
Java_org_opencv_highgui_VideoCapture_n_1getSupportedPreviewSizes(JNIEnv* env, jclass, jlong self) {
  static const char method_name[] = "highgui::cv::VideoCapture::getSupportedPreviewSizes(...)";
  try {
    LOGD("%s", method_name);
    cv::VideoCapture* me = (VideoCapture*)self; // TODO: check for NULL
    union {
      double prop;
      const char* name;
    } u;
    u.prop = me->get(cv::CAP_PROP_SUPPORTED_PREVIEW_SIZES_STRING);
    // cv::VideoCapture::get can return 0.0 or -1.0 if cv::it doesn't support
    // cv::CAP_PROP_SUPPORTED_PREVIEW_SIZES_STRING
    if(u.prop != 0.0 && u.prop != -1.0)
      return env->NewStringUTF(u.name);
  } catch(const std::exception& e) { throwJavaException(env, &e, method_name); } catch(...) {
    throwJavaException(env, 0, method_name);
  }
  return env->NewStringUTF("");
}

//
//  native support for java finalize()
//  static void cv::VideoCapture::n_delete( __int64 self )
//

JNIEXPORT void JNICALL Java_org_opencv_highgui_VideoCapture_n_1delete(JNIEnv*, jclass, jlong self);

JNIEXPORT void JNICALL
Java_org_opencv_highgui_VideoCapture_n_1delete(JNIEnv*, jclass, jlong self) {
  delete(cv::VideoCapture*)self;
}

} // extern "C"

#endif // HAVE_OPENCV_HIGHGUI