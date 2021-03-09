#ifndef JS_TYPED_ARRAY_HPP
#define JS_TYPED_ARRAY_HPP

#include <type_traits>
#include <cstdint>
#include "util.hpp"
#include "jsbindings.hpp"

template<class T> struct number_type { static constexpr bool typed_array = false; };

template<> struct number_type<int8_t> {
  typedef int8_t value_type;
  static constexpr bool typed_array = true;
  static constexpr const char*
  constructor_name() {
    return "Int8Array";
  }
};

template<> struct number_type<uint8_t> {
  typedef uint8_t value_type;
  static constexpr bool typed_array = true;
  static constexpr const char*
  constructor_name() {
    return "Uint8Array";
  }
};

template<> struct number_type<int16_t> {
  typedef int16_t value_type;
  static constexpr bool typed_array = true;
  static constexpr const char*
  constructor_name() {
    return "Int16Array";
  }
};

template<> struct number_type<uint16_t> {
  typedef uint16_t value_type;
  static constexpr bool typed_array = true;
  static constexpr const char*
  constructor_name() {
    return "Uint16Array";
  }
};

template<> struct number_type<int32_t> {
  typedef int32_t value_type;
  static constexpr bool typed_array = true;
  static constexpr const char*
  constructor_name() {
    return "Int32Array";
  }
};

template<> struct number_type<uint32_t> {
  typedef uint32_t value_type;
  static constexpr bool typed_array = true;
  static constexpr const char*
  constructor_name() {
    return "Uint32Array";
  }
};

template<> struct number_type<int64_t> {
  typedef int64_t value_type;
  static constexpr bool typed_array = true;
  static constexpr const char*
  constructor_name() {
    return "BigInt64Array";
  }
};

template<> struct number_type<uint64_t> {
  typedef uint64_t value_type;
  static constexpr bool typed_array = true;
  static constexpr const char*
  constructor_name() {
    return "BigUint64Array";
  }
};

template<> struct number_type<float> {
  typedef float value_type;
  static constexpr bool typed_array = true;
  static constexpr const char*
  constructor_name() {
    return "Float32Array";
  }
};

template<> struct number_type<double> {
  typedef double value_type;
  static constexpr bool typed_array = true;
  static constexpr const char*
  constructor_name() {
    return "Float64Array";
  }
};

template<class T> struct pointer_type {
  typedef typename std::remove_cv<typename std::remove_pointer<T>::type>::type value_type;
  static constexpr bool typed_array = number_type<value_type>::typed_array;
};

template<class T>
inline std::enable_if_t<number_type<T>::typed_array, JSValue>
js_array_from(JSContext* ctx, const T* start, const T* end) {
  JSValue buf, global, ctor;
  JSValueConst args[3];
  const char* name;
  const uint8_t *s, *e;
  s = reinterpret_cast<const uint8_t*>(start);
  e = reinterpret_cast<const uint8_t*>(end);
  buf = JS_NewArrayBufferCopy(ctx, s, e - s);
  name = number_type<T>::constructor_name();
  global = JS_GetGlobalObject(ctx);
  ctor = JS_GetPropertyStr(ctx, global, name);
  args[0] = buf;
  return JS_CallConstructor(ctx, ctor, 1, args);
}

struct TypedArrayProps {
  TypedArrayProps(int bsize, bool sig, bool flt) : byte_size(bsize), is_signed(sig), is_floating(flt) {}
  TypedArrayProps(const cv::Mat& mat)
      : byte_size(mat.elemSize1()), is_signed(mat_signed(mat)), is_floating(mat_floating(mat)) {}

  int byte_size;
  bool is_signed;
  bool is_floating;

  const std::string
  constructor_name() const {
    std::ostringstream os;
    if(!is_floating) {
      if(byte_size == 8)
        os << "Big";
      os << (is_signed ? "Int" : "Uint");
    } else {
      os << "Float";
    }
    os << (byte_size * 8);
    os << "Array";
    return os.str();
  }
};

static inline JSValue
js_typedarray_new(JSContext* ctx,
                  JSValueConst buffer,
                  uint32_t byteOffset,
                  uint32_t length,
                  const TypedArrayProps& props) {
  jsrt js(ctx);
  JSValue global, ctor, ret;
  std::string ctor_name = props.constructor_name();

  std::cerr << "js_typedarray_new " << ctor_name << std::endl;

  std::array<JSValueConst, 3> args = {buffer, js_number_new(ctx, byteOffset), js_number_new(ctx, length)};

  global = JS_GetGlobalObject(ctx);
  ctor = JS_GetPropertyStr(ctx, global, ctor_name.c_str());
  JS_FreeValue(ctx, global);

  auto range = js_arraybuffer_range(ctx, buffer);

  std::cerr << "js_typedarray_new function " << js.function_name(ctor) << std::endl;
  std::cerr << "js_typedarray_new buffer " << range.begin() << " - " << range.end() << std::endl;
  std::cerr << "js_typedarray_new size " << range.size() << std::endl;

  ret = JS_CallConstructor(ctx, ctor, args.size(), &args[0]);
  JS_FreeValue(ctx, ctor);
  return ret;
}

#endif /* defined(JS_TYPED_ARRAY_HPP) */