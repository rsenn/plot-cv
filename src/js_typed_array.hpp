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
  TypedArrayProps(int bsize, bool sig, bool flt) : byte_size(bsize), is_signed(sig), is_floating_point(flt) {}
  TypedArrayProps(const cv::Mat& mat) : byte_size(mat.elemSize1()), is_signed(mat_signed(mat)), is_floating_point(mat_floating(mat)) {}

  int byte_size;
  bool is_signed;
  bool is_floating_point;

  const std::string
  constructor_name() const {
    std::ostringstream os;
    if(!is_floating_point) {
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

template<class T> struct TypedArrayTraits {
  typedef typename std::remove_cvref<T>::type value_type;

  static_assert(std::is_arithmetic<T>::value, "TypedArray must contain arithmetic type");
  static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4 || sizeof(T) == 8, "TypedArray must contain type of size 1, 2, 4 or 8");

  static constexpr size_t byte_size = sizeof(T);

  static constexpr bool is_signed = std::is_signed<value_type>::value;
  static constexpr bool is_floating_point = std::is_floating_point<value_type>::value;

  static TypedArrayProps
  getProps() {
    return TypedArrayProps(byte_size, is_signed, is_floating_point);
  }
};

static inline JSValue
js_typedarray_new(JSContext* ctx, JSValueConst buffer, uint32_t byteOffset, uint32_t length, const char* ctor_name) {
  JSValue global, ctor, ret;

  std::array<JSValueConst, 3> args = {buffer, js_number_new(ctx, byteOffset), js_number_new(ctx, length)};

  global = JS_GetGlobalObject(ctx);
  ctor = JS_GetPropertyStr(ctx, global, ctor_name);
  JS_FreeValue(ctx, global);

  ret = JS_CallConstructor(ctx, ctor, args.size(), &args[0]);
  JS_FreeValue(ctx, ctor);
  return ret;
}

static inline JSValue
js_typedarray_new(JSContext* ctx, JSValueConst buffer, uint32_t byteOffset, uint32_t length, const TypedArrayProps& props) {
  auto range = js_arraybuffer_range(ctx, buffer);
  assert(byteOffset + length * props.byte_size < range.size());

  return js_typedarray_new(ctx, buffer, byteOffset, length, props.constructor_name().c_str());
}

template<class Iterator>
static inline std::enable_if_t<std::is_pointer<Iterator>::value, void>
js_typedarray_remain(Iterator& start, Iterator& end, uint32_t byteOffset, uint32_t& length) {
  typedef typename std::remove_pointer<Iterator>::type value_type;

  const uint8_t* ptr;
  size_t len;
  ptr = reinterpret_cast<const uint8_t*>(start) + byteOffset;
  len = reinterpret_cast<const uint8_t*>(end) - ptr;

  len /= sizeof(value_type);

  if(length > len)
    len = length;
}

template<class T> class js_typedarray {
public:
  template<class Container>
  static JSValue
  from(JSContext* ctx, const Container& in) {
    return from_sequence<typename Container::const_iterator>(ctx, in.begin(), in.end());
  }

  static JSValue
  from_vector(JSContext* ctx, const std::vector<T>& in) {
    return from_sequence(ctx, in.begin(), in.end());
  }

  template<class Iterator>
  static JSValue
  from_sequence(JSContext* ctx, const Iterator& start, const Iterator& end, uint32_t byteOffset = 0, uint32_t length = UINT32_MAX) {
    JSValue buf = js_arraybuffer_from(ctx, start, end);
    uint32_t count = std::min<uint32_t>(length, end - start);

    js_typedarray_remain(start, end, byteOffset, count);

    return js_typedarray_new(ctx, buf, 0, count, TypedArrayTraits<T>::getProps());
  }

  template<size_t N> static int64_t to_array(JSContext* ctx, JSValueConst arr, std::array<T, N>& out);
  static int64_t to_scalar(JSContext* ctx, JSValueConst arr, cv::Scalar_<T>& out);
};

template<class Iterator>
static inline std::enable_if_t<std::is_pointer<Iterator>::value, JSValue>
js_typedarray_from(JSContext* ctx, const Iterator& start, const Iterator& end, uint32_t byteOffset = 0, uint32_t length = UINT32_MAX) {
  return js_typedarray<typename std::remove_pointer<Iterator>::type>::from_sequence(ctx, start, end, byteOffset);
}

template<class Iterator>
static inline std::enable_if_t<Iterator::value_type, JSValue>
js_typedarray_from(JSContext* ctx, const Iterator& start, const Iterator& end, uint32_t byteOffset = 0, uint32_t length = UINT32_MAX) {
  return js_typedarray<typename Iterator::value_type>::from_sequence(ctx, start, end, byteOffset);
}

template<class Container>
static inline JSValue
js_typedarray_from(JSContext* ctx, const Container& v, uint32_t byteOffset = 0, uint32_t length = UINT32_MAX) {
  return js_typedarray<typename Container::value_type>::from_sequence(ctx, v.begin(), v.end(), byteOffset);
}

#endif /* defined(JS_TYPED_ARRAY_HPP) */