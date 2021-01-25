#ifndef JS_ALLOC_H
#define JS_ALLOC_H

#include <unistd.h>
#include <sys/mman.h>
#include <sys/user.h>
#include "quickjs.h"
#include <cstdlib>

template<class T> struct js_mmap {
  static constexpr size_t page_size = PAGE_SIZE;
  static constexpr size_t
  round_to_page_size(size_t n) {
    size_t pages = (n + (page_size - 1)) / page_size;
    return pages * page_size;
  }
  static constexpr size_t size = round_to_page_size(sizeof(T));
  static constexpr size_t offset = size - sizeof(T);

  static T*
  allocate(void* ctx) {
    return reinterpret_cast<T*>(
        static_cast<char*>(
            mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0)) +
        offset);
  }
  static void
  deallocate(void* ctx, T* ptr) {
    munmap(reinterpret_cast<char*>(ptr) - offset, size);
  }
};

template<class T> struct js_libcmalloc {
  static constexpr size_t size = ((sizeof(T) + 7) >> 3) << 3;

  static T*
  allocate(void* ctx) {
    return static_cast<T*>(malloc(size));
  }

  static void
  deallocate(void* ctx, T* ptr) {
    free(ptr);
  }
};

template<class T> struct js_qjsalloc {
  static constexpr size_t size = ((sizeof(T) + 7) >> 3) << 3;

  static T*
  allocate(void* ctx) {
    return static_cast<T*>(js_mallocz((JSContext*)ctx, size));
  }

  static void
  deallocate(void* ctx, T* ptr) {
    js_free((JSContext*)ctx, ptr);
  }
};

template<class T> struct js_cxxnew {
  static constexpr size_t size = ((sizeof(T) + 7) >> 3) << 3;

  static T*
  allocate(void* ctx) {
    return new T();
  }

  static void
  deallocate(void* ctx, T* ptr) {
    delete ptr;
  }
};

template<class T> using js_allocator = js_qjsalloc<T>;

template<class T>
static inline T*
js_allocate(void* ctx) {
  return js_allocator<T>::allocate(ctx);
}

template<class T>
static inline void
js_deallocate(void* ctx, T* ptr) {
  js_allocator<T>::deallocate(ctx, ptr);
}

#endif /* defined(JS_ALLOC_H) */
