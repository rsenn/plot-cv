#ifndef JS_ALLOC_H
#define JS_ALLOC_H

#include <unistd.h>
#include <sys/mman.h>

template<class T>
struct js_mmap {
  static constexpr size_t size = ((sizeof(T)+7)>>3)<<3;

  static T* allocate(JSContext* ctx) {
    return static_cast<T*>(mmap(nullptr,  size, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0));
  }
  static void deallocate(JSContext* ctx, T* ptr) {
    munmap(ptr,  size);
  }
};

template<class T>
struct js_libcmalloc  {
  static constexpr size_t size = ((sizeof(T)+7)>>3)<<3;

  static T* allocate(JSContext* ctx) {
    return static_cast<T*>(malloc(size));
  }

  static void deallocate(JSContext* ctx,T* ptr) {
    free(ptr);
  }
};

template<class T>
struct js_qjsalloc  {
  static constexpr size_t size = ((sizeof(T)+7)>>3)<<3;

  static T* allocate(JSContext* ctx) {
    return static_cast<T*>(js_mallocz(ctx, size));
  }

  static void deallocate(JSContext* ctx,T* ptr) {
    js_free(ctx, ptr);
  }
};

template<class T>
struct js_cxxnew  {
  static constexpr size_t size = ((sizeof(T)+7)>>3)<<3;

  static T* allocate(JSContext* ctx) {
    return new T();
  }

  static void deallocate(JSContext* ctx,T* ptr) {
    delete ptr;
  }
};

template<class T> using js_allocator = js_qjsalloc<T>;


template<class T>
T* js_allocate(JSContext*ctx) {
  return js_allocator<T>::allocate(ctx);
}

template<class T>
void js_deallocate(JSContext*ctx, T*ptr) {
 js_allocator<T>::deallocate(ctx, ptr);
}

#endif /* defined(JS_ALLOC_H) */
