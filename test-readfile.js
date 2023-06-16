import { debug, dlopen, define, dlerror, dlclose, dlsym, call, toString, toArrayBuffer, toPointer, errno, JSContext, RTLD_LAZY, RTLD_NOW, RTLD_GLOBAL, RTLD_LOCAL, RTLD_DEFAULT, RTLD_NEXT } from 'ffi';
import { _get_osfhandle } from 'misc';

function foreign(name, ret, ...args) {
  let fp = dlsym(RTLD_DEFAULT, name);
  define(name, fp, null, ret, ...args);
  return (...args) => call(name, ...args);
}

export const ReadFile = foreign('ReadFile', 'long', 'long', 'buffer', 'ulong', 'buffer', 'long');
export const read = foreign('_read', 'long', 'long', 'buffer', 'ulong');

if(/test-readfile\.js$/.test(scriptArgs[0])) {
  let hnd = _get_osfhandle(0);

  console.log('hnd', hnd);

  let ab = new ArrayBuffer(1024);
  let lpNumberOfBytesRead = new Uint32Array(2);

  console.log('ReadFile() =', ReadFile(hnd, ab, 1024, lpNumberOfBytesRead.buffer, null));
  console.log('lpNumberOfBytesRead', lpNumberOfBytesRead);
}
