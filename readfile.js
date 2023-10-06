import { platform, read } from 'os';
import { call, define, dlsym, RTLD_DEFAULT } from 'ffi';
import { _get_osfhandle } from 'misc';
function foreign(name, ret, ...args) {
  let fp = dlsym(RTLD_DEFAULT, name);
  define(name, fp, null, ret, ...args);
  return (...args) => call(name, ...args);
}

export const ReadFile =
  platform == 'win32'
    ? foreign('ReadFile', 'long', 'long', 'buffer', 'ulong', 'buffer', 'long')
    : (fd, buf, len, lpNumberOfBytesRead) => {
        let r = read(fd, 0, buf, len);
        new Uint32Array(lpNumberOfBytesRead)[0] = r;
        return r;
      };

//export const read = foreign('_read', 'long', 'long', 'buffer', 'ulong');

if(/readfile\.js$/.test(scriptArgs[0])) {
  let hnd = _get_osfhandle(0);

  console.log('hnd', hnd);

  let ab = new ArrayBuffer(1024);
  let lpNumberOfBytesRead = new Uint32Array(2);

  console.log('ReadFile() =', ReadFile(hnd, ab, 1024, lpNumberOfBytesRead.buffer, null));
  console.log('lpNumberOfBytesRead', lpNumberOfBytesRead);
}