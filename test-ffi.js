import * as std from 'std';
import * as os from 'os';
import { debug, dlopen, define, dlerror, dlclose, dlsym, call, toString, toArrayBuffer, errno, JSContext, RTLD_LAZY, RTLD_NOW, RTLD_GLOBAL, RTLD_LOCAL, RTLD_NODELETE, RTLD_NOLOAD, RTLD_DEEPBIND, RTLD_DEFAULT, RTLD_NEXT } from './ffi.so';

function foreign(name, ret, ...args) {
  let fp = dlsym(RTLD_DEFAULT, name);
  define(name, fp, null, ret, ...args);
  return (...args) => call(name, ...args);
}

let getpid = foreign('getpid', 'int');
let fcntl = foreign('fcntl', 'int', 'int', 'int', 'int');

console.log(getpid());

const F_DUPFD = 0;
const F_GETFD = 1;
const F_SETFD = 2;
const F_GETFL = 3;
const F_SETFL = 4;
const F_GETLK = 5;
const F_SETLK = 6;
const F_SETLKW = 7;
const F_GETLK64 = 8;
const F_SETLK64 = 9;
const F_SETLKW64 = 10;
const F_GETOWN = 11;
const F_SETOWN = 12;
const F_SETSIG = 13;
const F_GETSIG = 14;
const O_NONBLOCK = 0x0800;
const O_LARGEFILE = 0x8000;
const O_NOFOLLOW = 0x00020000;
const O_CLOEXEC = 0x00080000;
const O_NOATIME = 0x00040000;
let flags;
let fd = 1;
let newState = false;
console.log('F_GETFL:', toHex((flags = fcntl(fd, F_GETFL, 0))));
if(newState) flags |= O_NONBLOCK;
else flags &= ~O_NONBLOCK;

console.log('F_SETFL:', fcntl(fd, F_SETFL, flags));
console.log('F_GETFL:', toHex(fcntl(fd, F_GETFL, 0)));

function toHex(n, b = 2) {
  let s = (+n).toString(16);
  return '0x' + '0'.repeat(Math.ceil(s.length / b) * b - s.length) + s;
}
