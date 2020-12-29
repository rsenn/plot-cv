import * as std from 'std';
import * as os from 'os';
import { O_NONBLOCK, F_GETFL, F_SETFL, fcntl } from './fcntl.js';
import { debug, dlopen, define, dlerror, dlclose, dlsym, call, toString, toArrayBuffer, errno, JSContext, RTLD_LAZY, RTLD_NOW, RTLD_GLOBAL, RTLD_LOCAL, RTLD_NODELETE, RTLD_NOLOAD, RTLD_DEEPBIND, RTLD_DEFAULT, RTLD_NEXT } from './ffi.so';
import Util from './lib/util.js';

function foreign(name, ret, ...args) {
  let fp = dlsym(RTLD_DEFAULT, name);
  define(name, fp, null, ret, ...args);
  return (...args) => call(name, ...args);
}

let getpid = foreign('getpid', 'int');
let select = foreign('select', 'int', 'buffer', 'buffer', 'buffer', 'string');

console.log(getpid());

let flags;
let fd = 1;
let newState = false;
console.log('F_GETFL:', toHex((flags = fcntl(fd, F_GETFL, 0))));
if(newState) flags |= O_NONBLOCK;
else flags &= ~O_NONBLOCK;

console.log('F_SETFL:', fcntl(fd, F_SETFL, flags));
console.log('F_GETFL:', toHex(fcntl(fd, F_GETFL, 0)));
let rfds = new ArrayBuffer(1024 / 8);
let wfds = new ArrayBuffer(1024 / 8);
let efds = new ArrayBuffer(1024 / 8);

let timeval = new ArrayBuffer(4 * 8);

function TIMEVAL(sec, usec) {
  let a = Uint16Array.of(sec & 0xffff,
    (sec >> 16) & 0xfff,
    (sec >> 32) & 0xfff,
    (sec >> 48) & 0xfff,
    0,
    0,
    0,
    0
  );
  return a.buffer;
}

function FD_SET(sfd, set) {
  let a = new Uint8Array(set);
  let byte = sfd >> 3;
  let bit = sfd & 0x7;
  a[byte] |= 1 << bit;
}
FD_SET(0, rfds);
FD_SET(1, rfds);
timeval = TIMEVAL(1, 0);

console.log('timeval:', ArrayBufToHex(timeval.slice(), 1));
console.log('select:', toHex(select(4, rfds, wfds, efds, timeval)));
console.log('toHex:', toHex(1, 8));
console.log('toHex:', [...Util.partition(toHex(1, 8), 2)]);

function toHex(n, b = 2) {
  let s = (+n).toString(16);
  return '0'.repeat(Math.ceil(s.length / b) * b - s.length) + s;
}
function ArrayBufToString(buf, offset, length) {
  let arr = new Uint8Array(buf, offset, length);
  return arr.reduce((s, code) => s + String.fromCodePoint(code), '');
}

function MakeArray(buf, numBytes) {
  switch (numBytes) {
    case 8:
      return new BigUint64Array(buf);
    case 4:
      return new Uint32Array(buf);
    case 2:
      return new Uint16Array(buf);
    default: return new Uint8Array(buf);
  }
}

function ArrayBufToHex(buf, numBytes = 8) {
  let arr = MakeArray(buf, numBytes);
  return arr.reduce((s, code) =>
      (s != '' ? s + ' ' : '') + ('000000000000000' + code.toString(16)).slice(-(numBytes * 2)),
    ''
  );
}
