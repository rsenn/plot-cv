import * as std from 'std';
import * as os from 'os';
import { O_NONBLOCK, F_GETFL, F_SETFL, fcntl } from './fcntl.js';
import {
  debug,
  dlopen,
  define,
  dlerror,
  dlclose,
  dlsym,
  call,
  toString,
  toArrayBuffer,
  errno,
  JSContext,
  RTLD_LAZY,
  RTLD_NOW,
  RTLD_GLOBAL,
  RTLD_LOCAL,
  RTLD_NODELETE,
  RTLD_NOLOAD,
  RTLD_DEEPBIND,
  RTLD_DEFAULT,
  RTLD_NEXT
} from 'ffi';
import * as ffi from 'ffi';
import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';

function foreign(name, ret, ...args) {
  let fp = dlsym(RTLD_DEFAULT, name);
  define(name, fp, null, ret, ...args);
  return (...args) => call(name, ...args);
}

let getpid = foreign('getpid', 'int');
let select = foreign('select', 'int', 'buffer', 'buffer', 'buffer', 'buffer', 'buffer');
let sprintf = foreign('sprintf', 'int', 'buffer', 'string', 'void *');
let strdup = foreign('strdup', 'void *', 'string');
let dlopen_ = foreign('dlopen', 'void *', 'string', 'int');
let dlsym_ = foreign('dlsym', 'void *', 'string');
let snprintf = foreign('snprintf', 'int', 'buffer', 'size_t', 'string', 'void *');

ArrayBuffer.prototype.toPointer = function (hint = 'string') {
  let out = new ArrayBuffer(100);
  sprintf(out, '%p', this);
  let ret = ArrayBufToString(out);
  switch (hint) {
    case 'bigint':
      ret = BigInt(ret);
      break;
  }
  return ret;
};

async function main(...args) {
  await ConsoleSetup({
    //breakLength: 120,
    maxStringLength: 200,
    maxArrayLength: 20,
    multiline: 1,
    alignMap: true
  });

  console.log(getpid());

  const flagNames = Util.bitsToNames(Util.filterKeys(fcntl, /^O_/));
  let outBuf = new ArrayBuffer(256);
  let flags;
  let fd = 1;
  let newState = false;
  console.log('ffi:', ffi);
  console.log('strdup:', strdup('BLAH').toString(16));
  console.log('dlsym_(RTLD_DEFAULT, "strdup"):', dlsym(RTLD_DEFAULT, 'strdup').toString(16));
  console.log('snprintf(outBuf, outBuf.byteLength, "%p", -1):', snprintf(outBuf, outBuf.byteLength, '%p', 0x7fffffffffffffff));
  console.log('outBuf:', ArrayBufToString(outBuf));
  console.log('Util.isatty(1):', await Util.isatty(1));
  console.log('F_GETFL:', toHex((flags = fcntl(fd, F_GETFL, 0))));

  if (newState) flags |= O_NONBLOCK;
  else flags &= ~O_NONBLOCK;

  console.log('fcntl:', [...flagNames(flags)]);
  console.log('ttyGetWinSize:', await Util.ttyGetWinSize(1));

  console.log('F_SETFL:', fcntl(fd, F_SETFL, flags));
  console.log('F_GETFL:', toHex(fcntl(fd, F_GETFL, 0)));
  let rfds = new ArrayBuffer(1024 / 8);
  let wfds = new ArrayBuffer(1024 / 8);
  let efds = new ArrayBuffer(1024 / 8);

  let t = new timeval(0, 1000);

  function FD_SET(sfd, set) {
    let a = new Uint8Array(set);
    let byte = sfd >> 3;
    let bit = sfd & 0x7;
    a[byte] |= 1 << bit;
  }
  FD_SET(0, rfds);
  FD_SET(1, rfds);

  let u8 = new Uint8Array([0x41, 0x42, 0x43, 0x44, 0]);

  console.log('u8.buffer.toPointer().toString():', u8.buffer.toPointer().toString());
  // const ptr = u8.buffer.toPointer();
  const ptr = ffi.toPointer(u8.buffer);
  console.log('ptr:', ptr);
  console.log('toString:', ffi.toString(ptr));

  console.log('timeval:', t.slice());
  console.log('select:', toHex(select(4, rfds, wfds, efds, t)));
  console.log('toHex:', toHex(1, 8));
  console.log('toHex:', [...Util.partition(toHex(1, 8), 2)]);
  console.log('BigUint64Array.BYTES_PER_ELEMENT:', BigUint64Array.BYTES_PER_ELEMENT1);
  let out = new ArrayBuffer(100);
  console.log('sprintf:', sprintf(out, '%p', rfds));
  console.log('out:', MakeArray(out, 1).toString());
  console.log('rfds.toPointer():', rfds.toPointer());
  console.log('rfds.toPointer():', rfds.toPointer());
  console.log('BigInt methods:', Util.getMethodNames(BigInt));
  console.log('BigInt toString(16):', BigInt(1337).toString(16));
}

Util.callMain(main, true);

function toHex(n, b = 2) {
  console.log('toHex:', n);

  let s = (+n).toString(16);
  return '0'.repeat(Math.ceil(s.length / b) * b - s.length) + s;
}

function ArrayBufToString(buf, offset, length) {
  let arr = new Uint8Array(buf, offset || 0, length || buf.byteLength);
  let len = arr.indexOf(0);
  if (len != -1) arr = arr.slice(0, len);
  return arr.reduce((s, code) => s + String.fromCharCode(code), '');
}

function MakeArray(buf, numBytes) {
  try {
    switch (numBytes) {
      case 8:
        return new BigUint64Array(buf);
      case 4:
        return new Uint32Array(buf);
      case 2:
        return new Uint16Array(buf);
      default:
        return new Uint8Array(buf);
    }
  } catch (error) {
    console.error(`MakeArray(${Util.className(buf)}[${buf.byteLength}], ${numBytes}): ${error.message}`);
  }
}

function ArrayBufToHex(buf, numBytes = 8) {
  let arr = MakeArray(buf, numBytes);
  return arr.reduce((s, code) => (s != '' ? s + ' ' : '') + ('000000000000000' + code.toString(16)).slice(-(numBytes * 2)), '');
}

function timeval(sec = 0, usec = 0) {
  return new (class timeval extends ArrayBuffer {
    constructor(sec, usec) {
      super(2 * 8);

      if (sec !== undefined || usec !== undefined) {
        let a = new BigUint64Array(this);
        a[0] = BigInt(sec || 0n);
        a[1] = BigInt(usec || 0n);
      }
    }

    set tv_sec(s) {
      let a = new BigUint64Array(this);
      a[0] = BigInt(s);
    }
    get tv_sec() {
      let a = new BigUint64Array(this);
      return a[0];
    }
    set tv_usec(us) {
      let a = new BigUint64Array(this);
      a[1] = BigInt(us);
    }
    get tv_usec() {
      let a = new BigUint64Array(this);
      return a[1];
    }

    toString() {
      const { tv_sec, tv_usec } = this;
      return `{ .tv_sec = ${tv_sec}, .tv_usec = ${tv_usec} }`;
    }
  })(sec, usec);
}
