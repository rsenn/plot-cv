import { F_GETFL, F_SETFL, fcntl, O_NONBLOCK } from './quickjs/qjs-ffi/lib/fcntl.js';
import * as ffi from 'ffi';
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
let mmap = foreign('mmap', 'ulong', 'pointer', 'size_t', 'int', 'int', 'int', 'size_t');
let munmap = foreign('munmap', 'void', 'ulong', 'ulong');
let fork = foreign('fork', 'int');
let strcpy = foreign('strcpy', 'pointer', 'pointer', 'pointer');
let setjmp = foreign('setjmp', 'int', 'pointer');
let longjmp = foreign('longjmp', 'int', 'pointer', 'int');
let printf = foreign('printf', 'int', 'string', 'pointer', 'pointer');

Util.define(ArrayBuffer.prototype, {
  toPointer(hint = 'string') {
    let out = new ArrayBuffer(100);
    sprintf(out, '%p', this);
    let ret = ArrayBufToString(out);
    switch (hint) {
      case 'bigint':
        ret = BigInt(ret);
        break;
    }
    return ret;
  }
});

class Registers extends ArrayBuffer {
  constructor(obj = {}) {
    super(256);
    // Object.assign(this, obj);
  }
  /* prettier-ignore */ get [Symbol.toStringTag]() {
    return `[Registers @ ${toPointer(this)} ]`;
  }

  /* prettier-ignore */ set rax(v) {
    new BigUint64Array(this, 0, 1)[0] = BigInt(v);
  }
  /* prettier-ignore */ get rax() {
    let a = new BigUint64Array(this, 0, 1);
    return a[0];
  }

  /* prettier-ignore */ set rbx(v) {
    new BigUint64Array(this, 8, 1)[0] = BigInt(v);
  }
  /* prettier-ignore */ get rbx() {
    let a = new BigUint64Array(this, 8, 1);
    return a[0];
  }

  /* prettier-ignore */ set rcx(v) {
    new BigUint64Array(this, 16, 1)[0] = BigInt(v);
  }
  /* prettier-ignore */ get rcx() {
    let a = new BigUint64Array(this, 16, 1);
    return a[0];
  }
  /* prettier-ignore */ set rdx(v) {
    new BigUint64Array(this, 24, 1)[0] = BigInt(v);
  }
  /* prettier-ignore */ get rdx() {
    let a = new BigUint64Array(this, 24, 1);
    return a[0];
  }
  /* prettier-ignore */ set rsi(v) {
    new BigUint64Array(this, 32, 1)[0] = BigInt(v);
  }
  /* prettier-ignore */ get rsi() {
    let a = new BigUint64Array(this, 32, 1);
    return a[0];
  }
  /* prettier-ignore */ set rdi(v) {
    new BigUint64Array(this, 40, 1)[0] = BigInt(v);
  }
  /* prettier-ignore */ get rdi() {
    let a = new BigUint64Array(this, 40, 1);
    return a[0];
  }
  /* prettier-ignore */ set rsp(v) {
    new BigUint64Array(this, 48, 1)[0] = BigInt(v);
  }
  /* prettier-ignore */ get rsp() {
    let a = new BigUint64Array(this, 48, 1);
    return a[0];
  }
  /* prettier-ignore */ set rbp(v) {
    new BigUint64Array(this, 56, 1)[0] = BigInt(v);
  }
  /* prettier-ignore */ get rbp() {
    let a = new BigUint64Array(this, 56, 1);
    return a[0];
  }

  toString() {
    return `Registers { .rax = ${this.rax}, .rbx = ${this.rbx}, .rcx = ${this.rcx}, .rdx = ${this.rdx}, .rsi = ${this.rsi}, .rdi = ${this.rdi}, .rsp = ${this.rsp}, .rbp = ${this.rbp} }`;
  }
}

async function main(...args) {
    //breakLength: 120,
    maxStringLength: 200,
    multiline: 1,
    alignMap: true
  });
  printf('%p %s\n', 0xdeadbeef00000000, '0xdeadbeef');

  console.log(getpid());

  for(let [name, value] of Object.entries(ffi)) console.log(`ffi.${name}:`, value);
  console.log(`ffi:`, ffi);
  const flagNames = Util.bitsToNames(Util.filterKeys(fcntl, /^O_/));
  let outBuf = new ArrayBuffer(256);
  let flags;
  let fd = 1;
  let newState = false;
  console.log('strdup:', strdup('BLAH').toString(16));
  console.log('dlsym_(RTLD_DEFAULT, "strdup"):', dlsym(RTLD_DEFAULT, 'strdup').toString(16));
  console.log('snprintf(outBuf, outBuf.byteLength, "%p", -1):', snprintf(outBuf, outBuf.byteLength, '%p', 0x7fffffffffffffff));
  console.log('outBuf:', ArrayBufToString(outBuf));
  console.log('Util.isatty(1):', await Util.isatty(1));
  console.log('F_GETFL:', toHex((flags = fcntl(fd, F_GETFL, 0))));

  if(newState) flags |= O_NONBLOCK;
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
  console.log('BigInt methods:', Util.getMethodNames(BigInt));
  console.log('BigInt toString(16):', BigInt(1337).toString(16));

  const MAP_ANONYMOUS = 0x20;

  let base_addr = 0x7f0000000000 - 1024;

  let area = mmap((0x2000000 || 0x7f0000000000) - 8192, 8192, 0x7, 0x02 | MAP_ANONYMOUS, -1, 0);
  console.log('area:', area.toString(16));
  let fp = dlsym(RTLD_DEFAULT, 'strchr');
  console.log('fp:', fp.toString(16));
  strcpy(area, '\x48\x31\xc0\xc3');
  //  strcpy(area+0, '\x48\x31\xc0\x48\xff\xc0\xc3');

  let returnRAX = area + 100;
  strcpy(area + 100, '\x48\x31\xc0\x48\xff\xc0\xc3');
  let returnADDR = area + 200;
  strcpy(area + 200, '\x48\x8b\x04\x24\xc3');
  let writeREGS = area + 300;
  strcpy(area + 300, '\xf3\x0f\x1e\xfa\x48\x89\x07\x48\x89\x5f\x08\x48\x89\x4f\x10\x48\x89\x57\x18\x48\x89\x77\x20\x48\x89\x7f\x28\x48\x89\x6f\x30\x48\x89\x67\x38\x48\x31\xc0\x48\xff\xc0\xc3');
  console.log('writeREGS:', writeREGS.toString(16));
  let ret;
  printf('area: %s\n', StringToHex(area + 0));

  printf('returnRAX: %p\n', +returnRAX);
  printf('returnADDR: %s\n', StringToHex(returnADDR));
  printf('writeREGS: %p\n', +writeREGS);
  printf('writeREGS: %s\n', StringToHex(writeREGS));

  define('asm', area, null, 'uint64', 'uint64', 'uint64');
  define('returnRAX', +returnRAX, null, 'uint64');
  define('returnADDR', +returnADDR, null, 'uint64');
  define('writeREGS', +writeREGS, null, 'uint64', 'pointer', 'uint64', 'pointer', 'uint64');

  let regs = new Registers();

  /* ret = call('returnRAX', 1, 2);
  console.log('returnRAX() =', ret);
  ret = call('returnADDR');
  console.log('returnADDR() =', ret);

  ret = call('writeREGS', +toPointer(regs), +toPointer(regs), regs, regs); // setjmp(jb);
  console.log('writeREGS() =', ret);*/

  console.log('regs =', regs.toString());
  //if(ret != 1337)
  // longjmp(jb, 1337);
}

main(...scriptArgs.slice(1));

function toHex(n, b = 2) {
  console.log('toHex:', n);

  let s = (+n).toString(16);
  return '0'.repeat(Math.ceil(s.length / b) * b - s.length) + s;
}

function StringToHex(str, bytes = 1) {
  if(typeof str != 'string') str = toString(str);
  let buf = StringToArrayBuffer(str, bytes);
  return ArrayBufToHex(buf, bytes);
}

function StringToArrayBuffer(str, bytes = 1) {
  const buf = new ArrayBuffer(str.length * bytes);
  const view = new Uint8Array(buf);
  for(let i = 0, strLen = str.length; i < strLen; i++) view[i] = str.charCodeAt(i);
  return buf;
}

function ArrayBufToString(buf, offset, length) {
  let arr = new Uint8Array(buf, offset || 0, length || buf.byteLength);
  let len = arr.indexOf(0);
  if(len != -1) arr = arr.slice(0, len);
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
  } catch(error) {
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

      if(sec !== undefined || usec !== undefined) {
        let a = new BigUint64Array(this);
        a[0] = BigInt(sec || 0n);
        a[1] = BigInt(usec || 0n);
      }
    }

    /* prettier-ignore */ set tv_sec(s) {
      let a = new BigUint64Array(this);
      a[0] = BigInt(s);
    }
    /* prettier-ignore */ get tv_sec() {
      let a = new BigUint64Array(this);
      return a[0];
    }
    /* prettier-ignore */ set tv_usec(us) {
      let a = new BigUint64Array(this);
      a[1] = BigInt(us);
    }
    /* prettier-ignore */ get tv_usec() {
      let a = new BigUint64Array(this);
      return a[1];
    }

    toString() {
      const { tv_sec, tv_usec } = this;
      return `{ .tv_sec = ${tv_sec}, .tv_usec = ${tv_usec} }`;
    }
  })(sec, usec);
}