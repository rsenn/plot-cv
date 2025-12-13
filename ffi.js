import { call, debug, define, dlclose, dlerror, dlopen, dlsym, errno, JSContext, RTLD_DEFAULT, RTLD_NOW, toArrayBuffer, toString } from 'ffi';

/* test.js
 *
 * Test harness for JavaScript ffi
 *
 * Tektonics:
 *
 *   Build shared object:
 *
 *   gcc -g -fPIC -DJS_SHARED_LIBRARY -c ffi.c
 *   gcc -g -shared -o ffi.so ffi.o -lffi -ldl
 *
 *   Debug shared object:
 *
 *   gdb qjs
 *     set args test.js
 *     b js_debug
 *     run
 *
 *   This will stop in gdb on the first debug(); call.
 *
 */
async function main(...args) {

  var h;
  var r;
  console.log('Hello World');
  debug();
  console.log('RTLD_NOW = ', RTLD_NOW);
  /* Expect an error -- libc.so is (usually) a linker script */
  console.log('dlopen = ', (r = dlopen('libc.so.6', RTLD_NOW)));
  if(r == null) console.log('dlerror = ', dlerror());
  /* But, using libc.so.6 should work */
  console.log('dlopen = ', (h = dlopen('libc.so.6', RTLD_NOW)));
  if(h == null) console.log('dlerror = ', dlerror());
  console.log('dlsym = ', (r = dlsym(h, 'malloc')));
  if(r == null) console.log('dlerror = ', dlerror());
  console.log('dlclose = ', (r = dlclose(h)));
  if(r != 0) console.log('dlerror = ', dlerror());
  console.log('dlopen = ', (h = dlopen(null, RTLD_NOW)));
  if(h == null) console.log('dlerror = ', dlerror());
  console.log('dlsym = ', (r = dlsym(h, 'malloc')));
  if(r == null) console.log('dlerror = ', dlerror());
  console.log('dlclose = ', (r = dlclose(h)));
  if(r != 0) console.log('dlerror = ', dlerror());
  var malloc;
  console.log('dlsym = ', (malloc = dlsym(RTLD_DEFAULT, 'malloc')));
  if(malloc == null) console.log('dlerror = ', dlerror());
  var free;
  console.log('dlsym = ', (free = dlsym(RTLD_DEFAULT, 'free')));
  if(free == null) console.log('dlerror = ', dlerror());

  /* We have function pointers to malloc and free -- define the ffi
   * functions
   */
  define('malloc', malloc, null, 'void *', 'size_t');
  define('free', free, null, 'void', 'void *');

  /* p = malloc(10); display pointer, free(p)
   */
  var p;
  p = call('malloc', 10);
  console.log(p);
  call('free', p);

  /* n = strlen("hello"); which should result in 5
   */
  var strlen;
  strlen = dlsym(RTLD_DEFAULT, 'strlen');
  if(strlen == null) console.log(dlerror());
  define('strlen', strlen, null, 'int', 'char *');

  var n;
  n = call('strlen', 'hello');
  /* We expect 5 */
  console.log(n);

  /* p = strdup("dup this").
   */
  var strdup;
  strdup = dlsym(RTLD_DEFAULT, 'strdup');
  if(strdup == null) console.log(dlerror());
  define('strdup', strdup, null, 'char *', 'char *');

  p = call('strdup', 'dup this');

  /* Convert strdup() result into a string (should display
   * dup this 8
   */
  var s;
  s = toString(p);
  console.log(s, call('strlen', p));

  console.log();
  console.log('testing test.so functions');
  h = dlopen('./test.so', RTLD_NOW);
  if(h == null) console.log("can't load ./test.so: ", dlerror());
  var fp;
  fp = dlsym(h, 'test1');
  if(fp == null) console.log("can't find symbol test1: ", dlerror());
  if(!define('test1', fp, null, 'int', 'void *')) console.log("can't define test1");
  /* test1 takes a buffer but a string will work -- changes to the string
   * are lost, because a writable buffer is passed, but discarded before
   * the return.
   */
  r = call('test1', 'abc');
  console.log('should be 5: ', r);
  /* pass buffer to test1 -- test1 changes the buffer in place, and this
   * is reflected in quickjs
   */
  var b;
  b = new ArrayBuffer(8);
  var u;
  u = new Uint8Array(b);
  u[0] = 1;
  u[1] = 2;
  u[2] = 3;
  console.log('should print 1 2 3');
  r = call('test1', b);
  console.log('should print 3,2,1,0,0,0,0,0');
  console.log(u);

  /* p is a pointer to "dup this" -- 9 bytes of memory
   */
  b = toArrayBuffer(p, 9);
  u = new Uint8Array(b);
  console.log(u);

  call('free', p);

  fp = dlsym(RTLD_DEFAULT, 'strtoul');
  if(fp == null) console.log(dlerror());
  define('strtoul', fp, null, 'ulong', 'string', 'string', 'int');
  n = call('strtoul', '1234', null, 0);
  console.log(n, 'Should be 1234');
  call('strtoul', '1234567890123456789012345678901234567890', null, 0);
  console.log(errno(), 'should be 34 (ERANGE)');

  p = JSContext();
  console.log('jscontext = ', p);

  function syscall(name, retval, ...args) {
    let fn = dlsym(RTLD_DEFAULT, name);
    if(fn == null) console.log(dlerror());
    define(name, fn, null, retval, ...args);
    return (...args) => call(name, ...args);
  }

  function sockaddr_in(af = 0, port = 0, addr = '0.0.0.0') {
    let buf = new ArrayBuffer(16);
    let arr = new Uint8Array(buf);
    arr[0] = af & 0xff;
    arr[1] = af >> 8;
    arr[2] = port >> 8;
    arr[3] = port & 0xff;

    addr
      .split(/\./g)
      .map(n => +n)
      .forEach((n, i) => (arr[4 + i] = n));
    Object.assign(buf, {
      /* prettier-ignore */ get af() {
        let arr = new Uint8Array(this);
        return arr[0] | (arr[1] << 8);
      },
      /* prettier-ignore */ get port() {
        let arr = new Uint8Array(this);
        return arr[3] | (arr[2] << 8);
      },
      /* prettier-ignore */ get addr() {
        let arr = new Uint8Array(this);
        return arr
          .slice(4, 8)
          .map((n) => n + '')
          .join('.');
      }
    });
    return buf;
  }
  function getu32(buf) {
    let arr = new Uint8Array(buf);
    return (arr[3] << 24) | (arr[2] << 16) | (arr[1] << 8) | arr[0];
  }
  function setu32(buf, num) {
    let arr = new Uint8Array(buf);
    arr[3] = (num >> 24) & 0xff;
    arr[2] = (num >> 16) & 0xff;
    arr[1] = (num >> 8) & 0xff;
    arr[0] = num & 0xff;
  }

  function str2buf(str) {
    let buf = new ArrayBuffer(str.length);
    let arr = new Uint8Array(buf);
    for(let i = 0; i < str.length; i++) arr[i] = str.codePointAt(i);
    return buf;
  }

  function buf2str(buf, len) {
    let arr = new Uint8Array(buf);
    let s = '';
    len = len || arr.length;
    for(let i = 0; i < len; i++) s += String.fromCodePoint(arr[i]);
    return s;
  }

  function fd_set(size = 64) {
    let buf = new ArrayBuffer(size / 8);
    return buf;
  }
  function FD_SET(buf, n) {
    let arr = new Uint8Array(buf);
    arr[n >> 3] |= 1 << (n & 0x3);
    return buf;
  }
  function FD_CLR(buf, n) {
    let arr = new Uint8Array(buf);
    arr[n >> 3] &= ~(1 << (n & 0x3));
    return buf;
  }
  function FD_ISSET(buf, n) {
    let arr = new Uint8Array(buf);
    return arr[n >> 3] & (1 << (n & 0x3));
  }
  function FD_ZERO(buf) {
    let arr = new Uint8Array(buf);
    for(let i = 0; i < buf.byteLength; i++) arr[i] = 0;
    return buf;
  }
  function fd_array(buf) {
    let arr = new Uint8Array(buf);
    let size = buf.byteLength * 8;
    let ret = [];
    for(let fd = 0; fd < size; fd++) {
      if((+arr[fd >> 3] >> (fd & 0x7)) & 1) ret.push(fd);
    }
    return ret;
  }

  let socket = syscall('socket', 'int', 'int', 'int', 'int');
  let fd = socket(2, 1, 6);
  console.log('fd = ', fd);
  if(fd == -1) console.log('errno() = ', errno());

  let fcntl = syscall('fcntl', 'int', 'int', 'int', 'int');

  const F_DUPFD = 0;
  const F_GETFD = 1;
  const F_SETFD = 2;
  const F_GETFL = 3;
  const F_SETFL = 4;

  let ret;

  let connect = syscall('connect', 'int', 'int', 'void *', 'int');
  let sa = sockaddr_in(2, 3000, '127.0.0.1');

  console.log('sa = ', sa);
  console.log('sa.byteLength = ', sa.byteLength);
  ret = connect(fd, sa, sa.byteLength);
  console.log('ret = ', ret);
  if(ret == -1) console.log('errno() = ', errno());

  let flags = fcntl(fd, F_GETFL);
  console.log('fcntl() flags = ', ret);
  if(flags == -1) console.log('fcntl() errno() = ', errno());
  console.log(`fcntl(${fd}, F_SETFL, 0o${(+flags | 0o4000).toString(8)})`);
  fcntl(fd, F_SETFL, flags | 0o4000);
  flags = fcntl(fd, F_GETFL);
  console.log('fcntl() flags = ', ret);

  let send = syscall('send', 'int', 'int', 'void *', 'int', 'int');
  let recv = syscall('recv', 'int', 'int', 'void *', 'int', 'int');

  let req = str2buf('GET / HTTP/1.0\r\nHost: 127.0.0.1\r\n\r\n');
  ret = send(fd, req, req.byteLength, 0);
  console.log('ret = ', ret);

  let getsockname = syscall('getsockname', 'int', 'int', 'void *', 'void *');
  let getpeername = syscall('getpeername', 'int', 'int', 'void *', 'void *');

  let namelen = new ArrayBuffer(8);
  setu32(namelen, 16);
  let addr = sockaddr_in();
  ret = getsockname(fd, addr, namelen);
  console.log('getsockname() ret = ', ret);
  console.log('getsockname() getu32(namelen) = ', getu32(namelen));
  setu32(namelen, 16);
  console.log('getu32(namelen) = ', getu32(namelen));
  addr = sockaddr_in();

  ret = getpeername(fd, addr, namelen);
  console.log('getpeername() ret = ', ret);
  console.log('getpeername() getu32(namelen) = ', getu32(namelen));

  console.log('getpeername() addr = ', buf2str(addr));
  console.log('getpeername() addr.af = ', addr.af);
  console.log('getpeername() addr.port = ', addr.port);
  console.log('getpeername() addr.addr = ', addr.addr);

  let select = syscall('select', 'int', 'int', 'void *', 'void *', 'void *', 'void *');

  let rfd = fd_set(),
    wfd = fd_set(),
    efd = fd_set();

  FD_ZERO(rfd);
  FD_ZERO(wfd);
  FD_ZERO(efd);

  FD_SET(rfd, fd);
  //FD_SET(wfd, fd);
  FD_SET(efd, fd);

  ret = select(64, rfd, wfd, efd, null);

  console.log('ret = ', ret);
  console.log('rfd = ', fd_array(rfd));
  console.log('wfd = ', fd_array(wfd));
  console.log('efd = ', fd_array(efd));
  b = new ArrayBuffer(1024);
  ret = recv(fd, b, b.byteLength, 0);
  console.log('ret = ', ret);
  console.log('b = ', buf2str(b, ret));
}

main(...scriptArgs.slice(1));