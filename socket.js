import * as std from 'std';
import * as os from 'os';
import { O_NONBLOCK, F_GETFL, F_SETFL, fcntl } from './fcntl.js';
import { debug, dlopen, define, dlerror, dlclose, dlsym, call, toString, toArrayBuffer, errno, JSContext, RTLD_LAZY, RTLD_NOW, RTLD_GLOBAL, RTLD_LOCAL, RTLD_NODELETE, RTLD_NOLOAD, RTLD_DEEPBIND, RTLD_DEFAULT, RTLD_NEXT } from 'ffi';
import Util from './lib/util.js';

function foreign(name, ret, ...args) {
  let fp = dlsym(RTLD_DEFAULT, name);
  define(name, fp, null, ret, ...args);
  return (...args) => call(name, ...args);
}

export let FD_SETSIZE = 1024;

export const SOCK_STREAM = 1; /* stream (connection) socket */
export const SOCK_DGRAM = 2; /* datagram (conn.less) socket  */
export const SOCK_RAW = 3; /* raw socket     */
export const SOCK_RDM = 4; /* reliably-delivered message */
export const SOCK_SEQPACKET = 5; /* sequential packet socket */
export const SOCK_DCCP = 6; /* Datagram Congestion Control Protocol socket */
export const SOCK_PACKET = 10; /* linux specific way of  */

export const AF_UNIX = 1; /* Unix domain sockets          */
export const AF_LOCAL = 1; /* POSIX name for AF_UNIX       */
export const AF_INET = 2; /* Internet IP Protocol         */
export const AF_AX25 = 3; /* Amateur Radio AX.25          */
export const AF_IPX = 4; /* Novell IPX                   */
export const AF_APPLETALK = 5; /* AppleTalk DDP                */
export const AF_NETROM = 6; /* Amateur Radio NET/ROM        */
export const AF_BRIDGE = 7; /* Multiprotocol bridge         */
export const AF_ATMPVC = 8; /* ATM PVCs                     */
export const AF_X25 = 9; /* Reserved for X.25 project    */
export const AF_INET6 = 10; /* IP version 6                 */

export const IPPROTO_ROUTING = 43; /* IPv6 routing header    */
export const IPPROTO_FRAGMENT = 44; /* IPv6 fragmentation header  */
export const IPPROTO_ICMPV6 = 58; /* ICMPv6     */
export const IPPROTO_NONE = 59; /* IPv6 no next header    */
export const IPPROTO_DSTOPTS = 60; /* IPv6 destination options */

export const IPPROTO_IP = 0;
export const IPPROTO_ICMP = 1; /* Internet Control Message Protocol  */
export const IPPROTO_IGMP = 2; /* Internet Group Management Protocol */
export const IPPROTO_IPIP = 4; /* IPIP tunnels (older KA9Q tunnels use 94) */
export const IPPROTO_TCP = 6; /* Transmission Control Protocol  */
export const IPPROTO_EGP = 8; /* Exterior Gateway Protocol    */
export const IPPROTO_PUP = 12; /* PUP protocol       */
export const IPPROTO_UDP = 17; /* User Datagram Protocol   */
export const IPPROTO_IDP = 22; /* XNS IDP protocol     */
export const IPPROTO_RSVP = 46; /* RSVP protocol      */
export const IPPROTO_GRE = 47; /* Cisco GRE tunnels (rfc 1701,1702)  */
export const IPPROTO_IPV6 = 41; /* IPv6-in-IPv4 tunnelling    */
export const IPPROTO_PIM = 103; /* Protocol Independent Multicast */
export const IPPROTO_ESP = 50; /* Encapsulation Security Payload protocol */
export const IPPROTO_AH = 51; /* Authentication Header protocol       */
export const IPPROTO_COMP = 108; /* Compression Header protocol */
export const IPPROTO_SCTP = 132; /* Stream Control Transmission Protocol.  */
export const IPPROTO_UDPLITE = 136; /* UDP-Lite protocol.  */
export const IPPROTO_RAW = 255; /* Raw IP packets     */

export const EPERM = 1;
export const ENOENT = 2;
export const EINTR = 4;
export const EBADF = 9;
export const EAGAIN = 11;
export const ENOMEM = 12;
export const EACCES = 13;
export const EFAULT = 14;
export const ENOTDIR = 20;
export const EINVAL = 22;
export const ENFILE = 23;
export const EMFILE = 24;
export const EROFS = 30;
export const ENAMETOOLONG = 36;
export const ELOOP = 40;
export const ENOTSOCK = 88;
export const EPROTOTYPE = 91;
export const EPROTONOSUPPORT = 93;
export const EOPNOTSUPP = 95;
export const EAFNOSUPPORT = 97;
export const EADDRINUSE = 98;
export const EADDRNOTAVAIL = 99;
export const ENETUNREACH = 101;
export const ENOBUFS = 105;
export const EISCONN = 106;
export const ETIMEDOUT = 110;
export const ECONNREFUSED = 111;
export const EALREADY = 114;
export const EINPROGRESS = 115;

export const SO_DEBUG = 1;
export const SO_REUSEADDR = 2;
export const SO_TYPE = 3;
export const SO_ERROR = 4;
export const SO_DONTROUTE = 5;
export const SO_BROADCAST = 6;
export const SO_SNDBUF = 7;
export const SO_RCVBUF = 8;
export const SO_KEEPALIVE = 9;
export const SO_OOBINLINE = 10;
export const SO_NO_CHECK = 11;
export const SO_PRIORITY = 12;
export const SO_LINGER = 13;
export const SO_BSDCOMPAT = 14;
export const SO_REUSEPORT = 15;
export const SO_PASSCRED = 16;
export const SO_PEERCRED = 17;
export const SO_RCVLOWAT = 18;
export const SO_SNDLOWAT = 19;
export const SO_RCVTIMEO = 20;
export const SO_SNDTIMEO = 21;
export const SO_ACCEPTCONN = 30;
export const SO_SECURITY_AUTHENTICATION = 22;
export const SO_SECURITY_ENCRYPTION_TRANSPORT = 23;
export const SO_SECURITY_ENCRYPTION_NETWORK = 24;
export const SO_BINDTODEVICE = 25;
export const SO_ATTACH_FILTER = 26;
export const SO_DETACH_FILTER = 27;
export const SO_PEERNAME = 28;
export const SO_TIMESTAMP = 29;

export const SOL_SOCKET = 1;

const { read, write, close, setReadHandler, setWriteHandler } = os;

const syscall = {
  socket: foreign('socket', 'int', 'int', 'int', 'int'),
  select: foreign('select', 'int', 'buffer', 'buffer', 'buffer', 'string'),
  connect: foreign('connect', 'int', 'int', 'void *', 'size_t'),
  bind: foreign('bind', 'int', 'int', 'void *', 'size_t'),
  listen: foreign('listen', 'int', 'int'),
  accept: foreign('accept', 'int', 'int', 'buffer', 'buffer'),
  getsockopt: foreign('getsockopt', 'int', 'int', 'int', 'int', 'void *', 'buffer'),
  setsockopt: foreign('setsockopt', 'int', 'int', 'int', 'int', 'void *', 'size_t'),
  read,
  write,
  close,
  __errno_location: foreign('__errno_location', 'void *'),
  get errno() {
    return errno();
  },
  strerror: std.strerror
};

export const Error = Object.fromEntries(Object.getOwnPropertyNames(std.Error).map(name => [std.Error[name], name])
);

export function getError() {
  return syscall.errno;
}

export function strerror(errno) {
  return syscall.strerror(errno);
}

export function socket(af = AF_INET, type = SOCK_STREAM, proto = IPPROTO_IP) {
  return syscall.socket(af, type, proto);
}

export function ndelay(fd, on = true) {
  let flags = fcntl(+fd, F_GETFL);

  console.log('F_GETFL:', flags.toString(16));
  if(on) flags |= O_NONBLOCK;
  else flags &= ~O_NONBLOCK;

  return fcntl(+fd, F_SETFL, flags);
}

export function connect(fd, addr, addrlen) {
  if(!(typeof addrlen == 'number')) addrlen = addr.byteLength;

  return syscall.connect(+fd, addr, addrlen);
}

export function bind(fd, addr, addrlen) {
  if(!(typeof addrlen == 'number')) addrlen = addr.byteLength;

  return syscall.bind(+fd, addr, addrlen);
}

export function accept(fd, addr, addrlen) {
  if(addr == undefined) addr = null;
  if(addrlen == undefined) addrlen = null;

  return syscall.accept(+fd, addr, addrlen);
}

export function listen(fd, backlog = 5) {
  return syscall.listen(+fd, backlog);
}

export function recv(fd, buf, offset, len) {
  if(typeof buf.buffer == 'object') buf = buf.buffer;

  if(offset == undefined) offset = 0;
  if(len == undefined) len = buf.byteLength;
  return syscall.read(+fd, buf, offset, len);
}

export function send(fd, buf, offset, len) {
  if(typeof buf == 'string') buf = StringToArrayBuffer(buf);
  else if(typeof buf.buffer == 'object') buf = buf.buffer;
  if(offset == undefined) offset = 0;
  if(len == undefined) len = buf.byteLength;
  return syscall.write(+fd, buf, offset, len);
}

export function select(nfds, readfds = null, writefds = null, exceptfds = null, timeout = null) {
  if(!(typeof nfds == 'number')) {
    let maxfd = Math.max(...[readfds, writefds, exceptfds].filter(s => s instanceof fd_set).map(s => s.maxfd)
    );
    nfds = maxfd + 1;
  }
  return syscall.select(nfds, readfds, writefds, exceptfds, timeout);
}

export function getsockopt(sockfd, level, optname, optval, optlen) {
  optlen = optlen || optval.byteLength;

  return syscall.getsockopt(sockfd, level, optname, optval, optlen);
}

export function setsockopt(sockfd, level, optname, optval, optlen) {
  optlen = optlen || optval.byteLength;

  return syscall.setsockopt(sockfd, level, optname, optval, optlen);
}

export class timeval extends ArrayBuffer {
  constructor(sec = 0, usec = 0) {
    super(2 * 8);

    this.sec = sec;
    this.usec = usec;
  }

  set tv_sec(s) {
    let a = new Uint32Array(this);
    a[0] = s;
  }

  get tv_sec() {
    let a = new Uint32Array(this);
    return a[0];
  }

  set tv_usec(us) {
    let a = new Uint32Array(this);
    a[1] = us;
  }

  get tv_usec() {
    let a = new Uint32Array(this);
    return a[1];
  }

  toString() {
    const { tv_sec, tv_usec } = this;
    return `{ .tv_sec = ${tv_sec}, .tv_usec = ${tv_usec} }`;
  }
}

export class sockaddr_in extends ArrayBuffer {
  constructor(family, port, addr) {
    super(16);

    this.sin_family = family;
    this.sin_port = port;
    this.sin_addr = addr;
  }

  [Symbol.toPrimitive](hint) {
    return this.toString();
  }

  toString() {
    return `${this.sin_addr}:${this.sin_port}`;
  }

  get [Symbol.toStringTag]() {
    const { sin_family, sin_port, sin_addr } = this;
    return `{ .sin_family = ${sin_family}, .sin_port = ${sin_port}, .sin_addr = ${sin_addr} }`;
  }
}

Object.defineProperties(sockaddr_in.prototype, {
  sin_family: {
    set(af) {
      let a = new Uint16Array(this);
      a[0] = af;
    },
    get() {
      let a = new Uint16Array(this);
      return a[0];
    },
    enumerable: true
  },
  sin_port: {
    set(port) {
      let a = new Uint8Array(this);
      a[2] = port >> 8;
      a[3] = port & 0xff;
    },
    get() {
      let a = new Uint8Array(this);
      return (a[2] << 8) | a[3];
    },
    enumerable: true
  },
  sin_addr: {
    set(addr) {
      if(typeof addr == 'string') addr = addr.split(/[.:]/).map(n => +n);

      if(addr instanceof Array) {
        let a = new Uint8Array(this);
        a[4] = addr[0];
        a[5] = addr[1];
        a[6] = addr[2];
        a[7] = addr[3];
      } else {
        let a = new Uint32Array(this);
        a[1] = addr;
      }
    },
    get() {
      let a = new Uint8Array(this);
      return a.slice(4, 8).join('.');
    },
    enumerable: true
  }
});

export class fd_set extends ArrayBuffer {
  constructor() {
    super(FD_SETSIZE / 8);
  }

  get size() {
    return this.byteLength * 8;
  }

  get maxfd() {
    let a = this.array;
    return a[a.length - 1];
  }

  get array() {
    let a = new Uint8Array(this);
    let fds = [];
    for(let i = 0; i < a.byteLength; i++) {
      let b = a[i];

      for(let j = 0; j < 8; j++) {
        if(b & (1 << j)) fds.push(i * 8 + j);
      }
    }
    return fds;
  }

  toString() {
    return `[ ${this.array.join(', ')} ]`;
  }
}

export class socklen_t extends ArrayBuffer {
  constructor(value) {
    super(4);

    if(value != undefined) {
      let a = new Uint32Array(this);
      a[0] = value | 0;
    }
  }

  [Symbol.toPrimitive](hint) {
    let a = new Uint32Array(this);
    return a[0];
  }

  [Symbol.toStringTag] = `[object socklen_t]`;
}

export function FD_SET(fd, set) {
  let a = new Uint8Array(set);
  let byte = fd >> 3;
  let bit = fd & 0x7;
  a[byte] |= 1 << bit;
}

export function FD_CLR(fd, set) {
  let a = new Uint8Array(set);
  let byte = fd >> 3;
  let bit = fd & 0x7;
  a[byte] &= ~(1 << bit);
}

export function FD_ISSET(fd, set) {
  let a = new Uint8Array(set);
  let byte = fd >> 3;
  let bit = fd & 0x7;
  return !!(a[byte] & (1 << bit));
}

export function FD_ZERO(fd, set) {
  let a = new Uint8Array(set);
  for(let i = 0; i < a.byteLength; i++) a[i] = 0;
}

function StringToArrayBuffer(str) {
  return Uint8Array.from(str.split('').map(ch => ch.charCodeAt(0))).buffer;
}

export class Socket {
  constructor(proto = IPPROTO_IP) {
    this.type = [IPPROTO_UDP, SOCK_DGRAM].indexOf(proto) != -1 ? SOCK_DGRAM : SOCK_STREAM;
    this.fd = socket(this.family, this.type, proto);
    this.remote = new sockaddr_in(this.family);
    this.local = new sockaddr_in(this.family);
    this.pending = true;
  }

  /* prettier-ignore */ set remoteFamily(family) { this.remote.sin_family = family; }
  /* prettier-ignore */ get remoteFamily() { return this.remote.sin_family; }
  /* prettier-ignore */ set remoteAddress(a) { this.remote.sin_addr = a; }
  /* prettier-ignore */ get remoteAddress() { return this.remote.sin_addr; }
  /* prettier-ignore */ set remotePort(n) { this.remote.sin_port = n; }
  /* prettier-ignore */ get remotePort() { return this.remote.sin_port; }

  /* prettier-ignore */ set localFamily(family) { this.local.sin_family = family; }
  /* prettier-ignore */ get localFamily() { return this.local.sin_family; }
  /* prettier-ignore */ set localAddress(a) { this.local.sin_addr = a; }
  /* prettier-ignore */ get localAddress() { return this.local.sin_addr; }
  /* prettier-ignore */ set localPort(n) { this.local.sin_port = n; }
  /* prettier-ignore */ get localPort() { return this.local.sin_port; }

  connect(addr, port) {
    let ret;
    if(addr != undefined) this.remoteAddress = addr;
    if(port != undefined) this.remotePort = port;
    if((ret = connect(this.fd, this.remote, this.remote.byteLength)) == -1) {
      this.errno = syscall.errno;

      if(this.errno == EINPROGRESS) this.connecting = true;
    }
    return ret;
  }

  bind(addr, port) {
    let ret;
    if(addr != undefined) this.localAddress = addr;
    if(port != undefined) this.localPort = port;
    if((ret = bind(this.fd, this.local, this.local.byteLength)) == -1) this.errno = syscall.errno;

    setsockopt(this.fd, SOL_SOCKET, SO_REUSEADDR, new socklen_t(1));

    return ret;
  }

  listen(backlog = 5) {
    let ret;
    if((ret = listen(this.fd, backlog)) == -1) this.errno = syscall.errno;
    return ret;
  }

  accept(remote = new sockaddr_in(this.family)) {
    let len = new socklen_t(remote.byteLength);
    let ret = accept(this.fd, remote, len);

    //console.log(`Socket.accept(${remote}, ${len})`, { ret, remote });
    if(ret == -1) this.errno = syscall.errno;
    else
      ret = Object.create(Socket.prototype, {
        fd: { value: ret, enumerable: true },
        local: { value: this.local, enumerable: true },
        remote: { value: remote, enumerable: true }
      });
    return ret;
  }

  read(...args) {
    let ret;
    const [buf, offset, len] = args;
    if(args.length == 0 || typeof buf != 'object') {
      let data = new ArrayBuffer(typeof buf == 'number' ? buf : 1024);
      if((ret = this.read(data)) > 0) return dat.aslice(0, ret);
    } else if((ret = recv(this.fd, buf, offset, len)) <= 0) {
      if(ret < 0) this.errno = syscall.errno;
      else if(ret == 0) this.close();
    }
    return ret;
  }

  write(buf, offset, len) {
    let ret;
    if((ret = send(this.fd, buf, offset, len)) == -1) this.errno = syscall.errno;
    return ret;
  }

  close() {
    syscall.close(this.fd);
    this.destroyed = true;
  }

  valueOf() {
    return this.fd;
  }
}

Object.assign(Socket.prototype, {
  family: AF_INET,
  connecting: false,
  destroyed: false,
  pending: false,
  remote: null,
  local: null
});
