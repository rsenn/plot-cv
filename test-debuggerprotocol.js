import * as std from 'std';
import * as os from 'os';
import * as deep from './lib/deep.js';
import { O_NONBLOCK, F_GETFL, F_SETFL, fcntl } from './fcntl.js';
import { errno } from 'ffi';
import { Socket, WaitRead, socket, EAGAIN, AF_INET, SOCK_STREAM, /*ndelay, */ connect, sockaddr_in, select, fd_set, timeval, FD_SET, FD_CLR, FD_ISSET, FD_ZERO, send, recv } from './quickjs/qjs-ffi/examples/socket.js';
import Util from './lib/util.js';
import { Console } from 'console';
import { toString as ArrayBufferToString, toArrayBuffer as StringToArrayBuffer } from './lib/misc.js';
import { DebuggerProtocol } from './debuggerprotocol.js';

Util.define(Array.prototype, {
  contains(item) {
    return this.indexOf(item) != -1;
  }
});

async function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      colors: true,
      depth: Infinity,
      maxArrayLength: 100,
      breakLength: 10000,
      compact: 2,
      customInspect: true
    }
  });
  console.log('console.options', console.options);
  let params = Util.getOpt(
    {
      listen: [false, null, 'l'],
      debug: [false, null, 'x'],
      address: [true, null, 'c'],
      port: [true, null, 'p'],
      '@': 'address,port'
    },
    args
  );
  const { listen } = params;

  const [address = '127.0.0.1', port = 9000] = args;

  let sock = new Socket();
  console.log('socket() fd =', +sock);

  let ret;
  let debug;

  if(listen) {
    ret = sock.bind(address, port);
    retValue(ret, `sock.bind(${address}, ${port})`);
    ret = sock.listen();
    retValue(ret, `sock.listen())`);
  } else {
    sock.ndelay(true);
    ret = sock.connect(address, port);
    retValue(ret, `sock.connect(${address}, ${port})`);
  }

  /*  ret = sendRequest(+sock, 'next');
  retValue(ret);*/

  IOLoop();

  console.log('debuggerprotocol');

  function IOLoop() {
    //const buf = new ArrayBuffer(1024);
    const rfds = new fd_set();
    const wfds = new fd_set();

    FD_SET(+sock, wfds);

    do {
      FD_SET((debug ? debug.sock : sock).fd, rfds);

      if(debug) FD_SET(0, rfds);

      const timeout = new timeval(5, 0);

      //  console.log('select(1)',  { rfds: rfds.array, wfds: wfds.array });

      ret = select(null, rfds, wfds, null, timeout);
      let readable = rfds.array,
        writable = wfds.array;

      // console.log('select(2)',   {readable,writable });

      if(writable.contains(sock.fd)) {
        if(!debug) debug = new DebuggerProtocol(sock);
        FD_CLR(sock.fd, wfds);
      }
      if(readable.contains(sock.fd)) {
        if(listen) {
          let connection = sock.accept();
          if(!debug) debug = new DebuggerProtocol(connection);
          retValue(connection, 'sock.accept()');
          ndelay(connection);
        }
      }

      if(debug && readable.contains((debug.sock ?? sock).fd)) {
        debug.read();
      } else if(!debug && readable.contains(sock.fd)) {
        debug = new DebuggerProtocol(sock);
      }

      if(readable.contains(0)) {
        debug.readCommand();
      }
    } while(!sock.destroyed);
    console.log('end');
  }
}

function retValue(ret, ...args) {
  console.log(...args, `ret =`, ret, ...(ret == -1 ? [' errno =', errno(), ' error =', std.strerror(errno())] : []));
}

function toHex(n, b = 2) {
  let s = (+n).toString(16);
  return '0'.repeat(Math.ceil(s.length / b) * b - s.length) + s;
}
function MakeArray(buf, numBytes) {
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
}

function ArrayBufToHex(buf, numBytes = 8) {
  if(typeof buf == 'object' && buf != null && buf instanceof ArrayBuffer) {
    let arr = MakeArray(buf, numBytes);
    return arr.reduce((s, code) => (s != '' ? s + ' ' : '') + ('000000000000000' + code.toString(16)).slice(-(numBytes * 2)), '');
  }
  return buf;
}
Util.callMain(main, true);
