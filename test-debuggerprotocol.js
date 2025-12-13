import * as os from 'os';
import { DebuggerProtocol } from './debuggerprotocol.js';
import { define } from './lib/misc.js';
import { FD_CLR, fd_set, FD_SET } from './quickjs/qjs-ffi/lib/fd_set.js';
import timeval from './quickjs/qjs-ffi/lib/timeval.js';
import { Console } from 'console';
import { errno } from 'ffi';
import * as std from 'std';
import { Socket, AF_INET, SOCK_STREAM, SockAddr, select } from './quickjs/qjs-ffi/lib/socket.js';

define(Array.prototype, {
  contains(item) {
    return this.indexOf(item) != -1;
  },
});

async function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      depth: Infinity,
      maxArrayLength: 100,
      breakLength: 10000,
      compact: 3,
      customInspect: true,
    },
  });
  console.log('console.options', console.options);
  let params = getOpt(
    {
      listen: [false, null, 'l'],
      debug: [false, null, 'x'],
      address: [true, null, 'c'],
      port: [true, null, 'p'],
      '@': 'address,port',
    },
    args,
  );
  const { listen } = params;

  const [address = '127.0.0.1', port = 9000] = args;

  let sock = new Socket(AF_INET, SOCK_STREAM);
  let addr = new SockAddr(AF_INET, address, port);

  console.log('socket() fd =', +sock);

  let ret;
  let debug;

  if(listen) {
    ret = sock.bind(addr);
    retValue(ret, `sock.bind(${addr})`);
    ret = sock.listen();
    retValue(ret, `sock.listen())`);
  } else {
    //  sock.ndelay(true);
    ret = sock.connect(addr);
    retValue(ret, `sock.connect(${addr})`);
  }

  debug = new DebuggerProtocol(sock);
  console.log('debug', debug);

  os.setReadHandler(+sock, () => {
    //console.log('debug.read', debug.read);
    debug.read();
    if(sock.eof) os.setReadHandler(+sock, null);
  });

  os.setReadHandler(0, () => debug.sendRequest(std.in.getline()));

  /*  ret = sendRequest(+sock, 'next');
  retValue(ret);*/

  //  IOLoop();
  //
  if(sock.errno) {
    console.log(`error connecting to ${addr}:`, sock.error.message);
    std.exit(1);
  }

  console.log('debuggerprotocol', sock);

  function IOLoop() {
    const rfds = new fd_set();
    const wfds = new fd_set();
    console.log('IOLoop', sock);
    FD_SET(+sock, wfds);
    do {
      FD_SET((debug ? debug.sock : sock).fd, rfds);
      if(debug) FD_SET(0, rfds);
      const timeout = new timeval(5, 0);
      ret = select(null, rfds, wfds, null, timeout);
      let readable = rfds.toArray(),
        writable = wfds.toArray();
      console.log('select(2)', { readable, writable });
      if(writable.indexOf(sock.fd) != -1) {
        if(!debug) debug = new DebuggerProtocol(sock);
        FD_CLR(sock.fd, wfds);
      }

      if(readable.indexOf(sock.fd) != -1) {
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

      if(readable.contains(0)) debug.sendRequest(std.in.getline());
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

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error && error.message}\n${error && error.stack}`);
  std.exit(1);
} finally {
  //console.log('SUCCESS');
}