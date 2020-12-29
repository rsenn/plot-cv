import * as std from 'std';
import * as os from 'os';
import { O_NONBLOCK, F_GETFL, F_SETFL, fcntl } from './fcntl.js';
import { Socket, socket, AF_INET, SOCK_STREAM, ndelay, connect, sockaddr_in, select, fd_set, timeval, FD_SET, FD_ISSET, getError, Error, strerror, send, recv } from './socket.js';
import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';

async function main(...args) {
  await ConsoleSetup({
    breakLength: 120,
    maxStringLength: 200,
    maxArrayLength: 20,
    multiline: 1,
    alignMap: true
  });

  const listen = !!(args[0] == '-l' && args.shift());

  const [addr = '127.0.0.1', port = 9000] = args;

  let sock = new Socket();
  console.log('socket() fd =', +sock);

  let ret;

  if(listen) {
    ret = sock.bind(addr, port);
    retValue(ret, `sock.bind(${addr}, ${port})`);
    ret = sock.listen(addr, port);
    retValue(ret, `sock.listen())`);
  } else {
    ret = sock.connect(addr, port);

    retValue(ret, `sock.connect(${addr}, ${port})`);
  }

  console.log('Error:', Error);

  /*  ret = sendRequest(+sock, 'next');
  retValue(ret);*/

  const buf = new ArrayBuffer(1024);
  const rfds = new fd_set();

  do {
    FD_SET(+sock, rfds);

    ret = select(null, rfds, null, null, null);

    if(FD_ISSET(+sock, rfds)) {
      if(listen) {
        ret = sock.accept();

        retValue(ret, 'sock.accept()');
      } else {
        let data = ArrayBufToString(sock.read(9));

        let length = parseInt(ArrayBufToString(data), 16);
        console.log('length:', length);
        data = sock.read(length);
        let message = JSON.parse(ArrayBufToString(data));
        console.log('message:', message);
      }
    }
  } while(!sock.destroyed);
  console.log('end');
}

function sendMessage(fd, type, args = {}) {
  const msg = { type, ...args };

  console.log('sendMessage', msg);
  const json = JSON.stringify(msg);
  return send(fd, `${toHex(json.length, 8)}\n${json}`);
}

function sendRequest(fd, command, args = {}) {
  const request = { command, ...args };
  return sendMessage(fd, 'request', request);
}

function retValue(ret, ...args) {
  console.log(...args,
    `ret =`,
    ret,
    ...(ret == -1 ? [' errno =', getError(), ' error =', strerror(getError())] : [])
  );
}

function toHex(n, b = 2) {
  let s = (+n).toString(16);
  return '0'.repeat(Math.ceil(s.length / b) * b - s.length) + s;
}

function ArrayBufToString(buf, offset, length) {
  if(typeof buf == 'object' && buf != null && buf instanceof ArrayBuffer) {
    let arr = new Uint8Array(buf, offset, length);
    return arr.reduce((s, code) => s + String.fromCodePoint(code), '');
  }
  return buf;
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
  if(typeof buf == 'object' && buf != null && buf instanceof ArrayBuffer) {
    let arr = MakeArray(buf, numBytes);
    return arr.reduce((s, code) =>
        (s != '' ? s + ' ' : '') + ('000000000000000' + code.toString(16)).slice(-(numBytes * 2)),
      ''
    );
  }
  return buf;
}

Util.callMain(main, true);
