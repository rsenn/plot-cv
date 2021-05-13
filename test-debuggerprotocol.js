import { strerror, Error } from 'std';
import * as os from 'os';
import { O_NONBLOCK, F_GETFL, F_SETFL, fcntl } from './fcntl.js';
import { errno } from 'ffi';
import { Socket, socket, AF_INET, SOCK_STREAM, ndelay, connect, sockaddr_in, select, fd_set, timeval, FD_SET, FD_ISSET, FD_ZERO, send, recv } from './socket.js';
import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';

async function main(...args) {
  await ConsoleSetup({
    breakLength: 120,
    maxStringLength: 200,
    multiline: 1,
    alignMap: true
  });

  const listen = !!(args[0] == '-l' && args.shift());

  const [addr = '127.0.0.1', port = 9000] = args;

  let sock = new Socket();
  let connection;
  console.log('socket() fd =', +sock);

  let ret;

  if(listen) {
    ret = sock.bind(addr, port);
    retValue(ret, `sock.bind(${addr}, ${port})`);
    ret = sock.listen();
    retValue(ret, `sock.listen())`);
  } else {
    ret = sock.connect(addr, port);

    retValue(ret, `sock.connect(${addr}, ${port})`);
  }

  console.log('Error:', Error);

  await (async function IOHandler() {
    for await(let data of sock) {
      console.log('data:', data);
      if(data.length <= 9) continue;
      let [len, json] = [...Util.splitAt(data, 9)];
      let size = parseInt(len, 16);
      let message = JSON.parse(json);
      console.log('', { size, message });

      if(message.type == 'event') handleEvent(sock, message.event);
    }
  })();

  /*  ret = sendRequest(+sock, 'next');
  retValue(ret);*/

  console.log('debuggerprotocol', { sock, connection });

  function IOLoop() {
    const buf = new ArrayBuffer(1024);
    const rfds = new fd_set();
    const wfds = new fd_set();

    do {
      FD_ZERO(rfds);
      FD_ZERO(wfds);

      if(sock.connecting) FD_SET(+sock, wfds);
      else FD_SET(+sock, rfds);

      if(connection) {
        FD_SET(+connection, rfds);
        FD_SET(0, rfds);
      }

      const timeout = new timeval(5, 0);

      ret = select(null, rfds, null, null, timeout);

      console.log('select', { rfds, timeout });

      if(FD_ISSET(+sock, wfds)) {
        connection = sock;
        FD_SET(+sock, rfds);
      }
      if(FD_ISSET(+sock, rfds)) {
        if(listen) {
          connection = sock.accept();
          retValue(connection, 'sock.accept()');
        }
      }

      if(FD_ISSET(+connection, rfds)) {
        let data = ArrayBufToString(connection.read(9));
        let length = parseInt(ArrayBufToString(data), 16);
        if(length > 0) {
          console.log('length:', length);
          data = connection.read(length);
          let message = JSON.parse(ArrayBufToString(data));
          if(message.type == 'event') {
            handleEvent(connection, message.event);
          } else {
            console.log('message:', message);
          }
        } else {
          connection = undefined;
        }
      }

      if(FD_ISSET(0, rfds)) {
        readCommand(connection);
      }
    } while(!sock.destroyed);
    console.log('end');
  }
}

function readCommand(connection) {
  let line;

  while((line = std.in.getline())) {
    console.log('Command:', line);

    sendRequest(connection, line);
  }
}

function handleEvent(connection, event) {
  console.log('handleEvent', event);
  switch (event.type) {
    case 'StoppedEvent': {
      //      sendRequest(connection, 'stackTrace');
      sendRequest(connection, 'continue');

      Util.waitFor(10000).then(() => sendRequest(connection, 'pause'));
      break;
    }
  }
}

function sendMessage(sock, type, args = {}) {
  const msg = { type, ...args };

  console.log('sendMessage', msg, sock);
  const json = JSON.stringify(msg);
  return sock.puts(`${toHex(json.length, 8)}\n${json}`);
}

function getSeq(sock) {
  if(getSeq.numbers === undefined) getSeq.numbers = new Map();
  const numbers = getSeq.numbers;
  let num = numbers.get(sock);
  if(typeof num != 'number') num = 0;
  numbers.set(sock, ++num);
  return num;
}

function sendRequest(sock, command, args = {}) {
  const request = { command, seq: getSeq(sock), request: { command, ...args } };
  return sendMessage(sock, 'request', request);
}

function retValue(ret, ...args) {
  console.log(...args,
    `ret =`,
    ret,
    ...(ret == -1 ? [' errno =', errno(), ' error =', strerror(errno())] : [])
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
        (s != '' ? s + ' ' : '') +
        ('000000000000000' + code.toString(16)).slice(-(numBytes * 2)),
      ''
    );
  }
  return buf;
}

Util.callMain(main, true);
