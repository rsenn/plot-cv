import * as std from 'std';
import * as os from 'os';
import { O_NONBLOCK, F_GETFL, F_SETFL, fcntl } from './fcntl.js';
import { errno } from 'ffi';
import { Socket, WaitRead,socket, EAGAIN, AF_INET, SOCK_STREAM, ndelay, connect, sockaddr_in, select, fd_set, timeval, FD_SET, FD_CLR, FD_ISSET, FD_ZERO, send, recv } from './socket.js';
import Util from './lib/util.js';
import { Console } from 'console';
import { toString as ArrayBufferToString, toArrayBuffer as StringToArrayBuffer } from 'misc';

Util.define(Array.prototype, {
  contains(item) {
    return this.indexOf(item) != -1;
  }
});
      const cfg = (obj = {}) => console.config({ compact: false, breakLength: Infinity, ...obj })

async function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      colors: true,
      depth: Infinity,
      maxArrayLength: 100,
      compact: 2,
      customInspect: true
    }
  });
  console.log('console.options', console.options);
  let params = Util.getOpt({
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
     FD_SET((debug ? debug.sock : sock).fd , rfds);

      if(debug) 
         FD_SET(0, rfds);
    

      const timeout = new timeval(5, 0);

    //  console.log('select(1)', cfg(), { rfds: rfds.array, wfds: wfds.array });

      ret = select(null, rfds, wfds, null, timeout);
      let readable = rfds.array, writable = wfds.array;
    
     // console.log('select(2)', cfg(), {readable,writable });

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

class DebuggerProtocol {
  constructor(sock) {
    Util.define(this, { sock });
   
    console.log('constructor:', +sock);

    this.seq = 0;
    this.requests = new Map();
  }

  readCommand(connection) {
    let line;

    if((line = std.in.getline())) {
      // console.log('Command:', line);

      this.sendRequest(line);
    }
  }

  handleResponse(message) {
    const {  type, request_seq, ...response } = message;

    console.log(`handleResponse #${request_seq}`,cfg(), response);
  }

  handleBreakpoints(message) {
    const {/*breakpoints,*/ request_seq, ...response } = message;
    console.log(`handleBreakpoints #${request_seq}`,cfg(), message);

    this.breakpoints = this.breakpoints;
  }

  handleMessage(message) {
    switch (message.type) {
      case 'event':
        this.handleEvent(message.event);
        break;

      case 'breakpoints':
        this.handleBreakpoints(message);
        break;
      case 'response':
        this.handleResponse(message);
        break;
      default: throw new Error(`Unknown message type: ${message.type}`);
        break;
    }
  }

  handleEvent(event) {
    console.log('handleEvent',
      console.config({ compact: false, breakLength: Infinity }),
      event
    );
    switch (event.type) {
      case 'StoppedEvent': {
        if(event.reason == 'entry') {
          this.sendMessage('breakpoints', {
            breakpoints: { path: 'test-ecmascript2.js', breakpoints: [{ line: 47, column: 3 }] }
          });
          this.sendMessage('breakpoints', {
            path: 'test-ecmascript2.js'
          });
          this.sendRequest('stepIn');
//          this.sendRequest('continue');
        } else {
          this.sendRequest('stackTrace');
          this.sendRequest('variables', { args: { variablesReference: 1 } });
          //this.sendRequest('continue');
          this.sendRequest('step');
        }

        // Util.waitFor(10000).then(() => sendRequest(connection, 'pause'));
        break;
      }
    }
  }

  sendMessage(type, args = {}) {
    const msg = { type, /*request_seq: args.request?.request_seq ?? args.request_seq ?? this.getSeq(),*/  ...args };

    console.log(`sendMessage #${msg.request_seq}`, cfg({ hideKeys: [ 'request_seq' ] }), {type,...args});
    try {
    const json = JSON.stringify(msg);
    return this.sock.puts(`${toHex(json.length, 8)}\n${json}`);
  } catch(error) {
    console.log("sendMessage", error.message/*, { msg }*/);
  }
  }

  getSeq() {
      return ++this.seq;
  }

  sendRequest(command, args = {}) {
    const request_seq = 1 + this.getSeq();
    const request = { command, request_seq, ...args };
    const message = { request,request_seq };

     this.requests.set(request_seq, message);

    return this.sendMessage('request', message);
  }

   read() {
    let lengthBuf = new ArrayBuffer(9);
    let jsonBuf;

 
    let r =this.sock.read(lengthBuf);
    if(r <= 0) {
      if(r < 0 && errno() != EAGAIN) throw new Error(`read error ${std.strerror(errno())}`);
    } else {
      let len = ArrayBufferToString(lengthBuf);
      let size = parseInt(len, 16);
      jsonBuf = new ArrayBuffer(size);
      r = this.sock.read(jsonBuf);
            //console.log(`read`, { r, size });
      if(r <= 0) {
        if(r < 0 && errno() != EAGAIN) throw new Error(`read error ${strerror(errno())}`);
      } else {
        let json = ArrayBufferToString(jsonBuf.slice(0, r));
        try {
          let message = JSON.parse(json);
      this.handleMessage(message);
        } catch(e) {
          console.log('ERROR', e.message, '\nDATA\n', json, '\nSTACK\n', e.stack);
          throw e;
        }
      }
    }
    //console.log('read',  r < 0 ? std.strerror(errno()) : r);
    return r;
  }
  async readHandler() {
    let it = this.sock[Symbol.asyncIterator]();
    for(;;) {
      let result = await it.next(9);

      if(result.done) {
        if(result.error) throw result.error;
        break;
      }
      let { value: data } = result;
      let size = parseInt(data, 16);

      result = await it.next(size);
      if(result.done) {
        if(result.error) throw result.error;
        break;
      }
      let json = result.value;
      let message;

      try {
        message = JSON.parse(json);
        this.handleMessage(message);
      } catch(e) {
        console.log('ERROR', e.message, '\nDATA\n', json, '\nSTACK\n', e.stack);
        throw e;
      }
    }
  }
}

function retValue(ret, ...args) {
  console.log(...args,
    `ret =`,
    ret,
    ...(ret == -1 ? [' errno =', errno(), ' error =', std.strerror(errno())] : [])
  );
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
