import * as std from 'std';
import * as os from 'os';
import * as deep from './lib/deep.js';
import { errno } from 'ffi';
import { EAGAIN } from './socket.js';
import Util from './lib/util.js';
import { toString as ArrayBufferToString, toArrayBuffer as StringToArrayBuffer } from 'misc';

const cfg = (obj = {}) => console.config({ compact: false, breakLength: Infinity, ...obj });

export class DebuggerProtocol {
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
    const { type, request_seq, ...response } = message;
    console.log(`handleResponse #${request_seq}`, /*cfg(),*/ response);
  }

  handleBreakpoints(message) {
    const { /*breakpoints,*/ request_seq, ...response } = message;
    console.log(`handleBreakpoints #${request_seq}`, cfg(), message);
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
    console.log('handleEvent', console.config({ compact: false, breakLength: Infinity }), event);
    switch (event.type) {
      case 'StoppedEvent': {
        if(event.reason == 'entry') {
          this.sendMessage('stopOnException', { stopOnException: true });
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
    const msg = {
      type,
      /*request_seq: args.request?.request_seq ?? args.request_seq ?? this.getSeq(),*/ ...args
    };
    //for(let [value, key] of deep.iterate({ type, args }, v => typeof v != 'object')) console.log(`${key}=${value}`);
    //console.log(`sendMessage #${msg.request_seq}`, cfg({ hideKeys: ['request_seq'] }), {type, ...args });
    try {
      const json = JSON.stringify(msg);
      return this.sock.puts(`${toHex(json.length, 8)}\n${json}`);
    } catch(error) {
      console.log('sendMessage', error.message /*, { msg }*/);
    }
  }

  getSeq() {
    return ++this.seq;
  }

  sendRequest(command, args = {}) {
    const request_seq = 1 + this.getSeq();
    const request = { command, request_seq, ...args };
    const message = { request, request_seq };
    this.requests.set(request_seq, message);
    return this.sendMessage('request', message);
  }

  read() {
    let lengthBuf = new ArrayBuffer(9);
    let jsonBuf;

    let r = this.sock.read(lengthBuf);
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

export default DebuggerProtocol;
