import { define, escape, toString } from './lib/misc.js';
import { relative } from './lib/path.js';

const cfg = (obj = {}) => console.config({ compact: false, breakLength: Infinity, ...obj });

export class DebuggerProtocol {
  constructor(sock) {
    define(this, { sock });

    this.seq = 0;
    this.requests = new Map();
    this.files = {};

    this.on('message', this.handleMessage);
  }

  readCommand() {
    let line;

    if((line = std.in.getline())) this.sendRequest(line);
  }

  handleResponse(message) {
    const { type, request_seq, ...response } = message;
    const { request } = this.requests.get(request_seq);

    this.requests.delete(request_seq);

    console.log(`handleResponse #${request_seq}`, request.command, response);

    switch (request.command) {
      case 'stackTrace': {
        for(let frame of response.body) {
          const { id, name, filename, line } = frame;
          let code,
            wd = process.cwd(),
            location = filename && filename[0] == '/' ? relative(filename) : filename;

          if(typeof line == 'number') {
            code = this.getFile(location)?.[line - 1];
            location += ':' + line;
          }

          console.log(`Stack Frame #${id}`, name.padEnd(20), location + (code ? `: ` + code : ''));
        }

        break;
      }
    }
  }

  handleBreakpoints(message) {
    const { breakpoints, request_seq, ...response } = message;

    console.log(`handleBreakpoints`, cfg(), message);

    this.breakpoints = breakpoints;
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

      default:
        throw new Error(`Unknown message type: ${message.type}`);
        break;
    }
  }

  handleEvent(event) {
    console.log('handleEvent', cfg(), event, this.sendMessage);

    const stepMode = 'next';

    switch (event.type) {
      case 'StoppedEvent': {
        if(event.reason == 'entry') {
          this.sendMessage('stopOnException', { stopOnException: true });
          this.sendMessage('breakpoints', { breakpoints: { path: 'test-ecmascript2.js', breakpoints: [{ line: 47, column: 3 }] } });
          this.sendMessage('breakpoints', { path: 'test-ecmascript2.js' });
          this.sendRequest(stepMode);
        } else {
          this.sendRequest('stackTrace');
          this.sendRequest('variables', { args: { variablesReference: 1 } });
          this.sendRequest(stepMode);
        }

        break;
      }
    }
  }

  sendMessage(type, args) {
    const msg = args ? { type, ...args } : type;

    console.log('sendMessage', msg);

    try {
      const json = JSON.stringify(msg);

      if(this.send) return this.send(json);

      return this.sock.send(`${toHex(json.length, 8)}\n${json}`);
    } catch(error) {
      console.log('sendMessage', error.message, error.stack);
    }
  }

  getSeq() {
    return (this.seq = (this.seq | 0) + 1);
  }

  sendRequest(command, args = {}) {
    const request = { command, request_seq: this.getSeq(), ...args };

    switch (command) {
      case 'variables': {
        if(!('args' in request)) request.args = {};
        if(!('variablesReference' in request.args)) request.args.variablesReference = 1;

        break;
      }
    }

    const message = { type: 'request', request };
    this.requests.set(request_seq, message);
    return this.sendMessage(message);
  }

  static async read(sock) {
    const lengthBuf = new ArrayBuffer(9);
    let r = await sock.recv(lengthBuf);

    if(r <= 0) {
      console.log('sock.error', sock.error);

      if(r < 0 && sock.errno != sock.EAGAIN) throw sock.error;

      return null;
    }

    let len = toString(lengthBuf);
    let size = parseInt(len, 16);

    let n = 0,
      jsonBuf = new ArrayBuffer(size);

    while(n < size) {
      r = await sock.recv(jsonBuf, n, size - n);

      if(r <= 0) {
        if(r < 0 && sock.errno != sock.EAGAIN) throw sock.error;

        return null;
      }

      n += r;
    }

    console.log('async DebuggerProtocol.read', { r, len, size });
    return toString(jsonBuf.slice(0, n));
  }

  static send(sock, msg) {
    console.log('DebuggerProtocol.send', { sock, msg });

    const data = toHex(msg.length, 8) + '\n' + msg;

    console.log('data', escape(data));

    return sock.send(data);
  }

  async read() {
    const data = await DebuggerProtocol.read(sock);

    if(data) this.emit('message', JSON.parse(data));

    return data;
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

      let message,
        json = result.value;

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
  console.log(...args, `ret =`, ret, ...(ret == -1 ? [' errno =', errno(), ' error =', std.strerror(errno())] : []));
}

function toHex(n, b = 2) {
  const s = (+n).toString(16);
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
  if(typeof buf == 'object' && buf != null && buf instanceof ArrayBuffer)
    return MakeArray(buf, numBytes).reduce((s, code) => (s != '' ? s + ' ' : '') + ('000000000000000' + code.toString(16)).slice(-(numBytes * 2)), '');

  return buf;
}

export default DebuggerProtocol;
