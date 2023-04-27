import { Console } from 'console';
import { btoa, toString } from './lib/misc.js';
import * as os from 'os';
//import child_process from './lib/childProcess.js';
import { AsyncSocket, SockAddr, AF_INET, SOCK_STREAM, IPPROTO_TCP } from './quickjs/qjs-ffi/lib/socket.js';
import { assert, define, toString as ArrayBufferToString, toArrayBuffer as StringToArrayBuffer } from './lib/misc.js';
import { DebuggerProtocol } from './debuggerprotocol.js';

var worker;
var counter;
let sockets = (globalThis.sockets ??= new Set());

export function StartDebugger(args, connect, address) {
  let env = process.env ?? {};
  address ??= '127.0.0.1:9901';
  env['DISPLAY'] ??= ':0.0';
  if(connect) env['QUICKJS_DEBUG_ADDRESS'] = address;
  else env['QUICKJS_DEBUG_LISTEN_ADDRESS'] = address;

  console.log('StartDebugger', env);

  /*  let child = child_process.spawn('qjsm', args, {
    env,
    stdio: ['inherit', 'pipe', 'pipe']
  });*/
  let pipe = os.pipe();
  let pid = os.exec(['qjsm'].concat(args), { block: false, env, stdout: pipe[1], stderr: pipe[1] });

  os.close(pipe[1]);
  return { stdio: [undefined, pipe[0], pipe[0]], pid };
  //const { pid, stdio } = child;

  console.log('StartDebugger', child);
  return child;
}

export function ConnectDebugger(address, callback) {
  const addr = new SockAddr(AF_INET, ...address.split(':'));
  const sock = new AsyncSocket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  const ret = sock.connect(addr);

  if(typeof callback != 'function' && typeof callback == 'object') callback = callback.onMessage;
  console.log('ConnectDebugger', { ret, callback });

  if(ret >= 0) {
    sock.ndelay(true);
    console.log('Connected', +sock, 'to', sock.remote);
    sockets.add(sock);
    console.log('sockets', sockets);
  }

  let iter = async function* () {
    let ret,
      lenBuf = new ArrayBuffer(9);

    while((ret = await sock.recv(lenBuf, 0, 9)) > 0) {
      let len = parseInt(ArrayBufferToString(lenBuf, 0, ret), 16);
      let dataBuf = new ArrayBuffer(len);
      let offset = 0;
      while(offset < len) {
        if((ret = await sock.recv(dataBuf, offset, len - offset)) <= 0) {
          sock.close();
          return;
        }
        offset += ret;
      }
      let obj = JSON.parse(ArrayBufferToString(dataBuf));
      yield obj;
    }
  };

  if(callback) {
    sock.onmessage = callback;

    (async (s, it) => {
      console.log('\x1b[1;35mprocessing messages\x1b[0m', { s, it });
      for await(let message of it) s.onmessage(message);
      console.log('\x1b[1;36mprocessing end!\x1b[0m', strerror(error().errno));
    })(sock, iter());
  } else {
    define(sock, { [Symbol.asyncIterator]: iter });
  }

  define(sock, { sendMessage: msg => sock.send(msg.length.toString(16).padStart(8, '0') + '\n' + msg) });

  // DebuggerProtocol.send(sock, typeof msg == 'string' ? msg : JSON.stringify(msg)) });

  console.log('ConnectDebugger', { ret, sock });

  return sock;
}

export class DebuggerDispatcher {
  #seq = 0;
  #responses = {};

  constructor(sock) {
    const orig = sock.onmessage;

    console.log('DebuggerDispatcher', { orig });

    sock.onmessage = msg => {
      const { request_seq, body } = msg;

      if(request_seq in this.#responses) this.#responses[request_seq](msg);
      else orig.call(sock, msg);
    };

    this.sendMessage = msg => sock.sendMessage(msg);
  }

  stepIn() {
    return this.sendRequest('stepIn');
  }

  stepOut() {
    return this.sendRequest('stepOut');
  }

  next() {
    return this.sendRequest('next');
  }

  continue() {
    return this.sendRequest('continue');
  }

  pause() {
    return this.sendRequest('pause');
  }

  evaluate(frameId, expression) {
    return this.sendRequest('evaluate', { frameId, expression });
  }

  variables(variablesReference, options = {}) {
    return this.sendRequest('variables', { variablesReference, ...options });
  }

  scopes(frameId) {
    return this.sendRequest('scopes', { frameId });
  }

  async stackTrace() {
    let { body } = await this.sendRequest('stackTrace');
    return body;
  }

  sendRequest(command, args = {}) {
    const request_seq = ++this.#seq;

    this.sendMessage(JSON.stringify({ type: 'request', request: { request_seq, command, args } }));

    return new Promise(
      (resolve, reject) =>
        (this.#responses[request_seq] = response => {
          delete this.#responses[request_seq];
          resolve(response);
        })
    );
  }
}

function TestWorker() {
  globalThis.console = new Console({
    colors: true,
    compact: 1,
    prefix: '\x1b[38;5;220mPARENT\x1b[0m'
  });
  console.log('scriptArgs', scriptArgs);
  worker = new os.Worker('./ws-worker.js');
  counter = 0;
  worker.onmessage = WorkerMessage;
  console.log('TestWorker', worker.onmessage);
  os.setReadHandler(0, () => {
    let line = process.stdin.getline();
    worker.postMessage({ line });
  });
}

let sock, connection;
function WorkerMessage(e) {
  console.log('WorkerMessage', e);
  var ev = e.data;
  const { message, id } = ev;
  switch (ev.type) {
    case 'message': {
      switch (message.type) {
        case 'start': {
          console.log('START', message.start);
          const { args, connect, address } = message.start;
          let child = StartDebugger(args, connect, address);
          os.sleep(1000);
          sock = ConnectDebugger(address);
          break;
        }
        default: {
          console.log('From WORKER', ev);
          connection.sendMessage(message);
          break;
        }
      }
      break;
    }
    case 'num': {
      assert(ev.num, counter);
      counter++;
      if(counter == 10) {
        let sab = new SharedArrayBuffer(10);
        let buf = new Uint8Array(sab);
        worker.postMessage({ type: 'sab', buf: buf });
        counter = 0;
      }
      break;
    }
    case 'sab_done': {
      let buf = ev.buf;
      assert(buf[2], 10);
      worker.postMessage({ type: 'abort' });
      break;
    }
    case 'done': {
      break;
    }
  }
}

function send(id, body) {
  worker.postMessage({ type: 'send', id, body });
}
/*TestWorker();*/
