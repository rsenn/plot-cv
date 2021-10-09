import { Console } from 'console';
import { btoa } from './lib/misc.js';
import * as os from 'os';
import * as fs from 'fs';
import * as util from 'util';
import * as path from './lib/path.js';
import * as deep from './lib/deep.js';
import { toString } from './lib/misc.js';
import child_process from './lib/childProcess.js';
import { Socket, SockAddr, AF_INET, SOCK_STREAM, IPPROTO_TCP } from './quickjs/qjs-ffi/lib/socket.js';
import { toString as ArrayBufferToString, toArrayBuffer as StringToArrayBuffer } from './lib/misc.js';
import { DebuggerProtocol } from './debuggerprotocol.js';

console.log(
  'toString',
  ArrayBufferToString(new Uint8Array([0x61, 0x62, 0x64, 0x65, 0x66, 0x20, 0xc3, 0xa4, 0xc3, 0xb6, 0xc3, 0xbc]).buffer)
);
console.log('toArrayBuffer', StringToArrayBuffer('blah äöü'));
console.log('child_process', child_process.spawn + '');

var worker;
var counter;
let sockets = (globalThis.sockets ??= new Set());

export function StartDebugger(args, connect, address) {
  let env = {};
  address ??= '127.0.0.1:9901';
  if(connect) env['QUICKJS_DEBUG_ADDRESS'] = address;
  else env['QUICKJS_DEBUG_LISTEN_ADDRESS'] = address;

  let child = child_process.spawn('qjsm', args, {
    env,
    stdio: ['inherit', 'inherit', 'inherit']
  });

  console.log('StartDebugger', child.pid, child.args, child.env);

  return child;
}

export function ConnectDebugger(address, callback) {
  let addr = new SockAddr(AF_INET, ...address.split(':'));
  console.log('ConnectDebugger', addr);
  let sock = new Socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  let ret = sock.connect(addr);

  if(ret >= 0) {
    sock.ndelay(true);
    console.log('Connected', +sock, 'to', sock.remote);
    sockets.add(sock);
    console.log('sockets', sockets);
  }

  /*  os.setWriteHandler(+sock, () => {
    os.setWriteHandler(+sock, null);
    let dbg = new DebuggerProtocol(sock);
  
    os.setReadHandler(+sock, () => {
      if(dbg) {
        let r = dbg.read();
      console.log('readable', { fd: +sock, r });
        if(r <= 0) {
          os.setReadHandler(+sock, null);
          console.log('read() =', r, sock.error, callback + '');
          callback(dbg, sock);
        }
      }
    });
  });
*/
  //  if(ret < 0) throw new Error(`Connection failed: ${sock.error}`);
  console.log('ConnectDebugger', sock);

  return sock;
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
      util.assert(ev.num, counter);
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
      util.assert(buf[2], 10);
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
