import { Console } from 'console';
import { btoa } from './lib/misc.js';
import * as os from 'os';
import * as fs from 'fs';
import * as util from 'util';
import * as path from './lib/path.js';
import * as deep from './lib/deep.js';
import child_process from './lib/childProcess.js';
import { DebuggerProtocol } from './debuggerprotocol.js';
import { Socket, IPPROTO_TCP } from './quickjs/qjs-ffi/lib/socket.js';
import { toString as ArrayBufferToString, toArrayBuffer as StringToArrayBuffer } from './lib/misc.js';

console.log('toString', ArrayBufferToString(new Uint8Array([0x61, 0x62, 0x64, 0x65, 0x66, 0x20, 0xc3, 0xa4, 0xc3, 0xb6, 0xc3, 0xbc]).buffer));
console.log('toArrayBuffer', StringToArrayBuffer('blah äöü'));

function StartDebugger(args, connect, address) {
  let env = {};
  address ??= '127.0.0.1:9901';
  if(connect) env['QUICKJS_DEBUG_ADDRESS'] = address;
  else env['QUICKJS_DEBUG_LISTEN_ADDRESS'] = address;
  let child = child_process.spawn('qjsm', ['qjsm', ...args], {
    env,
    stdio: ['inherit', 'pipe', 'pipe']
  });

  child.stdio.slice(1).forEach((fd, i) => {
    const out = ['stdout', 'stderr'][i];
    child[out] = '';
    fs.onRead(fd, () => {
      let buf = fs.buffer(1024);
      let r = fs.readSync(fd, buf);
      if(r === 0) fs.onRead(fd, null);
      else if(r > 0) child[out] += fs.bufferToString(buf.slice(0, r));
      console.log(`child[${out}]`, child[out].replace(/\n/g, '\\n'));
    });
  });

  console.log('StartDebugger', child);

  return child;
}

var worker;
var counter;

function TestWorker() {
  globalThis.console = new Console({
    colors: true,
    compact: 1,
    prefix: '\x1b[38;5;220mPARENT\x1b[0m'
  });

  worker = new os.Worker('./ws-worker.js');

  counter = 0;
  worker.onmessage = WorkerMessage;
  console.log('TestWorker', worker.onmessage);

  os.setReadHandler(0, () => {
    let line = process.stdin.getline();

    worker.postMessage({ line });
  });
  /* while(1){
    os.sleep(500);
  }*/
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
          let fd = +(sock = new Socket(IPPROTO_TCP));

          fs.onWrite(fd, () => {
            //console.log('writeable', fd);
            fs.onWrite(fd, null);
            connection = new DebuggerProtocol(sock);
            connection.onmessage = body => {
              const json = JSON.parse(body);
              console.log('To WORKER', json);

              console.log('deep.select', deep.select + '');

              for(let [n, p] of deep.select(json, (n, k) => n.filename)) {
                if(n.filename) n.filename = n.filename.replace(process.cwd() + '/', './');
              }

              send(id, json);
            };
            fs.onRead(fd, () => {
              if(connection) {
                let r = connection.read();
                // console.log('readable', {fd, r});;

                if(r == 0) fs.onRead(fd, null);
              }
            });
          });

          let ret = sock.connect('127.0.0.1', 9901);
          /*console.log('sock', sock);
          console.log('ret:', ret);
          console.log('child(3)', child);*/

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
        /* test SharedArrayBuffer modification */
        let sab = new SharedArrayBuffer(10);
        let buf = new Uint8Array(sab);
        worker.postMessage({ type: 'sab', buf: buf });
        counter = 0;
      }
      break;
    }
    case 'sab_done': {
      let buf = ev.buf;
      /* check that the SharedArrayBuffer was modified */
      util.assert(buf[2], 10);
      worker.postMessage({ type: 'abort' });
      break;
    }
    case 'done': {
      /* terminate */
      // worker.onmessage = null;
      break;
    }
  }
}

function send(id, body) {
  worker.postMessage({ type: 'send', id, body });
}
TestWorker();
