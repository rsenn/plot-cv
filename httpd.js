import { Console } from 'console';
import { btoa } from './lib/misc.js';
import * as os from 'os';
import * as fs from 'fs';
import * as util from './lib/misc.js';
import Util from './lib/util.js';
import * as path from './lib/path.js';
import * as deep from './lib/deep.js';
import { Socket, IPPROTO_TCP } from './quickjs/qjs-ffi/lib/socket.js';
import { toString as ArrayBufferToString, toArrayBuffer as StringToArrayBuffer } from './lib/misc.js';

var worker;
var counter;

function main(script, ...args) {
  globalThis.console = new Console({
    colors: true,
    compact: 1,
    prefix: '\x1b[38;5;220mPARENT\x1b[0m'
  });

  let params = Util.getOpt({ host: [true, null, 'h'], port: [true, null, 'p'] }, args);

  console.log('params', params);

  worker = new os.Worker('./ws-worker.js');

  counter = 0;
  worker.onmessage = WorkerMessage;
  console.log('TestWorker', worker.onmessage);

  console.log('worker.postMessage', worker.postMessage);

  send('httpd', params);

  while(1) {
    os.sleep(500);
  }
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

    case 'done': {
      /* terminate */
      // worker.onmessage = null;
      break;
    }
  }
}

function send(type, body) {
  worker.postMessage({ type, ...body });
}

main(...scriptArgs);
