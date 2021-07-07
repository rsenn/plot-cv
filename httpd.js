import { Console } from 'console';
import { btoa } from './lib/misc.js';
import * as os from 'os';
import * as fs from 'fs';
import * as util from 'util';
import * as path from './lib/path.js';
import * as deep from './lib/deep.js';
 import { Socket, IPPROTO_TCP } from './socket.js';
import {
  toString as ArrayBufferToString,
  toArrayBuffer as StringToArrayBuffer
} from './lib/misc.js';
 
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

  console.log('worker.sendMessage', worker.sendMessage);

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

function send(id, body) {
  worker.postMessage({ type: 'send', id, body });
}

TestWorker();
