import { Console } from 'console';
import { btoa } from 'misc';
import * as os from 'os';
import * as fs from 'fs';
import * as util from 'util';
import child_process from './lib/childProcess.js';
import { DebuggerProtocol } from './debuggerprotocol.js';
import { Socket, IPPROTO_TCP } from './socket.js';

function StartDebugger(args, connect, address) {
  let env = {};
  address ??= '127.0.0.1:9901';
  if(connect) env['QUICKJS_DEBUG_ADDRESS'] = address;
  else env['QUICKJS_DEBUG_LISTEN_ADDRESS'] = address;
  let child = child_process.spawn('qjsm', ['qjsm', ...args], {
    env,
    stdio: ['inherit', 'pipe', 'pipe']
  });
  console.log('child(1)', child);

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

  console.log('child(2)', child);

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
  worker.onmessage = HandleMessage;
  console.log('TestWorker', worker.onmessage);

  /* while(1){
    os.sleep(500);
  }*/
}

function HandleMessage(e) {
  console.log('HandleMessage', e);
  var ev = e.data;
  switch (ev.type) {
    case 'start': {
    console.log('START', ev.start);
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
    case 'sab_done':
      {
        let buf = ev.buf;
        /* check that the SharedArrayBuffer was modified */
        util.assert(buf[2], 10);
        worker.postMessage({ type: 'abort' });
      break;
    }
      case 'done':
     { /* terminate */
      // worker.onmessage = null;
      break;
    }
  }
}

TestWorker();
