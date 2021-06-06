import { client, server, fetch } from 'net';
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
let sock, connection;

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      colors: true,
      depth: 8,
      breakLength: 100,
      maxStringLength: Infinity,
      maxArrayLength: 30,
      compact: 2,
      showHidden: false
    }
  });
  let port = 9900;
  for(let arg of args) {
    if(!isNaN(+arg)) arg = +arg;

    if(typeof arg == 'number' && arg >= 1024 && arg <= 65535) port = arg;
  }
  let i = 0;

  /*  os.setTimeout(() => {
    console.log(`interval #${i++}`);
  }, 500);*/

  let wsworker = new os.Worker('./ws-worker.js');

  wsworker.onmessage = HandleMessage;
}

main(...scriptArgs.slice(1));
function HandleMessage(e) {
  console.log('HandleMessage', e);
  var ev = e.data;
  switch (ev.type) {
    case 'num':
      assert(ev.num, counter);
      counter++;
      if(counter == 10) {
        /* test SharedArrayBuffer modification */
        let sab = new SharedArrayBuffer(10);
        let buf = new Uint8Array(sab);
        worker.postMessage({ type: 'sab', buf: buf });
        counter = 0;
      }
      break;
    case 'sab_done':
      {
        let buf = ev.buf;
        /* check that the SharedArrayBuffer was modified */
        assert(buf[2], 10);
        worker.postMessage({ type: 'abort' });
      }
      break;
    case 'done':
      /* terminate */
      worker.onmessage = null;
      break;
  }
}
