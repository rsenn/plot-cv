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
  worker.onmessage = WorkerMessage;
  console.log('TestWorker', worker.onmessage);

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
              console.log("DEBUGGER", body);
             send(id, body);
           }
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
          console.log('MESSAGE', message);
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
