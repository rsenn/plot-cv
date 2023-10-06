import * as os from 'os';
import { Console } from 'console';
var worker;
var counter;

function main(script, ...args) {
  globalThis.console = new Console({
    colors: true,
    compact: 1,
    prefix: '\x1b[38;5;220mPARENT\x1b[0m'
  });

  let params = getOpt({ host: [true, null, 'h'], port: [true, null, 'p'] }, args);

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