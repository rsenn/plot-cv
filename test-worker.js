/* os.Worker API test */
import * as std from 'std';
import * as os from 'os';
import { Console } from 'console';
import { assert } from './lib/misc.js';
import { WorkerScript } from './workerScript.js';

var worker;
var counter;

globalThis.console = new Console({
  colors: true,
  compact: 1,
  prefix: '\x1b[38;5;220mPARENT\x1b[0m'
});

function TestWorker() {
  //worker = new os.Worker('./ws-worker.js');
  worker = new WorkerScript(`
import { client, server, fetch, setLog, LLL_USER, LLL_NOTICE } from 'net';
import * as std from 'std';
import * as os from 'os';
import { Console } from 'console';
import { ProcessPool } from './process-pool.js';

globalThis.console = new Console({
  colors: true,
  compact: 2,
  prefix: '\x1b[38;5;220mCHILD\x1b[0m'
});

var parent = os.Worker?.parent;
let pool =new ProcessPool();

//const log = (...args) => console.log('WORKER', ...args);

console.log('parent',parent);

parent.onmessage = msg => {
  console.log('Message:',msg);
  const {data}=msg;
  switch(data.type) {
    case 'exec': 
    let child=pool.start(...data.args);
    child.then(r => {
      //console.log('child:',r);

      parent.postMessage({ type: 'wait', child: r});
    });

    break;
  }
}

parent.postMessage('test');
`);

  console.log('worker', inspect(worker));
  console.log('worker', Object.getOwnPropertyNames(Object.getPrototypeOf(Object.getPrototypeOf(worker))));
  console.log('worker', Object.getPrototypeOf(worker).constructor.name);
  counter = 0;
  worker.onmessage = HandleMessage;
  // console.log('worker', Object.getOwnPropertyNames(Object.getPrototypeOf(worker)).reduce((acc, n) => ({ ...acc, [n]: worker[n] }), {}));
  console.log('TestWorker', worker.onmessage);
  console.log('TestWorker', worker.postMessage);

  worker.postMessage({ type: 'exec', args: ['ls', '-la'] });

  os.setReadHandler(0, () => {
    let line = process.stdin.getline();

    worker.postMessage({ line });
  });

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
        // worker.onmessage = null;
        break;
    }
  }
}
TestWorker();
