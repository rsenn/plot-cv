/* os.Worker API test */
import * as std from 'std';
import * as os from 'os';
import { Console } from 'console';
import { assert } from './lib/misc.js';

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
  console.log(
    'worker',
    Object.getOwnPropertyNames(Object.getPrototypeOf(worker)).reduce((acc, n) => ({ ...acc, [n]: worker[n] }), {})
  );
  //console.log('TestWorker', worker.onmessage);

  os.setReadHandler(0, () => {
    let line = process.stdin.getline();

    worker.postMessage({ line });
  });
}

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

TestWorker();
