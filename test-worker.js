/* os.Worker API test */
import * as std from 'std';
import * as os from 'os';
import { Console } from 'console';
import { assert } from 'util';

var worker;
var counter;

function test_worker() {
  new Console({
    colors: true,
    showHidden: false,
    showProxy: false,
    stringBreakNewline: true,
    maxStringLength: Infinity,
    compact: 2,
    numberBase: 16,
    prefix: '\x1b[38;5;220mPARENT\x1b[0m'
  });

  console.log('test_worker');
  worker = new os.Worker('./worker.js');

  counter = 0;
  worker.onmessage = HandleMessage;
}

function HandleMessage(e) {
  var ev = e.data;
  console.log('HandleMessage', e);
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

test_worker();
