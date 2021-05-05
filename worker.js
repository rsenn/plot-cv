/* Worker code for test_worker.js */
import * as std from 'std';
import * as os from 'os';
import { Console } from 'console';
import * as cv from 'cv';

var parent = os.Worker.parent;

function WorkerMain() {
  globalThis.console = new Console({
    colors: true,
    stringBreakNewline: true,
    maxStringLength: Infinity,
    compact: 4,
    prefix: '\x1b[38;5;128mWORKER\x1b[0m'
  });

  console.log('WorkerMain', parent);

  var i;

  parent.onmessage = HandleMessage;
  for(i = 0; i < 10; i++) {
    parent.postMessage({ type: 'num', num: i });
  }
}

WorkerMain();

function HandleMessage(e) {
  var ev = e.data;
  console.log('HandleMessage', e);

  switch (ev.type) {
    case 'abort':
      parent.postMessage({ type: 'done' });
      break;
    case 'sab':
      /* modify the SharedArrayBuffer */
      ev.buf[2] = 10;
      parent.postMessage({ type: 'sab_done', buf: ev.buf });
      break;
  }
}

