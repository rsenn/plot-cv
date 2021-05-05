/* Worker code for test_worker.js */
import * as std from 'std';
import * as os from 'os';
import { Console } from 'console';
import * as cv from 'cv';

var parent = os.Worker.parent;

function WorkerMain() {
  new Console({
    colors: true,
    compact: 1,
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
  console.log('HandleMessage', ev);

  switch (ev.type) {
    case 'abort':
      parent.postMessage({ type: 'done' });
      break;
    case 'sab':
      /* modify the SharedArrayBuffer */
      ev.buf[2] = 10;
      ev.buf[3] = 9;
      ev.buf[4] = 8;
      ev.buf[5] = 7;
      ev.buf[6] = 6;
      parent.postMessage({ type: 'sab_done', buf: ev.buf });
      break;
  }
}
