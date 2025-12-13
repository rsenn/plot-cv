import { Worker } from 'os';
import { Console } from 'console';
import { toPointer } from 'util';
import process from 'process';

const log = (...args) => console.log('\x1b[38;5;45mOpenCV \x1b[38;5;129mWORKER\x1b[0m ', ...args);

/* Worker code for test_worker.js */
var parent = Worker.parent;

function handle_msg(e) {
  var ev = e.data;
  log('handle_msg', e);

  switch (ev.type) {
    case 'abort':
      parent.postMessage({ type: 'done' });
      break;
    case 'sab':

const u8arr = ev.buf;
const sab = u8arr.buffer;
  log(`SAB address:`, toPointer(sab));

      /* modify the SharedArrayBuffer */
      ev.buf[2] = 10;
      parent.postMessage({ type: 'sab_done', buf: ev.buf });
      break;
  }
}

function worker_main() {
  var i;
 
  globalThis.console = new Console(process.stderr, { inspectOptions: { colors: true } });

  log(`Worker started`);

  parent.onmessage = handle_msg;

  for(i = 0; i < 10; i++) {
    const msg = { type: 'num', num: i };

    log(`Posting message:`, msg);

    parent.postMessage(msg);
  }

  log('done');
}

worker_main();