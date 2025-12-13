import { Worker } from 'os';
import { Console } from 'console';
import { className, toPointer } from 'util';

const log = (...args) => console.log('\x1b[38;5;220mParent \x1b[38;5;34mTHREAD\x1b[0m ', ...args);

/* os.Worker API test */
function assert(actual, expected, message) {
  if(arguments.length == 1) expected = true;

  if(actual === expected) return;

  if(actual !== null && expected !== null && typeof actual == 'object' && typeof expected == 'object' && actual.toString() === expected.toString()) return;

  throw Error('assertion failed: got |' + actual + '|' + ', expected |' + expected + '|' + (message ? ' (' + message + ')' : ''));
}

var worker;

function TestOpenCVWorker() {
  var counter;

  globalThis.console = new Console({
    inspectOptions: { colors: true, depth: 2, maxArrayLength: Infinity, maxStringLength: 1024, customInspect: true, classKey: Symbol.toStringTag, hideKeys: [Symbol.toStringTag] }
  });

  worker = new Worker('./opencv-worker.js');

  counter = 0;
  log('worker', className(worker));

  worker.onmessage = function(e) {
    var ev = e.data;

    log('worker.onmessage', e);

    switch (ev.type) {
      case 'num':
        assert(ev.num, counter);
        counter++;
        if(counter == 10) {
          /* test SharedArrayBuffer modification */
          let sab = new SharedArrayBuffer(10);
          let buf = new Uint8Array(sab);

          log(`SAB address:`, toPointer(sab));

          worker.postMessage({ type: 'sab', buf: buf });
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
  };

  log('worker.onmessage', worker.onmessage);
}

try {
  TestOpenCVWorker();
} catch(error) {
  log(`FAIL: ${error?.message ?? error}\n${error?.stack}`);
} finally {
}