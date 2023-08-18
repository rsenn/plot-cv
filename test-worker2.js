import { setReadHandler, Worker } from 'os';
import { Console } from 'console';
/* os.Worker API test */
let worker;

function TestWorker() {
  globalThis.console = new Console({
    colors: true,
    compact: 1,
    prefix: '\x1b[38;5;220mPARENT\x1b[0m'
  });

  worker = new Worker(scriptArgs[1] ?? './workertest.js');

  worker.onmessage = e => console.log('worker.onmessage', e);

  setReadHandler(0, () => {
    let source = process.stdin.getline();

    worker.postMessage({ source });
  });
}

try {
  TestWorker();
} catch(err) {
  console.log('ERROR: ' + err.message + '\n' + err.stack);
}