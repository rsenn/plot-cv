import { readAllSync } from 'fs';
import { Worker } from 'os';
import { Spawn } from './os-helpers.js';
import { Console } from 'console';
import inspect from 'inspect';
import * as std from 'std';
globalThis.console = new Console(std.open('workertest.log', 'a+'), {
  inspectOptions: {
    compact: 2,
    customInspect: true,
    maxArrayLength: 200,
    prefix: '\x1b[38;5;220mWORKER\x1b[0m'
  }
});

const worker = Worker.parent;

console.log('worker started!');

worker.onmessage = msg => {
  console.log('worker.onmessage', inspect(msg));
  const {
    data: { source }
  } = msg;
  worker.postMessage(loadAST(source));
};

worker.postMessage({ message: 'worker started!' });

function loadAST(source) {
  const { pid, stdout, wait } = Spawn('meriyah', [source], {
    block: false,
    stdio: ['inherit', 'pipe', 'inherit']
  });
  console.log('loadAST', { source, pid, stdout });

  const data = readAllSync(stdout, 16384);
  return JSON.parse(data);
}