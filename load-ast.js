import { existsSync, readerSync } from 'fs';
import { Worker } from 'os';
import { gettid, toString } from 'util';
import { Spawn } from './os-helpers.js';
import { Console } from 'console';

globalThis.console = new Console({
  inspectOptions: {
    compact: 2,
    customInspect: true,
    maxArrayLength: 200,
    prefix: '\x1b[2K\x1b[G\x1b[1;33mWORKER\x1b[0m ',
  },
});

const worker = Worker.parent;

worker.onmessage = async ({ data }) => {
  const { type, source } = data;

  switch (type) {
    case 'gettid': {
      worker.postMessage({ tid: gettid() });
      break;
    }
    case 'quit': {
      worker.onmessage = null;
      console.log('quitting thread (' + gettid() + ')...');
      break;
    }
    default: {
      const ast = await loadAST(source);
      worker.postMessage({ ast });
      break;
    }
  }
};

async function loadAST(source) {
  if(!existsSync(source)) return null;
  const { stdout, wait } = Spawn('meriyah', ['-l', source], {
    block: false,
    stdio: ['inherit', 'pipe', 'inherit'],
  });

  let s = '';
  for(let chunk of readerSync(stdout)) s += toString(chunk);

  const [pid, status] = wait();
  const { length } = s;
  //console.log('loadAST', { source, length, status });

  return JSON.parse(s);
}
