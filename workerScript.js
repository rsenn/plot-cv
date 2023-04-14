import { ReadFile, WriteFile } from './io-helpers.js';
import { Worker } from 'os';
import * as fs from 'fs';

export class WorkerScript {
  #worker = null;
  #cleanup = null;

  constructor(script) {
    let file = 'tmp-worker.js';

    if(!(WriteFile(file, script) > 0)) throw new Error(`Error writing '${file}'`);

    let worker = (this.#worker = new Worker(file));
    this.#cleanup = () => {
      fs.unlinkSync(file);
      this.#cleanup = null;
      worker.onmessage = worker.onmessage.fn;
    };
  }

  set onmessage(fn) {
    const worker = this.#worker;
    if(this.#cleanup) {
      worker.onmessage = (...args) => {
        this.#cleanup();
        return fn(...args);
      };
      worker.onmessage.fn = fn;
    } else {
      worker.onmessage = fn;
    }
  }

  get onmessage() {
    const onmessage = this.#worker.onmessage;
    return onmessage?.fn ? onmessage.fn : onmessage;
  }

  postMessage(...args) {
    return this.#worker.postMessage(...args);
  }
}
