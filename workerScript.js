import { Worker } from 'os';
import * as fs from 'fs';
import * as std from 'std';
import { once } from 'util';

export class WorkerScript {
  #real = null;
  #removeFile = null;

  constructor(script) {
    let ret,
      scriptName = fs.tempnamSync() + '.js';
    //std.puts('scriptName: ' + scriptName + '\n');

    if(!(ret = fs.writeFileSync(scriptName, script)) > 0) throw new Error(`Error writing '${scriptName}'`);

    this.#real = new Worker(scriptName);
    this.#removeFile = once(() => {
      fs.unlinkSync(scriptName);
      this.#removeFile = null;
      this.#real.onmessage = this.#real.onmessage.fn;
    });
  }

  set onmessage(fn) {
    const worker = this.#real;
    if(this.#removeFile) {
      worker.onmessage = (...args) => fn(...args);
      worker.onmessage.fn = fn;
    } else {
      worker.onmessage = fn;
    }
  }

  postMessage(...args) {
    return this.#real.postMessage(...args);
  }
}
