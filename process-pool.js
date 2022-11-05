import * as std from 'std';
import * as os from 'os';
import * as fs from 'fs';
import { setInterval, toString } from 'util';

export class ProcessPool {
  #processes = new Map();
  #timer = null;

  constructor() {
    let i = 0;
    this.#timer = setInterval(() => {
      let ret, status;

      do {
        [ret, status] = os.waitpid(-1, os.WNOHANG);
        if(ret > 0) {
          let [resolve, child] = this.#processes.get(ret);
          this.#processes.delete(ret);

          child.exitcode = (status & 0xff00) / 256;

          // console.log('timer', { ret, status: status.toString(16), child });

          resolve(child);
        }
      } while(ret > 0);
      // console.log('timer', i++);
    }, 100);
  }

  start(...args) {
    let [rd, wr] = os.pipe();

    let child = { stdout: rd, stderr: rd, output: '' };
    child.pid = os.exec(args, { block: false, stdout: wr, stderr: wr });
    fs.closeSync(wr);

    os.setReadHandler(rd, () => {
      let ret,
        buf = new ArrayBuffer(1024);

      ret = fs.readSync(rd, buf, 0, buf.byteLength);

      if(ret > 0) {
        child.output += toString(buf.slice(0, ret));
        //console.log('output', child.output.length);
      } else {
        fs.closeSync(rd);
      }
    });
    let entry = [, child];
    this.#processes.set(child.pid, entry);
    return new Promise((resolve, reject) => {
      entry[0] = resolve;
    });
    return child;
  }
}
