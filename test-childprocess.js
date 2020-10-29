import Util from './lib/util.js';
import PortableChildProcess, { SIGTERM, SIGKILL, SIGSTOP, SIGCONT } from './lib/childProcess.js';
import ConsoleSetup from './lib/consoleSetup.js';
import { Repeater } from './lib/repeater/repeater.js';

let childProcess;

function FdReader(fd, bufferSize = 1024) {
  let buf = filesystem.buffer(bufferSize);
  return new Repeater(async (push, stop) => {
    let ret;
    do {
      await filesystem.waitRead(fd);
      ret = filesystem.read(fd, buf);
      if(ret > 0) await push(filesystem.bufferToString(buf.slice(0, ret)));
    } while(ret == bufferSize);
    stop();
    filesystem.close(fd);
  });
}

async function main(...args) {
  await ConsoleSetup({ colors: true, depth: Infinity });
  await PortableChildProcess(p => (childProcess = p));

  let proc = childProcess('ls', ['-la'], { block: false, stdio: ['pipe', 'pipe', 'pipe'] });

  console.log('proc:', proc);
  proc.kill(SIGSTOP);
  proc.kill(SIGCONT);
  let w = proc.wait();
  console.log('proc.wait():', w);
  console.log('proc.wait():', await w);
  console.log('childProcess.errno:', childProcess.errno);
  console.log('childProcess.errstr:', childProcess.errstr);

  let output = '';
  for await (let data of FdReader(proc.stdout)) {
    output += data;
    console.log('output:', output.length);
  }
  console.log('output:', output);
}

Util.callMain(main, true);
