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
      let r = await filesystem.waitRead(fd);
      ret = filesystem.read(fd, buf);
      if (ret > 0) {
        let data = buf.slice(0, ret);
        await push(filesystem.bufferToString(data));
      }
    } while (ret == bufferSize);
    stop();
    filesystem.close(fd);
  });
}

async function main(...args) {
  await ConsoleSetup({ colors: true, depth: Infinity });
  await PortableChildProcess((p) => (childProcess = p));

  let proc = childProcess('ls', ['-la'], { block: false, stdio: ['pipe', 'pipe', 'pipe'] });

  // console.log('proc:', proc);
  proc.kill(SIGSTOP);
  proc.kill(SIGCONT);
  console.log('proc.stdout:', proc.stdout);

  let output = '';
  for await (let data of FdReader(proc.stdout)) {
    console.log('data:', data);
    output += data;
    console.log('output.length:', output.length);
  }
  console.log('output:', output);

  let w = proc.wait();
  console.log('proc.wait():', w);
  console.log('proc.wait():', await w);
  console.log('childProcess.errno:', childProcess.errno);
  console.log('childProcess.errstr:', childProcess.errstr);
}

Util.callMain(main, true);
