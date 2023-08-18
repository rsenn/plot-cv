import child_process from 'child_process';
import { bufferToString, closeSync, readSync } from 'fs';
import { Repeater } from './lib/repeater/repeater.js';
let childProcess;

function waitRead(file) {
  if(typeof file == 'object' && file != null && 'once' in file) {
    file.setMaxListeners(100);
    file.resume();
    return new Promise((resolve, reject) => {
      let len;
      file.once('data', chunk => resolve(chunk));
      file.once('end', chunk => resolve(chunk));
    });
  } else {
    let fd = typeof file == 'number' ? file : file.fileno();
    return new Promise((resolve, reject) => {
      os.setReadHandler(fd, () => {
        os.setReadHandler(fd, null);
        resolve(file);
      });
    });
  }
}

function waitExit(proc) {
  if(typeof proc == 'object' && proc != null && 'once' in proc) {
    return new Promise((resolve, reject) => {
      proc.on('exit', resolve);
    });
  } else {
    return proc.wait();
  }
}

function FdReader(fd, bufferSize = 1024) {
  let buf = new ArrayBuffer(bufferSize);
  return new Repeater(async (push, stop) => {
    let ret;
    do {
      let r = await waitRead(fd);
      ret = typeof fd == 'number' ? readSync(fd, buf) : fd.read(buf);
      if(ret > 0) {
        let data = buf.slice(0, ret);
        await push(bufferToString(data));
      }
    } while(ret == bufferSize);
    stop();
    typeof fd == 'number' ? closeSync(fd) : fd.destroy();
  });
}

async function main(...args) {
  // await PortableChildProcess(p => (childProcess = p));

  let proc = child_process.spawn('ls', ['-la'], {
    block: false,
    stdio: ['pipe', 'pipe', 'pipe']
  });

  // console.log('proc:', proc);
  process.kill(proc.pid, 'SIGSTOP');
  process.kill(proc.pid, 'SIGCONT');
  //console.log('proc.stdout:', proc.stdout);
  console.log('proc:', proc);

  let output = '';
  for await(let data of FdReader(proc.stdio[1])) {
    console.log('data:', data);
    output += data;
    console.log('output.length:', output.length);
  }
  // console.log('output:', output);

  let w = await waitExit(proc);
  console.log('proc.wait():', w);
  console.log('proc.wait():', await w);
  console.log('childProcess.errno:', proc.errno);
  console.log('childProcess.errstr:', proc.errstr);
}

main().catch(err => console.log('error:', err.message, err.stack));