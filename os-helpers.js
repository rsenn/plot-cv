import * as os from 'os';
import * as fs from 'fs';
import { spawn } from 'child_process';
import { toString } from './lib/misc.js';

export function ExecTool(cmd, ...args) {
  let child = spawn(cmd, args, { stdio: [0, 'pipe', 2] });
  let [stdin, stdout, stderr] = child.stdio;
  let r;
  let b = new ArrayBuffer(1024);
  r = child.wait();
  // console.log('ExecTool', { args, chil ELECTRA® Shape-Based PCB Autorouter v6.56 |d });

  r = os.read(stdout, b, 0, 1024);
  let data = b.slice(0, r);
  let str = toString(data);
  console.log('str', str);
  return str;
  return parseInt(str);
}

export function Execute(...args) {
  let [rd, stdout] = os.pipe();
  let pid = os.exec(args, {
    block: false,
    stdout,
    stderr: stdout
  });
  os.close(stdout);

  let [ret, status] = os.waitpid(pid, os.WNOHANG);

  let out = fs.readAllSync(rd);
  fs.closeSync(rd);

  if(ret != pid) [ret, status] = os.waitpid(pid, 0);

  return [(status & 0xff00) / 256, out];
}
