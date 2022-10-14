import * as os from 'os';
import * as fs from 'fs';
import { spawn } from 'child_process';
//import { mmap, munmap, mprotect, PROT_READ, PROT_WRITE, MAP_PRIVATE, msync, MS_SYNC } from 'mmap';
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
