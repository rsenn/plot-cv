import { WNOHANG, Worker, close, exec, pipe, read, waitpid } from 'os';
import * as fs from 'fs';
import { Spawn } from './io-helpers.js';
import { define, toString, btoa } from './lib/misc.js';
import { RepeaterOverflowError, FixedBuffer, SlidingBuffer, DroppingBuffer, MAX_QUEUE_LENGTH, Repeater } from './lib/repeater/repeater.js';

export function ExecTool(cmd, ...args) {
  let child = Spawn(cmd, args, { stdio: [0, 'pipe', 2] });
  let [stdin, stdout, stderr] = child.stdio;
  let r;
  let b = new ArrayBuffer(1024);
  r = child.wait();
  // console.log('ExecTool', { args, chil ELECTRA® Shape-Based PCB Autorouter v6.56 |d });

  r = read(stdout, b, 0, 1024);
  let data = b.slice(0, r);
  let str = toString(data);
  console.log('str', str);
  return str;
  return parseInt(str);
}

export function Execute(...args) {
  let [rd, stdout] = pipe();
  let pid = exec(args, {
    block: false,
    stdout,
    stderr: stdout
  });
  close(stdout);

  let [ret, status] = waitpid(pid, WNOHANG);

  let out = fs.readAllSync(rd);
  fs.closeSync(rd);

  if(ret != pid) [ret, status] = waitpid(pid, 0);

  return [(status & 0xff00) / 256, out];
}

export function URLWorker(script) {
  const dataURL = s => `data:application/javascript;charset=utf-8;base64,` + btoa(s).replaceAll('+', '-').replaceAll('/', '_').replaceAll('=', '');

  const url = dataURL(script);
  const w = new Worker(url);

  return define(new Repeater((push, stop) => (w.onmessage = push)), { postMessage: msg => w.postMessage(msg) });
}
