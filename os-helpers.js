import { spawn } from 'child_process';
import * as fs from 'fs';
import { close, exec, pipe, waitpid, Worker } from 'os';
import { btoa, define, properties, setInterval, waitFor } from 'util';
import { Repeater } from './lib/repeater/repeater.js';
import { fdopen, popen } from 'std';
export { WNOHANG } from 'child_process';

export function ReadClipboard() {
  const f = popen('xclip -out 2>/dev/null', 'r');
  const r = f.readAsString();
  f.close();
  return r;
}

export function WriteClipboard(s) {
  const f = popen('xclip -in 2>/dev/null', 'w');
  f.puts(s);
  f.flush();
  f.close();
}

export function MonitorClipboard(cb, interval = 50) {
  let s = ReadClipboard();

  if(!cb)
    return (async function* () {
      for(;;) {
        await waitFor(interval);
        const tmp = ReadClipboard();
        if(tmp != s) yield (s = tmp);
      }
    })();

  setInterval(() => {
    const tmp = ReadClipboard();
    if(tmp != s) cb((s = tmp));
  }, interval);
}

export function Execute(...args) {
  let [rd, stdout] = pipe();
  let pid = exec(args, {
    block: false,
    stdout,
    stderr: stdout,
  });
  close(stdout);

  let [ret, status] = waitpid(pid, 1);

  let out = fs.readAllSync(rd);
  fs.closeSync(rd);

  if(ret != pid) [ret, status] = waitpid(pid, 0);

  return [(status & 0xff00) / 256, out];
}

export function URLWorker(script) {
  const dataURL = s => `data:application/javascript;charset=utf-8;base64,` + btoa(s).replaceAll('+', '-').replaceAll('/', '_').replaceAll('=', '');

  const url = dataURL(script);
  const w = new Worker(url);

  return define(new Repeater((push, stop) => (w.onmessage = push)), {
    postMessage: msg => w.postMessage(msg),
  });
}

export function Spawn(...args) {
  const child = spawn(...args);

  define(
    child,
    properties(
      {
        stdin() {
          return this.stdio[0] >= 0 ? fdopen(this.stdio[0], 'w') : null;
        },
        stdout() {
          return this.stdio[1] >= 0 ? fdopen(this.stdio[1], 'r') : null;
        },
        stderr() {
          return this.stdio[2] >= 0 ? fdopen(this.stdio[2], 'r') : null;
        },
      },
      { memoize: true },
    ),
  );

  return child;
}

export function Shell(cmd) {
  let f = popen(cmd, 'r');
  let s = '';

  while(!f.eof() && !f.error()) s += f.readAsString();

  f.close();
  return s;
}

export function ExecTool(cmd, ...args) {
  let f = popen([cmd, ...args].join(' '), 'r');
  let s = '';

  for(;;) {
    let line = f.getline();

    if(line === null) break;
    s += line + '\n';
  }

  f.close();
  return s;
}
