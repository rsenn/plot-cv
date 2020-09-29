import Stream from 'stream';
import Timers from './lib/repeater/timers.js';
import { spawn } from 'child_process';

export function execStream(cmd, args, options = { stdio: 'pipe' }) {
  var AB = new Stream.Duplex({ ...options, readableHighWaterMark: 1, writableHighWaterMark: 1 });
  var A = new Stream.PassThrough();
  var B = new Stream.PassThrough();
  // console.log = function() {}
  AB._write = function(chunk, encoding, cb) {
    return A.write(chunk, encoding, cb);
  };
  AB.on('finish', function() {
    A.end();
    A.emit('end');
  });
  AB._read = function(n) {};
  B.on('readable', function() {
    AB.push(B.read());
  });
  B.on('end', function() {
    AB.end();
    AB.emit('end');
  });
  var proc = spawn(cmd, args, options);
  A.pipe(proc.stdin);
  proc.stdout.pipe(B);
  return AB;
}

export function exec(cmd, args = [], options = { stdio: 'pipe' }) {
  let child = spawn(cmd, args, options);
  const { stdin, stdout, stderr } = child;

  child.stdout.on('data', data => {
    console.log(`stdout: ${data}`);
  });

  child.stderr.on('data', data => {
    console.error(`stderr: ${data}`);
  });

  child.on('close', code => {
    console.log(`child process exited with code ${code}`);
  });
  return [stdin, stdout, stderr];
}
