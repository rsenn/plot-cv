import { Message } from './message.js';
import Util from './lib/util.js';
import { Alea } from './lib/alea.js';
import Stream, { Readable, Writable, PassThrough, Duplex } from 'stream';
import Timers, { TimeoutError } from './lib/repeater/timers.js';
import { spawn } from 'child_process';
import { AsyncWrite, AsyncRead, PipeToRepeater, LineReader, WritableRepeater, WriteIterator, ReadFromIterator } from './lib/stream/utils.js';
import { Repeater } from './lib/repeater/repeater.js';
import ConsoleSetup from './consoleSetup.js';

const prng = new Alea();

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
