import filesystem from 'fs';
import * as std from 'std';
import * as os from 'os';
import * as deep from './lib/deep.js';
import * as path from './lib/path.js';
import { toArrayBuffer, toString, quote, escape } from './lib/misc.js';
import { Console } from 'console';
import inspect from './lib/objectInspect.js';
import * as fs from './lib/filesystem.js';
import * as net from 'net';
import { Socket, SockAddr, AF_INET, SOCK_STREAM, IPPROTO_TCP } from './quickjs/qjs-ffi/lib/socket.js';
import { define } from './lib/misc.js';

globalThis.fs = fs;

async function main(...args) {
  globalThis.console = new Console({
    inspectOptions: { compact: 2, customInspect: true }
  });

  let sock = new Socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  sock.ndelay(true);

  let addr = new SockAddr(AF_INET, '127.0.0.1', 22);

  let ret = sock.connect(addr);
  console.log('connect() =', ret, sock.errno);

  //await sock.waitWrite();
  /*ret = await sock.send('TEST\n');

 console.log('connected',ret,sock);*/

  //await sock.waitRead();
  let buf = new ArrayBuffer(1024);
  ret = await sock.recv(buf);
  console.log('sock.recv() =', ret);
  console.log('buf =', escape(toString(buf, 0, ret)));

  sock.close();
}

main().catch(err => console.log('error:', err.message, err.stack));
