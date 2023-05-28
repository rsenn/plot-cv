import * as std from 'std';
import * as os from 'os';
import * as deep from 'deep';
import * as path from 'path';
import { define, toArrayBuffer, toString, quote, escape } from 'util';
import { Console } from 'console';
import inspect from 'inspect';
import * as fs from 'fs';
import { Socket, AsyncSocket, SockAddr, AF_INET, SOCK_STREAM, IPPROTO_TCP } from 'sockets';

globalThis.fs = fs;

async function main(...args) {
  globalThis.console = new Console({
    inspectOptions: { compact: 2, customInspect: true }
  });

  let sock = new AsyncSocket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
   console.log('new Socket() =', sock);
   console.log('sock.ndelay:', sock.ndelay);
 sock.ndelay(true);

  let addr = new SockAddr(AF_INET, '192.168.178.23', 22);

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
