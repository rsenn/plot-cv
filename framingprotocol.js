import { AsyncSocket } from 'sockets';
import { TextDecoder } from 'textcode';
import { define, wrapFunction } from 'util';
import extendFunction from 'extendFunction';

extendFunction();

export async function connect(host = '127.0.0.1', port = 9999) {
  const sock = new AsyncSocket(AF_INET, SOCK_STREAM, IPPROTO_IP);

  define(sock, {
    readBuf: [
      async (call, [n, ab]) => {
        ab ??= new ArrayBuffer(n);
        return (await call(ab, n)) ? ab : null;
      },
      async (call, [n, ab]) => {
        if((ab = await call(n, ab))) return new TextDecoder().decode(ab.slice(0, n));
      },
      async call => await call(+('0x' + (await call(9)))),
      async call => JSON.parse(await call()),
    ].reduce(wrapFunction, readFully(sock.recv)),
  });

  await sock.connect(host, port);
  return sock;
}

export function readFully(fn, thisObj) {
  return async function(buf, length) {
    let offset = 0;

    while(offset < length) {
      let received = await fn.call(thisObj ?? this, buf, offset, length - offset);
      if(received <= 0) return 0;
      offset += received;
    }

    return 1;
  };
}
