import { AsyncSocket } from 'sockets';
import { TextDecoder } from 'textcode';
import { extend, define, properties, nonenumerable, declare, assign, wrapFunction } from 'util';
import extendFunction from 'extendFunction';

extendFunction();

export async function connect(host = '127.0.0.1', port = 9999) {
  const sock = new AsyncSocket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  const lenBuf = new ArrayBuffer(9);

  let id,
    obj = Object.setPrototypeOf({}, { [Symbol.toStringTag]: 'FramingProtocol' });

  const encode = (
    (e = new TextEncoder()) =>
    s =>
      e.encode(s)?.buffer
  )();

  extend(obj, {
    read: [
      async (call, [n, ab = new ArrayBuffer(n)]) => (await call(ab, n)) && ab,
      async (call, [n, ab]) => (ab = await call(n, ab)) && new TextDecoder().decode(ab.slice(0, n)),
      async call => await call(+('0x' + (await call(9, lenBuf)))),
      async call => JSON.parse((await call()) || 'null'),
    ].reduce(wrapFunction, readFully),
  });

  extend(obj, {
    async *[Symbol.asyncIterator]() {
      let data;
      while((data = await this.read())) yield data;
    },
  });

  extend(obj, {
    sendMessage: [
      async (send, [b]) => (await send(b.byteLength.toString(16).padStart(8, '0') + '\n')) + (await send(b)),
      (send, [msg]) => send(encode(JSON.stringify(msg))),
    ].reduce(wrapFunction, (buf, length = buf.byteLength) => sock.send(buf, 0, length)),
  });

  extend(obj, {
    sendRequest: [
      (message, [request]) => message({ type: 'request', request }),
      async (request, [command, args], request_seq = (id = (id ?? 0) + 1)) =>
        (await request({ command, args, request_seq })) && request_seq,
    ].reduce(wrapFunction, obj.sendMessage),
  });

  async function readFully(buf, length) {
    let offset = 0;

    while(offset < length) {
      const received = await sock.recv(buf, offset, length - offset);
      if(received <= 0) return 0;
      offset += received;
    }

    return 1;
  }

  await sock.connect(host, port);
  return obj;
}
