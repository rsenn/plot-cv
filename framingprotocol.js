import { AsyncSocket } from 'sockets';
import { TextDecoder } from 'textcode';
import { readFileSync } from 'fs';
import { getOrCreate, ucfirst, extend, define, properties, nonenumerable, declare, assign, wrapFunction } from 'util';
import extendFunction from 'extendFunction';

extendFunction();

const sourceFiles = getOrCreate(new Map(), file => '\n' + readFileSync(file, 'utf-8').split('\n'));

export async function connect(host = '127.0.0.1', port = 9999) {
  const sock = new AsyncSocket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  const lenBuf = new ArrayBuffer(9);

  let id,
    obj = Object.setPrototypeOf({}, Object.create(null, { [Symbol.toStringTag]: { value: 'FramingProtocol' } }));

  const encode = (
    (e = new TextEncoder()) =>
    s =>
      e.encode(s)?.buffer
  )();

  const requests = {};

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
      async (request, [command, args]) => {
        const seq = await request(command, args);

        return new Promise(resolve => {
          requests[seq] = resolve;
        });
      },
    ].reduce(wrapFunction, obj.sendMessage),
  });

  extend(obj, {
    continue: () => obj.sendMessage({ type: 'continue' }),
    breakpoints: (path, breakpoints) =>
      obj.sendMessage({ type: 'breakpoints', ...(breakpoints ? { breakpoints: { path, breakpoints } } : { path }) }),
    stopOnException: flag => obj.sendMessage({ type: 'stopOnException', stopOnException: !!flag }),
  });

  extend(obj, {
    pause: () => obj.sendRequest('pause'),
    next: () => obj.sendRequest('next'),
    stepIn: () => obj.sendRequest('stepIn'),
    stepOut: () => obj.sendRequest('stepOut'),
    evaluate: expression => obj.sendRequest('evaluate', { expression }),
    stackTrace: () => obj.sendRequest('stackTrace'),
    scopes: frameId => obj.sendRequest('scopes', { frameId }),
    variables: (variablesReference, filter, start, count) =>
      obj.sendRequest('variables', { variablesReference, filter }),
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

  extend(obj, {
    async processResponses() {
      const processors = {
        response: ({ request_seq, body }) => {
          if(request_seq in requests) {
            requests[request_seq](setType(body ?? {}, 'ResponseBody'));
            delete requests[request_seq];
            return true;
          }
        },
        event: async ({ event: { type, ...event } }) => {
          setType(event, type);

          if(obj['on' + type]) if (await obj['on' + type](event)) return true;

          console.log(event);
          return true;
        },
      };

      for await(const response of obj) {
        const { type, ...rest } = response;
        const resp = setType(rest, type);

        if(!(type in processors) || !(await processors[type](resp))) console.log(response.type, resp);
      }
    },
    XonStoppedEvent: async event => {
      console.log('onStoppedEvent', { event });
      const st = await obj.stackTrace();

      console.log('onStoppedEvent', { st });

      const [top] = st;
      const { id, name, filename, line, column } = top;

      const s = sourceFiles(filename)[line];

      const prefix = filename + ':' + line + ': ';

      console.log(prefix + s);
      console.log(' '.repeat(prefix.length) + ' ' + ' '.repeat(column) + '^');

      return true;
    },
  });

  await sock.connect(host, port);
  return obj;
}

function setType(obj, type) {
  return Object.setPrototypeOf(obj, { [Symbol.toStringTag]: ucfirst(type) });
}
