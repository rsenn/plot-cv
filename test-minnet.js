import { client, createServer, fetch, LLL_ALL, LLL_USER, setLog } from 'net';
import * as os from 'os';
import { quote, toString } from './lib/misc.js';
import { Console } from 'console';
setLog(LLL_USER, (level, message) => console.log('LWS', message));

const console = new Console({ inspectOptions: { compact: 0, customInspect: true } });

const print = (...args) => console.log(...args);

function CreateServer() {
  print('SERVER');

  setLog(LLL_ALL, (level, ...args) =>
    console.log((level ?? ['ERR', 'WARN', 'NOTICE', 'INFO', 'DEBUG', 'PARSER', 'HEADER', 'EXT', 'CLIENT', 'LATENCY', 'MINNET', 'THREAD'][Math.log2(level)] ?? level + '').padEnd(8), ...args)
  );

  createServer({
    port: 3300,
    mounts: [
      ['/', '.', 'index.html'],
      async function* test(req, resp) {
        console.log('*test', { req, resp });
        yield 'test\r\n';
      }
    ],
    onConnect: socket => {
      print('Client connected');
      print('Socket: ' + socket);
      socket.send('Hello from server');
    },
    onMessage: (socket, msg) => {
      print('Received: ', msg);
    },
    onClose: why => {
      print('Client disconnected. Reason: ', why);
    },
    onPong: (socket, data) => {
      print('Pong: ', data);
    },
    onFd(fd, rd, wr) {
      os.setReadHandler(fd, rd);
      os.setWriteHandler(fd, wr);
    }
  });
}

function CreateClient() {
  print('CLIENT');
  // setLog(() => {});
  setLog((level, ...args) => (level > 256 ? console.log('WSI', ...args) : null));

  let url;
  url = 'https://127.0.0.1:9000/debugger-client.js';
  url = 'ws://127.0.0.1:9000/';
  let cl;

  cl = client(url, {
    sslCA: 'warmcat.com.cer',
    onMessage(ws, msg) {
      /*    console.log('onMessage', ws, escape(msg.slice(0, 30)));*/
      console.log('data:', quote(msg, "'"));
    },
    onConnect(ws, req) {
      console.log('onConnect', ws, req);

      os.setReadHandler(0, () => {
        let rbuf = new ArrayBuffer(1024);
        let ret = os.read(0, rbuf, 0, 1024);
        console.log('os.read() =', ret);
        if(ret === 0) {
          os.setReadHandler(0, null);
          return;
        }
        let data = toString(rbuf, 0, ret);
        console.log('Read:', data.trimRight());
        ws.send(data);
      });
      let n = 0;
      os.signal(os.SIGINT, () => {
        if(n++ < 1) console.log('(Press Ctrl-C again to quit)');
        else ws.close();
      });
    },
    onPong(ws, req) {
      console.log('onPong', ws, req);
    },
    onClose(ws, req) {
      console.log('onClose', ws, req);
      std.exit(0);
    },
    onFd(fd, rd, wr) {
      console.log('onFd', fd, rd, wr);
      os.setReadHandler(fd, rd);
      os.setWriteHandler(fd, wr);
    }
  });
  return cl;
}

function getJSON() {
  console.log('getJSON');
  const res = fetch('https://api.github.com/repos/rsenn/plot-cv', {
    method: 'head'
  });
  const { ok, status, type } = res;
  console.log('res:', { ok, status, type });

  const json = res.json();
  console.log('json:', json);

  const data = new Map(Object.entries(json));
  console.log('data:', data);
  return data;
}

function main(...args) {
  let ws;

  args[0] ??= 's';

  switch (args[0]) {
    case 's':
      ws = CreateServer();
      break;
    case 'c':
      ws = CreateClient();
      break;
    case 'f':
      ws = getJSON();
      break;
  }
  console.log('ws', ws);
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error?.message ?? error}\n${error?.stack}`);
  1;
  std.exit(1);
} finally {
  //console.log('SUCCESS');
}