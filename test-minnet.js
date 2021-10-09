import { client, server, fetch, setLog } from 'net';
import { concat, escape, toString, toArrayBuffer } from 'util';
import Util from './lib/util.js';
import { Console } from 'console';

function CreateServer() {
  print('SERVER');
  server({
    port: 3300,
    mounts: [['/', '.', 'index.html']],
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
    }
  });
}

function CreateClient() {
  print('CLIENT');

  setLog(() => {});

  return client({
    port: 22,
    host: '127.0.0.1',
    raw: true,
    binary: true,
    onMessage(ws, msg) {
      console.log('onMessage', ws, msg);
      const b = concat(toArrayBuffer('BLAH'), msg.slice(0, 30), toArrayBuffer('\n'));
      console.log('b', b);
      let ret = ws.send(b); //ws.send('XXX\n');

      console.log('ret=ws.send(msg)', ret, toString(b));
    },
    onConnect(ws, req) {
      console.log('onConnect', ws, req);
    },
    onPong(ws, req) {
      console.log('onPong', ws, req);
    },
    onClose(ws, req) {
      console.log('onClose', ws, req);
    },
    onFd(fd, rd, wr) {
      console.log('onFd', fd, rd, wr);
      os.setReadHandler(fd, rd);
      os.setWriteHandler(fd, wr);
    }
  });
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
  globalThis.console = new Console({ inspectOptions: { compact: 2, customInspect: true } });
  let ws;
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
