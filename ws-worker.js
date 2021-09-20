import { client, server, fetch, setLog } from 'net';
import * as std from 'std';
import * as os from 'os';
import { Console } from 'console';

var parent = os.Worker?.parent;

const log = (...args) => console.log('WORKER', ...args);

function WorkerMain() {
  globalThis.console = new Console({
    colors: true,
    compact: 1,
    prefix: '\x1b[38;5;128mWORKER\x1b[0m'
  });

  if(parent) {
    log(
      'WorkerMain.parent',
      Object.getOwnPropertyNames(Object.getPrototypeOf(parent)).reduce((acc, n) => ({ ...acc, [n]: parent[n] }), {})
    );

    parent.onmessage = e => HandleMessage.call(parent, e);
  }
  os.sleep(500);
  //CreateServer();
}

const clients = new Map();

class WSClient {
  static get(fd) {
    return clients.get(fd);
  }

  constructor(socket) {
    this.socket = socket;
    this.id = WSClient.id = (WSClient.id ?? 0) + 1;

    clients.set(socket.fd, this);
  }
}

function CreateServer({ host = '127.0.0.1', port = 9900, sslCert = '/home/roman/.acme.sh/senn.gotdns.ch/senn.gotdns.ch.cer', sslPrivateKey = '/home/roman/.acme.sh/senn.gotdns.ch/senn.gotdns.ch.key', index = 'index.html', ...options }) {
  print(`Listening on http://${host}:${port}`);
  if(sslCert) print(`SSL certificate file: ${sslCert}`);
  if(sslPrivateKey) print(`SSL certificate file: ${sslPrivateKey}`);
  server({
    host,
    port,
    sslCert,
    sslPrivateKey,
    mounts: [['/', '.', index ?? 'debugger.html']],
    onConnect(ws) {
      let client = new WSClient(ws);
      log(`Server.onConnect client#${client.id} (${ws.fd})`);
    },
    onMessage(ws, msg) {
      let client = WSClient.get(ws.fd);
      log(`Server.onMessage client#${client.id} (${ws.fd})`);

      if(typeof msg == 'string') {
        const message = { id: client.id, type: 'message', message: JSON.parse(msg) };
        log('Received', message);
        parent.postMessage(message);
        return;
      }
    },
    onClose(ws, why) {
      let client = WSClient.get(ws.fd);
      os.setReadHandler(fd, null);
      os.setWriteHandler(fd, null);
      log(`Server.onClose client#${client.id} (${ws.fd})` + (why ? ' Reason: ' + why : ''));
    },
    onPong(ws, data) {
      let client = WSClient.get(ws.fd);
      log(`Server.onPong client#${client.id} (${ws.fd})` + (data ? ' Data: ' + data : ''));
    },
    onHttp(ws, data) {
      log(`Server.onHttp ws`, ws, data ? ' Data: ' + data : '');
    },
    onBody(ws) {
      log(`Server.onBody ws`, ws);
    },
    onFd(fd, readable, writable) {
      os.setReadHandler(fd, readable);
      os.setWriteHandler(fd, writable);

      //log(`Server.onFd ${fd}`, readable && 1 | readable, writable && 1 | writable);
    },
    ...options
  });
}

function HandleMessage(e) {
  var ev = e.data;
  log('HandleMessage', ev);

  switch (ev.type) {
    case 'send': {
      const { id, body } = ev;
      let client;
      const json = typeof body != 'string' ? JSON.stringify(body) : body;

      if((client = [...clients.values()].find(cl => cl.id == id))) client.socket.send(json);

      break;
    }
    case 'httpd': {
      setLog((module, msg) => {
        if(/ERROR/.test(msg)) throw new Error(msg);
        console.log(`${module}.log: ${msg}`);
      });
      CreateServer(ev);

      setLog(null);
      this.postMessage({ type: 'done' });
      break;
    }
    case 'abort': {
      this.postMessage({ type: 'done' });
      break;
    }
    case 'sab':
      /* modify the SharedArrayBuffer */
      ev.buf[2] = 10;
      ev.buf[3] = 9;
      ev.buf[4] = 8;
      ev.buf[5] = 7;
      ev.buf[6] = 6;
      this.postMessage({ type: 'sab_done', buf: ev.buf });
      break;
  }
}

try {
  WorkerMain();
} catch(error) {
  console.log(`FAIL: ${error?.message}\n${error?.stack}`);
  std.exit(1);
} finally {
  console.log('SUCCESS');
}
