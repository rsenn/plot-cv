import { client, server, fetch } from 'net';
import * as std from 'std';
import * as os from 'os';
import { Console } from 'console';

var parent = os.Worker.parent;

const log = (...args) => console.log('WORKER', ...args);

function WorkerMain() {
  globalThis.console = new Console({
    colors: true,
    compact: 1,
    prefix: '\x1b[38;5;128mWORKER\x1b[0m'
  });

  log('WorkerMain.parent', parent);
  parent.onmessage = HandleMessage;
  os.sleep(500);
  CreateServer();
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

function CreateServer(port = 9900) {
  print(`Listening on http://127.0.0.1:${port}`);
  server({
    port,
    mounts: [['/', '.', 'debugger.html']],
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

      log(`Server.onClose client#${client.id} (${ws.fd})` + (why ? ' Reason: ' + why : ''));
    },
    onPong(ws, data) {
      let client = WSClient.get(ws.fd);
      log(`Server.onPong client#${client.id} (${ws.fd})` + (data ? ' Data: ' + data : ''));
    }
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
    case 'abort': {
      parent.postMessage({ type: 'done' });
      break;
    }
    case 'sab':
      /* modify the SharedArrayBuffer */
      ev.buf[2] = 10;
      ev.buf[3] = 9;
      ev.buf[4] = 8;
      ev.buf[5] = 7;
      ev.buf[6] = 6;
      parent.postMessage({ type: 'sab_done', buf: ev.buf });
      break;
  }
}

try {
  WorkerMain();
} catch(error) {
  log(`FAIL: ${error?.message}\n${error.stack}`);
  std.exit(1);
} finally {
  log('SUCCESS');
}
