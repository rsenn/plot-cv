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

  var i;

  parent.onmessage = HandleMessage;

  for(i = 0; i < 10; i++) {
    parent.postMessage({ type: 'num', num: i });
  }
  os.sleep(1000);

  log('parent.postMessage', parent.postMessage);

  CreateServer();
}

try {
  WorkerMain();
} catch(error) {
  log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  log('SUCCESS');
}

class WSClient {
  static map = new Map();

  static get(socket) {
    return WSClient.map.get(socket.fd);
  }

  constructor(socket) {
    this.socket = socket;

    WSClient.map.set(socket.fd, this);
  }
}

function CreateServer(port = 9900) {
  print(`Listening on http://127.0.0.1:${port}`);
  server({
    port,
    mounts: [['/', '.', 'debugger.html']],
    onConnect: socket => {
      log(`Client connected (${socket.fd})`);
      new WSClient(socket);
    },
    onMessage: (socket, msg) => {
      //let client = WSClient.get(socket);

      log(`onMessage (${socket.fd})`, socket);

      if(typeof msg == 'string') {
        const json = JSON.parse(msg);
        log('Received', { json });
        parent.postMessage(json);
        return;

        if(json.type == 'start') {
          const { args, connect, address } = json.start;
          let child = StartDebugger(args, connect, address);

          /* log('child.wait()', child.wait());
          log('child(3).stderr', child.stderr);*/
          os.sleep(1000);
          sock = new Socket(IPPROTO_TCP);

          fs.onWrite(+sock, () => {
            log('writeable', +sock);
            fs.onWrite(+sock, null);
            connection = new DebuggerProtocol(sock);
            fs.onRead(+sock, () => {
              log('readable', +sock);
              if(connection) connection.read();
            });
          });

          ret = sock.connect('127.0.0.1', 9901);
          log('sock', sock);

          // sock.ndelay(true);

          log('ret:', ret);

          log('child(3)', child);
        } else if(sock) {
          sock.send(msg);
        }
      }
    },
    onClose: why => {
      log('Client disconnected.' + (why ? ' Reason: ' + why : ''));
    },
    onPong: (socket, data) => {
      log('PONG', data);
    }
  });
}

function HandleMessage(e) {
  var ev = e.data;
  log('HandleMessage', ev);

  switch (ev.type) {
    case 'abort':
      parent.postMessage({ type: 'done' });
      break;
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
