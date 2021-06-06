import { client, server, fetch } from 'net';
import * as std from 'std';
import * as os from 'os';
import { Console } from 'console';

var parent = os.Worker.parent;

function WorkerMain() {
  globalThis.console = new Console({
    colors: true,
    compact: 1,
    prefix: '\x1b[38;5;128mWORKER\x1b[0m'
  });

  console.log('WorkerMain.parent', parent);

  var i;

  parent.onmessage = HandleMessage;

  for(i = 0; i < 10; i++) {
    parent.postMessage({ type: 'num', num: i });
  }

  CreateServer();
}

WorkerMain();

function CreateServer(port = 9900) {
  print(`Listening on http://127.0.0.1:${port}`);
  server({
    port,
    mounts: [['/', '.', 'debugger.html']],
    onConnect: socket => {
      console.log('Client connected', socket);
    },
    onMessage: async (socket, msg) => {
      console.log('Received:', msg);
      if(typeof msg == 'string') {
        const json = JSON.parse(msg);
        console.log('Received:', json);

        if(json.type == 'start') {
          const { args, connect, address } = json.start;
          let child = StartDebugger(args, connect, address);

          /* console.log('child.wait()', child.wait());
          console.log('child(3).stderr', child.stderr);*/
          os.sleep(1000);
          sock = new Socket(IPPROTO_TCP);

          fs.onWrite(+sock, () => {
            console.log('writeable', +sock);
            fs.onWrite(+sock, null);
            connection = new DebuggerProtocol(sock);
            fs.onRead(+sock, () => {
              console.log('readable', +sock);
              if(connection) connection.read();
            });
          });

          ret = sock.connect('127.0.0.1', 9901);
          console.log('sock', sock);

          // sock.ndelay(true);

          console.log('ret:', ret);

          console.log('child(3)', child);
        } else if(sock) {
          sock.send(msg);
        }
      }
    },
    onClose: why => {
      console.log('Client disconnected.' + (why ? ' Reason: ' + why : ''));
    },
    onPong: (socket, data) => {
      console.log('PONG', data);
    }
  });
}
function HandleMessage(e) {
  var ev = e.data;
  console.log('Worker HandleMessage', ev);

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
