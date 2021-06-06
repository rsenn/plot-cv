import { client, server, fetch } from 'net';
import { Console } from 'console';
import { btoa } from 'misc';
import * as os from 'os';
import * as fs from 'fs';

function CreateServer(port = 9900) {
  print(`Listening on http://127.0.0.1:${port}`);
  server({
    port,
    mounts: [
      ['/', '.', 'debug.html'],
    ],
    onConnect: socket => {
      console.log('Client connected', socket);
    },
    onMessage: (socket, msg) => {
      print('Received:', msg);
    },
    onClose: why => {
      print('Client disconnected.' + (why ? ' Reason: ' + why : ''));
    },
    onPong: (socket, data) => {
      console.log('PONG', data);
    }
  });
}

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      colors: true,
      depth: 8,
      breakLength: 100,
      maxStringLength: Infinity,
      maxArrayLength: 30,
      compact: 0,
      showHidden: false
    }
  });
  let port = 9900;
  for(let arg of args) {
    if(!isNaN(+arg)) arg = +arg;

    if(typeof arg == 'number' && arg >= 1024 && arg <= 65535) port = arg;
  }

  CreateServer(port);
}

main(...scriptArgs.slice(1));
