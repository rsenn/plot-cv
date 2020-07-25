import WebSocket from 'ws';
import WebSocketClient from './lib/net/websocket-async.js';
import { websocketEvents, websocketData } from './lib/net/websocket-iterator.js';
import { ReconnectingWebSocket } from './lib/net/reconnectingWebSocket.js';
import Util from './lib/util.js';

import { Console } from 'console';

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 2, colors: true }
});

async function main() {
  console.log('WebSocket:', WebSocket, Util.isConstructor(WebSocket));
  let ws = new WebSocketClient(WebSocket);
  const url = 'ws://127.0.0.1:3000/ws';
  let rws = new ReconnectingWebSocket(url, ['appProtocol', 'appProtocol-v2'], { ctor: WebSocket });
  const dump = () => console.log('ws:', Util.getKeys(ws, ['receiveDataQueue', 'receiveCallbacksQueue', 'connected']));

  await ws.connect(url);
  dump();

  await ws.send('Hello!');
  dump();

  let it = websocketEvents(ws.socket);

  for await (let { type, data } of it) {
    console.log('data:', { type, data });
  }

  // Close the connection.
  await ws.disconnect();
  dump();
}

main(process.argv.slice(2)).catch(error => {
  const stack = error.stack;
  console.log('ERROR:', error.message, stack);
});
