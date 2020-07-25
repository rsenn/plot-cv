import WebSocket from 'ws';
import WebSocketAsync from './lib/net/websocket-async.js';
import { WebSocketClient } from './lib/net/websocket-client.js';
import { websocketEvents, websocketData } from './lib/net/websocket-iterator.js';
import { ReconnectingWebSocket } from './lib/net/reconnectingWebSocket.js';
import Util from './lib/util.js';

import { Console } from 'console';
import { Message } from './message.js';

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 2, colors: true }
});

async function main() {
  console.log('WebSocket:', WebSocket, Util.isConstructor(WebSocket));
  //let ws = new WebSocketClient(WebSocket);
  const url = 'ws://127.0.0.1:3000/ws';
  let ws = new WebSocketAsync(/*url, ['appProtocol', 'appProtocol-v2'],*/ WebSocket);

  const dump = () => console.log('ws:', Util.getKeys(ws, ['receiveDataQueue', 'receiveCallbacksQueue', 'connected']));

  await ws.connect(url);

  await ws.send('test-websocket.js data!');

  for await (let data of ws) {
    let msg = new Message(data);
    console.log('data:', msg);
  }
  /*
  while(ws.connected) {
    let data = await ws.receive();

    console.log('data:', { data });
  }
    let it = websocketEvents(ws.socket);
*/

  // Close the connection.
  await ws.disconnect();
  dump();
}

main(process.argv.slice(2)).catch(error => {
  const stack = error.stack;
  console.log('ERROR:', error.message, stack);
});
