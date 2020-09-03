import WebSocket from 'ws';
import WebSocketAsync from './lib/net/websocket-async.js';
import ConsoleSetup from './consoleSetup.js';
import { WebSocketClient } from './lib/net/websocket-client.js';
import { websocketEvents } from './lib/net/websocket-iterator.js';
import Util from './lib/util.js';

import { Message } from './message.js';

async function main() {
  await ConsoleSetup();
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

main(Util.getArgs()).catch((error) => {
  const stack = error.stack;
  console.log('ERROR:', error.message, stack);
});
