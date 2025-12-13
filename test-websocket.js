import WebSocket from 'ws';
import WebSocketAsync from './lib/net/websocket-async.js';
import { Message } from './message.js';
async function main() {
  console.log('WebSocket:', WebSocket, Util.isConstructor(WebSocket));

  const url = 'ws://127.0.0.1:3000/ws';
  let ws = new WebSocketAsync(WebSocket);

  const dump = () => console.log('ws:', Util.getKeys(ws, ['receiveDataQueue', 'receiveCallbacksQueue', 'connected']));

  await ws.connect(url);

  // await ws.send('test-websocket.js data!');

  ws.sendMessage = function(...args) {
    let { data } = new Message(...args);
    console.debug(`send => '${data}'`);

    return this.send(data);
  };

  let myId;
  ws.sendMessage({ type: 'PING', body: Date.now() });
  for await(let data of ws) {
    for(let line of data.split(/\n/g)) {
      let msg = new Message(line);
      //console.log(`line = '${line}'`);

      switch (msg.type) {
        case 'HELLO': {
          myId = msg.body;
          console.log(`Your client Id is '${myId}'`);
          break;
        }
        case 'PONG': {
          console.log(`PONG '${msg.body}'`);
          break;
        }
        case 'USERS': {
          console.log(`USERS '${msg.body}'`);
          for(let id of [...msg.body, myId]) ws.send(`INFO ${id}`);
          break;
        }
        case 'INFO': {
          console.log(`Info for '${msg.origin}':`, msg.body);

          if(msg.origin == myId) ws.sendMessage({ type: 'QUIT', body: 'reason' });

          break;
        }
        default: {
          console.log('Message:', msg);
          break;
        }
      }
    }
  }

  /*
  while(ws.connected) {
    let data = await ws.receive();

    console.log('data:', { data });
  }
    let it = websocketEvents(ws.socket);
*/

  // Close the connection.
  process.exit(0);
  dump();
}

main(...scriptArgs.slice(1));