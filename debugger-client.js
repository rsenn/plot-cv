import { WebSocketClient } from './lib/net/websocket-async.js';
import Util from './lib/util.js';

window.addEventListener('load', e => {
  console.log('Loaded');
  let socketURL = Util.makeURL({ location: '/ws', protocol: 'ws' });

  globalThis.CreateSocket = async function CreateSocket() {
    let ws = (Util.getGlobalObject().ws = new WebSocketClient());

    console.log('ws', ws);
    await ws.connect(socketURL);

    (async function ReadSocket() {
      for await(let msg of ws) {
        let data = JSON.parse(msg.data);
        console.log('WS', data);
      }
    })();

    await ws.send(JSON.stringify({
        type: 'start',
        start: { connect: false, args: ['test-ecmascript2.js'], address: '127.0.0.1:9901' }
      })
    );
  };
  CreateSocket();
});
