import { WebSocketClient } from './lib/net/websocket-async.js';
import Util from './lib/util.js';
import { DebuggerProtocol } from './debuggerprotocol.js';
import { toString as ArrayBufferToString, toArrayBuffer as StringToArrayBuffer } from './lib/misc.js';

window.addEventListener('load', e => {
  console.log('Loaded');
  let socketURL = Util.makeURL({ location: '/ws', protocol: 'ws' });

  globalThis.CreateSocket = async function CreateSocket() {
    let ws = (Util.getGlobalObject().ws = new WebSocketClient());

    console.log('ws', ws);
    await ws.connect(socketURL);

    (async function ReadSocket() {
      for await(let msg of ws) {
        let data;
        try {
          data = JSON.parse(msg.data);
        } catch(e) {
          console.log('WS ERROR parsing', msg.data);
        }
        if(data) console.log('WS', data);
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

async function LoadSource(filename) {
  try {
    let response = await fetch(filename);
    return await response.text();
  } catch(e) {}
}

Object.assign(globalThis, {
  DebuggerProtocol,
  LoadSource,
  Util,
  ArrayBufferToString,
  StringToArrayBuffer
});
