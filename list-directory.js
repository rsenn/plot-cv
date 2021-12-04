import { LogWrap, VfnAdapter, VfnDecorator, Memoize, DebugFlags, Mapper, DefaultConstructor, EventLogger, MessageReceiver, MessageTransmitter, MessageTransceiver, RPCApi, RPCProxy, RPCObject, RPCFactory, Connection, RPCServer, RPCClient, RPCSocket, isThenable, hasHandler, callHandler, parseURL, GetProperties, GetKeys, getPropertyDescriptors, define, setHandlers, statusResponse, objectCommand, MakeListCommand, getPrototypeName, SerializeValue, DeserializeSymbols, DeserializeValue, RPCConnect, RPCListen } from './quickjs/qjs-net/rpc.js';
//import * as React from './lib/dom/preactComponent.js';
import Util from './lib/util.js';
import { once, streamify, filter, map, throttle, distinct, subscribe } from './lib/async/events.js';
import iterify from './lib/async/iterify.js';

Object.assign(globalThis, { ListDirectory, LogWrap, VfnAdapter, VfnDecorator, Memoize, DebugFlags, Mapper, DefaultConstructor, EventLogger, MessageReceiver, MessageTransmitter, MessageTransceiver, RPCApi, RPCProxy, RPCObject, RPCFactory, Connection, RPCServer, RPCClient, RPCSocket, isThenable, hasHandler, callHandler, parseURL, GetProperties, GetKeys, getPropertyDescriptors, define, setHandlers, statusResponse, objectCommand, MakeListCommand, getPrototypeName, SerializeValue, DeserializeSymbols, DeserializeValue, RPCConnect, RPCListen });
// Add an async iterator to all WebSockets
/*WebSocket.prototype[Symbol.asyncIterator] = async function* () {
  while(true || this.readyState < 3) {
    yield await once(this, 'open','message','close','error');
  }
}
*/
globalThis.addEventListener('load', async () => {
  let url = Util.makeURL({ location: '/rpc/ws', protocol: 'wss', port: undefined });

  console.log('Loaded', url);

  let ws = (globalThis.ws = new WebSocket(url));

  let iter = streamify(['open', 'message', 'close', 'error'], ws);

  /*   ws.onopen = e => {
    console.log('WS established', e);
  };
  ws.onmessage = e => {
    console.log('WS message', e);
  };

 ws.onclose = e => {
    console.log('WS close', e);
  };
  ws.onerror = e => {
    console.log('WS error', e);
  };*/

  for await(let e of iter) {
    console.log(`WS ${e.type}`, e);
  }
});

async function ListDirectory(dir = '.', filter = '.*') {
  let response = await fetch('rpc/files', { method: 'POST', body: JSON.stringify({ dir, filter }) });
  console.log('ListDirectory', { response });
  let json = await response.json();
  console.log('ListDirectory', { json });
  return json;
}
