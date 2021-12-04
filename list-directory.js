import { LogWrap, VfnAdapter, VfnDecorator, Memoize, DebugFlags, Mapper, DefaultConstructor, EventLogger, MessageReceiver, MessageTransmitter, MessageTransceiver, RPCApi, RPCProxy, RPCObject, RPCFactory, Connection, RPCServer, RPCClient, RPCSocket, isThenable, hasHandler, callHandler, parseURL, GetProperties, GetKeys, getPropertyDescriptors, define, setHandlers, statusResponse, objectCommand, MakeListCommand, getPrototypeName, SerializeValue, DeserializeSymbols, DeserializeValue, RPCConnect, RPCListen } from './quickjs/qjs-net/rpc.js';
//import * as React from './lib/dom/preactComponent.js';
import Util from './lib/util.js';

window.addEventListener('load', () => {
  let url = Util.makeURL({ location: '/rpc/ws', protocol: 'wss', port: undefined });

  console.log('Loaded', url);

  let ws = globalThis.ws = new WebSocket(url);
ws.onopen = e => {
  console.log('WS established',e);

}
ws.onmessage = e => {
console.log('WS message',e);

}
ws.onclose = e => {
console.log('WS close',e);

}
ws.onerror = e => {
console.log('WS error',e);

}


});
