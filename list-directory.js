import { LogWrap, VfnAdapter, VfnDecorator, Memoize, DebugFlags, Mapper, DefaultConstructor, EventLogger, MessageReceiver, MessageTransmitter, MessageTransceiver, RPCApi, RPCProxy, RPCObject, RPCFactory, Connection, RPCServer, RPCClient, RPCSocket, isThenable, hasHandler, callHandler, parseURL, GetProperties, GetKeys, getPropertyDescriptors, define, setHandlers, statusResponse, objectCommand, MakeListCommand, getPrototypeName, SerializeValue, DeserializeSymbols, DeserializeValue, RPCConnect, RPCListen } from './quickjs/qjs-net/rpc.js';
//import * as React from './lib/dom/preactComponent.js';
import Util from './lib/util.js';
import { once, streamify, filter, map, throttle, distinct, subscribe } from './lib/async/events.js';
import iterify from './lib/async/iterify.js';

Object.assign(globalThis, { Util, ListDirectory, LogWrap, VfnAdapter, VfnDecorator, Memoize, DebugFlags, Mapper, DefaultConstructor, EventLogger, MessageReceiver, MessageTransmitter, MessageTransceiver, RPCApi, RPCProxy, RPCObject, RPCFactory, Connection, RPCServer, RPCClient, RPCSocket, isThenable, hasHandler, callHandler, parseURL, GetProperties, GetKeys, getPropertyDescriptors, define, setHandlers, statusResponse, objectCommand, MakeListCommand, getPrototypeName, SerializeValue, DeserializeSymbols, DeserializeValue, RPCConnect, RPCListen });

globalThis.addEventListener('load', async () => {
  let url = Util.makeURL({ location: '/rpc/ws', protocol: 'wss', port: undefined });

  console.log('Loaded', url);

  let ws = (globalThis.ws = new WebSocket(url));

  let iter = streamify(['open', 'message', 'close', 'error'], ws);

  for await(let e of iter) {
    console.log(`WS ${e.type}`, e);
  }
});

async function ListDirectory(dir = '.', options = {}) {
  const { filter = '.*', key = 'mtime', ...opts } = typeof options == 'string' ? { filter: options } : options;
  let response = await fetch('rpc/files', { method: 'POST', body: JSON.stringify({ dir, filter, key, ...opts }) });

  //  console.log('ListDirectory', { response });

  return await response.json();
}
