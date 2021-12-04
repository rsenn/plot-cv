import { LogWrap, VfnAdapter, VfnDecorator, Memoize, DebugFlags, Mapper, DefaultConstructor, EventLogger, MessageReceiver, MessageTransmitter, MessageTransceiver, RPCApi, RPCProxy, RPCObject, RPCFactory, Connection, RPCServer, RPCClient, RPCSocket, isThenable, hasHandler, callHandler, parseURL, GetProperties, GetKeys, getPropertyDescriptors, define, setHandlers, statusResponse, objectCommand, MakeListCommand, getPrototypeName, SerializeValue, DeserializeSymbols, DeserializeValue, RPCConnect, RPCListen } from './quickjs/qjs-net/rpc.js';
import { h, options, html, render, Component, createContext, createRef, useState, useReducer, useEffect, useLayoutEffect, useRef, useImperativeHandle, useMemo, useCallback, useContext, useDebugValue, forwardRef, Fragment, React, ReactComponent, Portal, toChildArray } from './lib/dom/preactComponent.js';
import Util from './lib/util.js';
import { once, streamify, filter, map, throttle, distinct, subscribe } from './lib/async/events.js';
import iterify from './lib/async/iterify.js';

Object.assign(globalThis, {
  callHandler,
  Connection,
  DebugFlags,
  DefaultConstructor,
  define,
  DeserializeSymbols,
  DeserializeValue,
  EventLogger,
  GetKeys,
  GetProperties,
  getPropertyDescriptors,
  getPrototypeName,
  hasHandler,
  isThenable,
  ListDirectory,
  LogWrap,
  MakeListCommand,
  Mapper,
  Memoize,
  MessageReceiver,
  MessageTransceiver,
  MessageTransmitter,
  objectCommand,
  parseURL,
  RPCApi,
  RPCClient,
  RPCConnect,
  RPCFactory,
  RPCListen,
  RPCObject,
  RPCProxy,
  RPCServer,
  RPCSocket,
  SerializeValue,
  setHandlers,
  statusResponse,
  Util,
  VfnAdapter,
  VfnDecorator,
  Refresh
});
let columns = ['mode', 'name', 'size', 'mtime'];
let input = {
  mode(s) {
    return +s;
  },
  name(s) {
    return s;
  },
  size(s) {
    return +s;
  },
  mtime(s) {
    return '' + new Date(s * 1000);
  }
};

globalThis.addEventListener('load', async () => {
  StartSocket();

  Refresh();
});

async function StartSocket() {
  let url = Util.makeURL({ location: '/rpc/ws', protocol: 'wss', port: undefined });

  console.log('Connect to', url);
  let ws = (globalThis.ws = new WebSocket(url));

  let iter = streamify(['open', 'message', 'close', 'error'], ws);

  for await(let e of iter) {
    console.log(`WS ${e.type}`, e);
  }
}

function Refresh(list = []) {
  let component = h(
    'list',
    {},
    list.map(obj => columns.map((name, i) => h(name, { class: 'item' }, [input[name] ? input[name](obj[name]) : obj[name]])))
  );
  render(component, document.body);
  console.log('rendered');
}

async function ListDirectory(dir = '.', options = { objects: true }) {
  const { filter = '.*', key = 'mtime', ...opts } = typeof options == 'string' ? { filter: options } : options;
  let response = await fetch('rpc/files', { method: 'POST', body: JSON.stringify({ dir, filter, key, ...opts }) });

  return await response.json();
}
