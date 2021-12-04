import { LogWrap, VfnAdapter, VfnDecorator, Memoize, DebugFlags, Mapper, DefaultConstructor, EventLogger, MessageReceiver, MessageTransmitter, MessageTransceiver, RPCApi, RPCProxy, RPCObject, RPCFactory, Connection, RPCServer, RPCClient, RPCSocket, isThenable, hasHandler, callHandler, parseURL, GetProperties, GetKeys, getPropertyDescriptors, define, setHandlers, statusResponse, objectCommand, MakeListCommand, getPrototypeName, SerializeValue, DeserializeSymbols, DeserializeValue, RPCConnect, RPCListen } from './quickjs/qjs-net/rpc.js';
import { h, options, html, render, Component, createContext, createRef, useState, useReducer, useEffect, useLayoutEffect, useRef, useImperativeHandle, useMemo, useCallback, useContext, useDebugValue, forwardRef, Fragment, React, ReactComponent, Portal, toChildArray } from './lib/dom/preactComponent.js';
import Util from './lib/util.js';
import { once, streamify, filter, map, throttle, distinct, subscribe } from './lib/async/events.js';
import iterify from './lib/async/iterify.js';
import trkl from './lib/trkl.js';

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
  mode(s, obj) {
    return h('td', { class: `${name} item` }, [
      [(obj.name ?? '').endsWith('/') ? 'd' : '-']
        .concat(
          parseInt(s, 8)
            .toString(2)
            .split('')
            .map((n, i) => (i < 9 ? (+n ? 'rwx'[i % 3] : '-') : +n))
        )
        .join('')
    ]);
  },
  name(s) {
    return h('td', { class: `${name} item` }, h('a', { href: s, onClick }, [s.replace(/\/$/, '')]));
  },
  size(s, obj) {
    return h('td', { class: `${name} item` }, (obj.name ?? '').endsWith('/') ? [] : [HumanSize(+s)]);
  },
  mtime(epoch) {
    let date = new Date(epoch * 1000);
    let [Y, M, D, hr, mi, se, ms] = date.toISOString().split(/[^0-9]+/g);
    return h('td', { class: `${name} item` }, [`${Y}/${M}/${D} ${hr}:${mi}:${se}`]);
  }
};

globalThis.addEventListener('load', async () => {
  StartSocket();

  ListDirectory().then(item => Refresh(item));

  //  Refresh();
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

function Round(n, digits = 3, f = Math.round) {
  return (f(n * Math.pow(10, digits)) * Math.pow(10, -digits)).toFixed(digits).replace(/0*$/g, '');
}

function HumanSize(n) {
  if(n >= 1e12) return Round(n / 1e12) + 'T';
  if(n >= 1e9) return Round(n / 1e9) + 'G';
  if(n >= 1e6) return Round(n / 1e6) + 'M';
  if(n >= 1e3) return Round(n / 1e3) + 'K';
  return n;
}
function Item(obj) {
  return h(
    Fragment,
    {},
    columns.map(name => h(name, { class: `item ${name}` }, [input[name] ? input[name](obj[name], obj, name) : obj[name]]))
  );
}

function TableItem(obj) {
  return h(
    'tr',
    { 'data-name': obj.name },
    columns.map(field => [input[field] ? input[field](obj[field], obj, field) : obj[field]])
  );
}

function onClick(event) {
  let { target, currentTarget } = event;

  while(target.parentElement && target.tagName != 'TR') target = target.parentElement;
  const name = target.getAttribute('data-name');

  // const name =target.parentElement.attributes['data-name'].value;
  console.log('onClick', { target, name });

  ListDirectory().then(item => Refresh(item));

  event.preventDefault(true);
  return false;
}

function TableHeader() {
  let titles = ['Mode', 'Name', 'Size', 'Modification Time'];
  return h(
    'tr',
    { class: 'head' },
    columns.map((name, i) => h('th', { class: `${name} header` }, [h('a', { href: `#?sort=${name}`, onClick }, [titles[i]])]))
  );
}

function Refresh([dir, list]) {
  let component = h(Fragment, {}, [h('h1', {}, [`Index of ${dir}`]), h('table', { class: 'list preformatted', cellpadding: 2, cellspacing: 0 }, [TableHeader(), ...list.filter(({ name }) => !/^\.\/$/.test(name)).map(obj => h(TableItem, obj))])]);
  render(component, document.body);
  console.log('rendered');
}

async function ListDirectory(dir = '.', options = { objects: true }) {
  const { filter = '.*', key = 'mtime', ...opts } = typeof options == 'string' ? { filter: options } : options;
  let response = await fetch('rpc/files', { method: 'POST', body: JSON.stringify({ dir, filter, key, ...opts }) });

  return [dir, await response.json()];
}
