import { filter, map, streamify } from './lib/async.js';
import { Element } from './lib/dom/element.js';
import { Fragment, h, options, render } from './lib/dom/preactComponent.js';
import * as path from './lib/path.js';
import { callHandler, Connection, DefaultConstructor, define, DeserializeSymbols, DeserializeValue, EventLogger, GetKeys, GetProperties, getPropertyDescriptors, getPrototypeName, hasHandler, isThenable, LogWrap, MakeListCommand, Mapper, MessageReceiver, MessageTransceiver, MessageTransmitter, objectCommand, parseURL, RPCApi, RPCClient, RPCConnect, RPCFactory, RPCListen, RPCObject, RPCProxy, RPCServer, RPCSocket, SerializeValue, setHandlers, statusResponse, VfnAdapter, VfnDecorator } from './quickjs/qjs-net/js/rpc.js';

Object.assign(globalThis, {
  callHandler,
  Connection,
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
  LogWrap,
  MakeListCommand,
  Mapper,
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
  Refresh,
  path,
  GetDir,
  Round,
  Table2Array,
  HumanSize,
  Refresh,
  Row2Obj,
  Element
});
let columns = ['mode', 'name', 'size', 'mtime'];

globalThis.addEventListener('load', async () => {
  StartSocket();

  ListDirectory().then(item => Refresh(item));

  //  Refresh();
});

async function StartSocket() {
  let origin = new URL(window.location.href);
  let url = new URL('/ws', Object.assign(origin, { protocol: origin.protocol.replace('http', 'ws') }));

  console.log('Connect to', url);
  let ws = (globalThis.ws = new WebSocket(url));

  let iter = streamify(['open', 'message', 'close', 'error'], ws);

  for await(let e of iter) {
    console.log(`WS ${e.type}`, e);
  }
}

function Table2Array(table = 'table') {
  let e = Element.find(table);
  let rows = [...e.rows];
  //let arr = [...e.rows].map(row => [...row.children]);
  for(let row of rows) {
    row.parentElement.removeChild(row);
  }
  return rows;
}

function Row2Obj(row) {
  let columns = [...row.children];
  let obj = {};
  for(let column of columns) {
    console.log('column', column);
    let field = [...column.classList].filter(n => n != 'item')[0];
    let data = column.getAttribute('data-value') ?? column.innerText;

    obj[field] = data;
  }
  return obj;
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

const input = {
  mode(s, obj, name) {
    return h('td', { class: `mode item`, 'data-value': s }, [
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
  name(s, obj, name) {
    return h('td', { class: `name item` }, h('a', { href: path.normalize(s), onClick }, [s.replace(/\/*$/, '').replace(/.*\//g, '')]));
  },
  size(s, obj, name) {
    return h('td', { class: `size item`, 'data-value': s }, (obj.name ?? '').endsWith('/') ? [] : [HumanSize(+s)]);
  },
  mtime(epoch) {
    let date = new Date(epoch * 1000);
    let [Y, M, D, hr, mi, se, ms] = date.toISOString().split(/[^0-9]+/g);
    return h('td', { class: `mtime item`, 'data-value': epoch }, [`${Y}/${M}/${D} ${hr}:${mi}:${se}`]);
  }
};

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
    { 'data-name': obj.name, 'data-path': obj.name },
    columns.map(field => [input[field] ? input[field](obj[field], obj, field) : obj[field]])
  );
}

function onClick(event) {
  let { target, currentTarget } = event;

  while(target.parentElement && target.tagName != 'TR') target = target.parentElement;
  const name = target.getAttribute('data-path') ?? '';

  if(name.endsWith('/')) {
    let dir = GetDir();

    //dir = path.join(dir, name);
    // const name =target.parentElement.attributes['data-name'].value;
    console.log('onClick', { target, name, dir });
    ListDirectory(name).then(item => {
      console.log('item', item);
      Refresh(item);
    });
    event.preventDefault(true);
    return false;
  }
}

function TableHeader() {
  let titles = ['Mode', 'Name', 'Size', 'Modification Time'];
  return h(
    'tr',
    { class: 'head' },
    columns.map((name, i) => h('th', { class: `${name} header` }, [h('a', { href: `#?sort=${name}`, onClick }, [titles[i]])]))
  );
}

function GetDir() {
  return document.querySelector('#dir').innerText;
}

function Refresh([dir, list]) {
  list = list.filter(({ name }) => !/^\.\/$/.test(name));

  if(!list.some(({ name }) => /^\.\.\/$/.test(name)))
    list.unshift({
      name: '../',
      mtime: 0,
      time: 0,
      mode: 0
    });
  //if(!list.some(({ name }) => /^\.\/$/.test(name))) list.unshift({ name: './', mtime: 0, time: 0, mode: 0 });
  const names = list.map(({ name }) => name);
  console.log('Refresh', { names });

  list = list.map(({ name, ...obj }) => ({
    ...obj,
    name: dir + '/' + name
  }));

  list = list.map(obj => h(TableItem, obj));

  let component = h(Fragment, {}, [
    h('h1', {}, [`Index of `, h('span', { id: 'dir' }, [dir])]),
    h(
      'table',
      {
        class: 'list preformatted',
        cellpadding: 2,
        cellspacing: 0
      },
      [TableHeader(), ...list]
    )
  ]);
  render(component, document.body);
  console.log('rendered');
}

async function ListDirectory(dir = '.', options = { objects: true }) {
  dir = path.normalize(dir);

  const { filter = '.*', key = 'mtime', ...opts } = typeof options == 'string' ? { filter: options } : options;
  let response = await fetch('files?filter=.*', {
    method: 'GET'
    // body: JSON.stringify({ dir, filter, key, ...opts })
  });
  try {
    return [dir, await response.json()];
  } catch(e) {
    throw new Error('Error parsing');
  }
}