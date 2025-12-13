import { MessageReceiver, MessageTransmitter, MessageTransceiver, codecs, RPCApi, RPCProxy, RPCObject, RPCFactory, Connection, RPC_PARSE_ERROR, RPC_INVALID_REQUEST, RPC_METHOD_NOT_FOUND, RPC_INVALID_PARAMS, RPC_INTERNAL_ERROR, RPC_SERVER_ERROR_BASE, FactoryEndpoint, RPCServer, RPCClient, FactoryClient, RPCSocket, GetProperties, GetKeys, SerializeValue, DeserializeSymbols, DeserializeValue, RPCConnect, RPCListen, } from './quickjs/qjs-net/js/rpc.js';
import { BaseCache, CachedFetch, Implementations, lscache, brcache } from './lib/lscache.js';
import { propertyLookupHandlers, propertyLookup, lookupObject } from './lib/misc.js';

Object.assign(globalThis, {
  CreateWS,
  /*  BaseCache,
  CachedFetch,
  Implementations,
  lscache,
  brcache,
  GetTitle,
  rpc: {
    MessageReceiver,
    MessageTransmitter,
    MessageTransceiver,
    codecs,
    RPCApi,
    RPCProxy,
    RPCObject,
    RPCFactory,
    Connection,
    RPC_PARSE_ERROR,
    RPC_INVALID_REQUEST,
    RPC_METHOD_NOT_FOUND,
    RPC_INVALID_PARAMS,
    RPC_INTERNAL_ERROR,
    RPC_SERVER_ERROR_BASE,
    FactoryEndpoint,
    RPCServer,
    RPCClient,
    FactoryClient,
    RPCSocket,
    GetProperties,
    GetKeys,
    SerializeValue,
    DeserializeSymbols,
    DeserializeValue,
    RPCConnect,
    RPCListen,
  },
  ListAll,
  GlobAll,
  Consume,
  KVStorage,
  CreateElement,
  propertyLookupHandlers, propertyLookup, lookupObject,
  */
  now: () => +performance.now().toFixed(0),
});

globalThis.lsc = KVStorage();
//console.log('lsc.pages', await lsc.pages, now());

function KVStorage(
  g = async k => JSON.parse((await localStorage.getItem(k)) ?? 'null'),
  s = (k, v) => (localStorage.setItem(k, JSON.stringify(v)), true),
  K = () => {
    let r = [];
    for(let i = 0; i < localStorage.length; i++) r.push(localStorage.key(i));
    return r;
  },
) {
  return lookupObject(
    async (k, v) => (v === undefined ? g(k) : s(k, v)),
    {},
    {
      has(k) {
        return K().indexOf(k) != -1;
      },
    },
  );
}

async function Consume(iter) {
  let entry,
    ret = [];

  while((entry = await iter.next())) {
    const { value, done } = entry;
    if(done) break;

    ret.push(value);
  }

  return ret;
}

async function* ListAll(re = /^[^-]+\.html?$/gi) {
  let entry,
    dir = await cl.new('Directory', '.', 3, 1);
  while((entry = await dir.next(0b11, 1))) {
    const { value, done } = entry;
    if(done) break;
    const [name, type] = value;
    if(re.test(name)) yield name;
  }
}

async function GlobAll(pattern = '*.htm*', f = /^[^-]+\.html?$/gi) {
  const m = await cl.new('Match', pattern);
  let a = await m.apply();
  if(f) a = a.filter(([e, x]) => f.test(e));
  return a;
}

function CreatePageList(pages) {
  let e;

  while((e = document.body.firstElementChild.lastElementChild)) {
    if(e.tagName != 'UL') break;
    document.body.firstElementChild.removeChild(e);
  }

  let list = CreateElement(
    'ul',
    {},
    pages.map(([page, title]) => CreateElement('li', {}, [CreateElement('a', { href: page }, [document.createTextNode(page)]), document.createTextNode(' • '), document.createTextNode(title)])),
    document.body.firstElementChild,
  );
}

function CreateWS(subprotocols = ['ws']) {
  const u = new URL(window.location.href);
  u.protocol = /https/.test(u.protocol) ? 'wss:' : 'ws:';
  u.pathname = '/ws';
  let ws = new WebSocket(u + '', subprotocols);

  Object.assign(ws, {
    async onopen({ target }) {
      globalThis.cl = new FactoryClient(false);
      cl.send = msg => ws.send(msg);

      const pageList = await GlobAll();
      const pages = (globalThis.pages = pageList
        .map(([name, content]) => [name, GetTitle(content)])
        .filter(([name, content]) => !/^\s*$/.test(content) && !/^(404|directory|index)\.html?$/gi.test(name)));

      setTimeout(() => (lsc.pages = pages), 100);

      CreatePageList(pages);
    },
    onmessage({ data }) {
      const msg = JSON.parse(data);

      switch (msg.type) {
        case 'uuid':
          globalThis.uuid = msg.data;
          break;

        default: {
          try {
            cl.processMessage(msg);
          } catch(e) {
            console.log('WS MESSAGE', msg);
          }
          break;
        }
      }
    },
    onerror(e) {
      console.log('WS ERROR', e);
    },
    onclose({ wasClean, code, reason }) {
      console.log('WS CLOSE', { wasClean, code, reason });
    },
  });

  return ws;
}

function CreateElement(tag = 'div', attrs = {}, children = [], parent) {
  let e = document.createElement(tag);

  for(let a in attrs) e.setAttribute(a, attrs[a]);

  for(let c of children) e.appendChild(c);

  if(parent && parent.appendChild) parent.appendChild(e);

  return e;
}

function GetTitle(s) {
  let r = '';
  try {
    r = [.../.*<title>([^<>]*)<\/title>.*/gi.exec(s)][1];
  } catch(e) {}
  r = r.replaceAll('&quot;', '"');
  return r;
}

document.addEventListener('DOMContentLoaded', async e => {
  console.log('Loaded', now());

  let pages = (globalThis.pages = await lsc.pages);

  if(pages && Array.isArray(pages) && pages.length > 0) {
    CreatePageList(pages);
  } else {
    globalThis.ws = CreateWS();
  }
});