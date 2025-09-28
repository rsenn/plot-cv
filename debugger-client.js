#!/usr/bin/env qjsm
import { Button, Panel } from './components.js';
import { DebuggerDispatcher, TrivialTokenizer } from './debugger.js';
import { ReconnectingWebSocket, WebSocketURL } from './lib/async/websocket.js';
import { classNames } from './lib/classNames.js';
import { Fragment, h, render, toChildArray } from './lib/dom/preactComponent.js';
import { useFetch, useTrkl } from './lib/hooks.js';
import { JSLexer } from './lib/jslexer.js';
import { define, weakDefine, memoize, rand } from './lib/misc.js';
import { trkl } from './lib/trkl.js';
import {
  MessageReceiver,
  MessageTransmitter,
  MessageTransceiver,
  parseURL,
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
  RPCListen
} from './quickjs/qjs-net/js/rpc.js';

let cwd = '.';
let responses = {};
let currentSource = trkl(null);
let currentLine = trkl(-1);
let url;
let seq = 0,
  numLines = trkl(0);

function GetCurrentPos() {
  return [currentSource(), currentLine()];
}

function RGBA(r, g, b, a = 255) {
  function toHex(n) {
    return Number(n).toString(16).padStart(2, '0');
  }
  return {
    r,
    g,
    b,
    a,
    hex() {
      if (this.a == 255) return toHex(this.r) + toHex(this.g) + toHex(this.b);
      return toHex(this.r) + toHex(this.g) + toHex(this.b) + toHex(this.a);
    }
  };
}

weakDefine(WebSocket.prototype, {
  sendMessage(msg) {
    return this.send(JSON.stringify(msg));
  }
});

globalThis.process = { env: { DEBUG: true } };

currentSource.id = 'currentSource';
currentLine.id = 'currentLine';

currentSource.subscribe(source => console.log('currentSource set to', source));
currentLine.subscribe(line => console.log('currentLine set to', line));
currentLine.subscribe(line => {
  let e;

  if ((e = document.querySelector('main'))) {
    const numLines = Math.floor(e.offsetHeight / document.querySelector('.source > pre').offsetHeight);
    let pos = Math.max(0, line - (numLines >>> 1));
    window.location.hash = `#line-${pos}`;
  }
});

const doRender = memoize(RenderUI);

window.addEventListener('load', e => {
  url = new URL(document.location.href);

  let socketURL = new URL('ws', url);
  socketURL.protocol = socketURL.protocol.replace('http', 'ws');
  console.log('socketURL', socketURL);

  (async () => {
    await CreateSocket(socketURL);
    console.log(`Loaded`, { socketURL, ws });
  })();
});

globalThis.addEventListener('keypress', e => {
  const handler = {
    KeyN: Next,
    KeyI: StepIn,
    KeyO: StepOut,
    KeyC: Continue,
    KeyP: Pause
  }[e.code];
  //console.log('keypress', e, handler);

  if (handler) handler();
});

/******************************************************************************
 * Components                                                                 *
 ******************************************************************************/
const SourceLine = ({ lineno, text, active, children }) => {
  return h(Fragment, {}, [
    h('a', { name: `line-${lineno.trim()}` }, []),
    h(
      'pre',
      {
        class: classNames('text', active && 'active')
        //innerHTML: `${lineno} ` + text
      },
      toChildArray([h('span', { class: classNames('lineno', active && 'active', ['even', 'odd'][lineno % 2]) }, [`${lineno} `]), ...children])
    )
  ]);
};

const SourceText = ({ text, filename }) => {
  const activeLine = useTrkl(currentLine);
  const lines = text.split(/\n/g);
  const numDigits = n => Math.floor(Math.log10(n) + 1);
  const n = numDigits(lines.length) + 2;
  return h(
    'div',
    { class: 'source', 'data-name': filename },
    lines.map((line, i) =>
      h(
        SourceLine,
        {
          lineno: (i + 1 + '').padStart(n),
          active: activeLine == i + 1
          //text: TrivialTokenizer(line).reduce((acc, [type, token]) => acc + `<span class="${type}">${token}</span>`, '')
        },
        TrivialTokenizer(line).map(([type, token]) => h('span', { class: type }, [token]))
      )
    )
  );
};

const SourceFile = props => {
  console.log('props.file', currentSource());
  const file = useTrkl(currentSource);

  console.log('file', { cwd, file });

  const filename = file; /*? path.relative(cwd, file, cwd) : null*/

  let text =
    (file &&
      !/^<.*>$/.test(file) &&
      useFetch(filename, resp => {
        console.log('Fetch', resp.status, new URL(filename, document.location.href));
        return resp.text();
      })) ||
    '';

  return /*h('div', { class: 'container' }, [*/ h(Fragment, {}, [
    //h('div', {}, []),
    h('div', { class: 'header' }, [filename]),
    h(SourceText, { text, filename })
  ]);
};

/******************************************************************************
 * End of Components                                                          *
 ******************************************************************************/

async function LoadSource(filename) {
  try {
    let response = await fetch(filename);
    return await response.text();
  } catch (e) {}
}

function Start(args, address) {
  if (!address) {
    globalThis.address = address = '127.0.0.1:' + (Math.floor(Math.random() * 4096) + 8192);
  }

  return Initiate('start', address, false, args);
}

function Connect(address = globalThis.address) {
  return Initiate('connect', address, true);
}

function Initiate(command, address, connect = false, args) {
  address ??= `${url.searchParams.get('address') ?? '127.0.0.1'}:${(globalThis.port ??= url.searchParams.get('port') ?? (Math.floor(rand()) % 900) + 9000)}`;
  console.log('Initiate', { command, address, connect, args });
  return ws.send(JSON.stringify({ command, connect, address, args }));
}

const tokenColors = {
  comment: new RGBA(0, 255, 0),
  regexpLiteral: new RGBA(255, 0, 255),
  templateLiteral: new RGBA(0, 255, 255),
  punctuator: new RGBA(0, 255, 255),
  numericLiteral: new RGBA(0, 255, 255),
  string: new RGBA(0, 255, 255),
  booleanLiteral: new RGBA(255, 255, 0),
  nullLiteral: new RGBA(255, 0, 255),
  keyword: new RGBA(255, 0, 0),
  identifier: new RGBA(255, 255, 0),
  privateIdentifier: new RGBA(255, 255, 0),
  whitespace: new RGBA(255, 255, 255)
};

function* TokenizeJS(data, filename) {
  let lex = new JSLexer();
  lex.setInput(data, filename);

  let { tokens } = lex;
  let colors = Object.entries(tokenColors).reduce((acc, [type, c]) => ({ ...acc, [tokens.indexOf(type) + 1]: c.hex() }), {});
  let prev = {};
  let out = [];
  for (let { id, lexeme, line } of lex) {
    const type = tokens[id - 1];
    let { line } = lex.loc;
    line -= lexeme.split(/\n/g).length - 1;

    //console.log('tok', { id, lexeme, line });

    if (prev.line != line) {
      for (let i = prev.line; i < line; i++) {
        yield out;
        out = [];
      }
    }

    for (let s of lexeme.split(/\n/g).reduce((acc, l) => {
      if (l != '') {
        if (acc.length) acc[acc.length - 1] += '\n';
        acc.push(l);
      }
      return acc;
    }, [])) {
      out.push([type, s]);
      if (s.endsWith('\n')) {
        yield out;
        out = [];
        line++;
      }
    }

    prev.line = line;
    line = lex.loc;
  }
  out += '</pre>';
}

Object.assign(globalThis, {
  Start,
  Connect,
  StepIn,
  StepOut,
  Next,
  Until,
  Continue,
  Pause,
  Evaluate,
  StackTrace,
  SendRequest,
  GetCurrentPos
});
Object.assign(globalThis, { currentLine, currentSource, TokenizeJS });
Object.assign(globalThis, { CreateSocket, Start, Initiate, LoadSource, GetVariables });
Object.assign(globalThis, {
  WebSocketURL,
  parseURL,
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
  RPCListen
});

async function CreateSocket(endpoint) {
  let url = WebSocketURL('/ws');
  let rws = (globalThis.rws = new ReconnectingWebSocket(url, 'ws', {
    onOpen() {
      console.log('ReconnectingWebSocket connected!');
    }
  }));

  define(globalThis, {
    get ws() {
      return rws.socket;
    }
  });

  //let ws = (globalThis.ws = rws.ws/*new WebSocketClient()*/);

  console.log('ws', ws);
  //
  await rws.connect(endpoint);

  let dispatch = (globalThis.dispatch = new DebuggerDispatcher({
    async process(callback) {
      for await (let msg of rws) {
        let data = JSON.parse(msg);

        process.env.DEBUG && console.log('WS received:', data);

        callback(data);
      }
    },
    sendMessage: msg => rws.socket.sendMessage(msg)
  }));

  responses = globalThis.responses = dispatch.responses;

  ws.sendMessage = function (msg) {
    process.env.DEBUG && console.log('WS sending:', msg);
    return this.send(JSON.stringify(msg));
  };

  return rws;
}

function GetVariables(ref = 0) {
  return SendRequest('variables', { variablesReference: ref });
}

async function UpdatePosition() {
  const stack = (globalThis.stack = await StackTrace());
  console.log('stack', stack);

  const { filename, line, name } = stack[0];

  currentSource(filename);
  //  RenderUI(filename);
  currentLine(line);

  RenderUI();
}

async function StepIn() {
  await SendRequest('stepIn');
  await UpdatePosition();
}

async function StepOut() {
  await SendRequest('stepOut');
  await UpdatePosition();
}

async function Next() {
  await SendRequest('next');
  await UpdatePosition();
}
async function Until(cond = () => true) {
  do {
    await Next();
  } while (!cond());
}

async function Continue() {
  return SendRequest('continue');
}

async function Pause() {
  await SendRequest('pause');
  await UpdatePosition();
}

async function Evaluate(expression) {
  return SendRequest('evaluate', { expression });
}

async function StackTrace() {
  let { body } = await SendRequest('stackTrace');
  return body;
}

function SendRequest(command, args = {}) {
  const request_seq = ++seq;

  ws.sendMessage({ type: 'request', request: { request_seq, command, args } });

  return new Promise((resolve, reject) => (responses[request_seq] = resolve));
}

function RenderUI() {
  console.log('RenderUI');
  const component = h(Fragment, {}, [
    h(Panel, { className: classNames('buttons', 'no-select'), tag: 'header' }, [
      h(Button, { image: 'static/svg/continue.svg', fn: Continue }),
      h(Button, { image: 'static/svg/pause.svg', fn: Pause }),
      //h(Button, {image: 'static/svg/start.svg'}),
      h(Button, {
        image: 'static/svg/step-into.svg',
        fn: StepIn
      }),
      h(Button, {
        image: 'static/svg/step-out.svg',
        fn: StepOut
      }),
      h(Button, { image: 'static/svg/step-over.svg', fn: Next })
      //   h(Button, { image: 'static/svg/restart.svg' }),
      //h(Button, {image: 'static/svg/stop.svg', enable: trkl(false)}),
    ]),
    h('main', {}, h(SourceFile, { file: currentSource })),
    h('footer', {}, [])
  ]);
  const { body } = document;
  render(component, body);
  console.log('rendered', component);
}
