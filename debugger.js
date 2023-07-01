import { toString, ansiStyles, assert, define, error, isFunction } from './lib/misc.js';
import { consume as consumeSync } from './lib/iterator/helpers.js';
import { Pointer } from './lib/pointer.js';
import * as deep from './lib/deep.js';

globalThis.process ??= { env: {} };

var worker;
var counter;
let sockets = (globalThis.sockets ??= new Set());
let listeners = (globalThis.listeners = {});

const { redBright, greenBright, cyanBright, yellowBright, magentaBright } = ansiStyles;
const syntaxPalette = [{ open: '\x1b[0m' }, redBright, greenBright, yellowBright, cyanBright, magentaBright].map(c => c.open);

export function TrivialTokenizer(input) {
  const re =
    /(\n|\t| )|(\b(?:arguments|as|async|await|break|case|catch|class|const|constructor|continue|debugger|default|delete|do|else|enum|eval|export|extends|false|finally|for|from|function|get|identifier|if|implements|import|in|instanceof|interface|let|meta|new|null|number|of|package|private|protected|public|return|set|static|string|super|switch|target|this|throw|true|try|typeof|var|void|while|with|yield)\b)|(\/\*(?:[^*]\/|[^\/])*\*\/|\/\/[^\n]*\n)|([A-Za-z_][A-Za-z_0-9]*)|("(?:\\"|[^"\n])*"|'(?:\\'|[^'\n])*'|[^\sA-Za-z_'"])/g;
  let match,
    prev,
    ret = [];

  while((match = re.exec(input))) {
    let tokenType = match.findIndex((m, i) => i > 0 && m !== undefined);
    const { index } = match;
    let str = match[tokenType] ?? match[0];

    ret.push([[undefined, 'whitespace', 'keyword', 'comment', 'identifier', 'string'][tokenType], str]);
    prev = tokenType;
  }
  return ret;
}

export function TrivialSyntaxHighlighter(input) {
  const re =
    /(\n|\t| )|(\b(?:arguments|as|async|await|break|case|catch|class|const|constructor|continue|debugger|default|delete|do|else|enum|eval|export|extends|false|finally|for|from|function|get|identifier|if|implements|import|in|instanceof|interface|let|meta|new|null|number|of|package|private|protected|public|return|set|static|string|super|switch|target|this|throw|true|try|typeof|var|void|while|with|yield)\b)|(\/\*(?:[^*]\/|[^\/])*\*\/|\/\/[^\n]*\n)|([A-Za-z_][A-Za-z_0-9]*)|("(?:\\"|[^"\n])*"|'(?:\\'|[^'\n])*'|[^\sA-Za-z_'"])/g;
  let match,
    prev,
    s = '';

  while((match = re.exec(input))) {
    let tokenType = match.findIndex((m, i) => i > 0 && m !== undefined);
    const { index } = match;
    let str = match[tokenType] ?? match[0];

    if(tokenType == 1 || tokenType != prev) if (syntaxPalette[tokenType - 1]) str = syntaxPalette[tokenType - 1] + str;

    s += str;
    prev = tokenType;
  }
  return s;
}

/*export function ECMAScriptSyntaxHighlighter(input, filename) {
  const lexer = new ECMAScriptLexer(input, filename);
  let prev = 0,
    s = '';
  consumeSync(lexer.values(), ({ id, type, lexeme }) => {
    let color = {
      shebang: 32,
      comment: 32,
      regexpLiteral: 35,
      templateLiteral: 35,
      templateLiteralHead: 35,
      templateLiteralPart: 35,
      templateLiteralTail: 35,
      punctuator: 36,
      numericLiteral: 36,
      stringLiteral: 36,
      booleanLiteral: 31,
      nullLiteral: 35,
      keyword: 31,
      identifier: 33,
      privateIdentifier: 33,
      whitespace: false
    }[type];

    if(lexeme.indexOf('\n') != -1) color = 0;

    if(color !== false) {
      if(prev != color) lexeme = '\x1b[' + (color ? '1;' : '') + color + 'm' + lexeme;
      prev = color;
    }

    s += lexeme;
  });

  if(prev) s += '\x1b[0m';
  return s;
}
*/
export class DebuggerDispatcher {
  #seq = 0;
  #responses = {};
  #callback = null;
  #promise = null;
  onclose = () => console.log('CLOSED');
  onerror = ({ errno, message }) => console.log('ERROR', { errno, message });

  constructor(conn) {
    console.log('DebuggerDispatcher', { conn });

    let ret;

    try {
      let v = conn.process(msg => {
        if(process.env.DEBUG) console.log('\x1b[38;5;220mRECEIVE\x1b[0m ', console.config({ compact: -1, depth: 10 }), msg);

        const { type, event, request_seq, body } = msg;

        switch (type) {
          case 'response':
            let fn = this.#responses[request_seq] ?? this.#callback;
            if(typeof fn == 'function') fn.call(this, msg);
            break;
          case 'event':
            const prop = 'on' + event.type.slice(0, event.type.indexOf('Event')).toLowerCase();

            for(let receiver of [this, conn]) {
              if(!receiver[prop]) continue;
              if(receiver[prop]) {
                const callback = receiver[prop];
                if(process.env.DEBUG) console.log('\x1b[38;5;56mEVENT\x1b[0m ', { prop, event });
                if(callback.call(receiver, event) === false) if (receiver[prop] === callback) delete receiver[prop];
              }
            }
            break;
          default:
            //console.log('DebuggerDispatcher', { msg });
            if(conn.onmessage) conn.onmessage(msg);
            break;
        }
      });
      console.log('process(handler) returned:', v);

      isFunction(v.then) && v.then(r => (ret = r));
    } catch(err) {
      console.log('process(handler) threw:', err.message + '\n' + err.stack);
      ret = -1;
    } finally {
      if(ret == 0) isFunction(this.onclose) && this.onclose();
      if(ret < 0) isFunction(this.onerror) && this.onerror(error());
      console.log('process(handler) function returned:', ret);
    }

    define(this, {
      sendMessage: async msg => {
        if(process.env.DEBUG) console.log('\x1b[38;5;33mSEND\x1b[0m    ', msg);
        const ret = await conn.sendMessage((msg = JSON.stringify(msg)));
        console.log('sendMessage() returned:', ret);
        return ret;
      }
    });
  }

  stepIn() {
    return this.sendRequest('stepIn').then(resp => this.waitRun());
  }
  stepOut() {
    return this.sendRequest('stepOut').then(resp => this.waitRun());
  }
  next() {
    return this.sendRequest('next').then(resp => this.waitRun());
  }
  continue() {
    return this.sendRequest('continue').then(resp => this.waitRun());
  }

  async waitRun() {
    process.env.DEBUG && console.log('\x1b[38;5;118mRUNNING\x1b[0m  ');
    this.running = true;
    const event = await waitEvent('stopped');
    process.env.DEBUG && console.log('\x1b[38;5;124mSTOPPED\x1b[0m  ');
    this.running = false;
    const trace = await this.stackTrace();
    return [event, trace];
  }

  pause() {
    return this.sendRequest('pause');
  }

  stopOnException(stopOnException = true) {
    return this.sendMessage({ type: 'stopOnException', stopOnException });
  }

  breakpoints(path, breakpoints) {
    if(Array.isArray(breakpoints)) {
      if(typeof breakpoints[0] == 'number') breakpoints = breakpoints.map(n => ({ line: n }));
    }
    const msg = breakpoints ? { type: 'breakpoints', breakpoints: { path, breakpoints } } : { type: 'breakpoints', path };
    this.sendMessage(msg);
    return msg;
  }

  async evaluate(frameId, expression) {
    return (await this.sendRequest('evaluate', { frameId, expression })).body;
  }

  async variables(variablesReference, options = {}) {
    if(Array.isArray(variablesReference)) {
      const [frame, scope] = variablesReference;
      variablesReference = frame * 4 + scope;
    }
    return (await this.sendRequest('variables', { variablesReference, ...options })).body;
  }

  async scopes(frameId) {
    return (await this.sendRequest('scopes', { frameId })).body;
  }

  async stackTrace() {
    let { body } = await this.sendRequest('stackTrace');

    return body;
  }

  waitEvent(name) {
    const prop = 'on' + name.toLowerCase();

    return new Promise(resolve => {
      this[prop] = arg => (resolve(arg), false);
    });
  }

  async sendRequest(command, args = {}) {
    const request_seq = ++this.#seq;

    await this.sendMessage({ type: 'request', request: { request_seq, command, args } });

    return new Promise(
      (resolve, reject) =>
        (this.#responses[request_seq] = response => {
          delete this.#responses[request_seq];
          resolve(response);
        })
    );
  }
}

Object.assign(DebuggerDispatcher.prototype, { [Symbol.toStringTag]: 'DebuggerDispatcher' });
Object.setPrototypeOf(DebuggerDispatcher.prototype, null);

/*function TestWorker() {
  globalThis.console = new Console({
    colors: true,
    compact: 1,
    prefix: '\x1b[38;5;220mPARENT\x1b[0m'
  });
  worker = new Worker('./ws-worker.js');
  counter = 0;
  worker.onmessage = WorkerMessage;
  setReadHandler(0, () => {
    let line = process.stdin.getline();
    worker.postMessage({ line });
  });
}

let sock, connection;
function WorkerMessage(e) {
  var ev = e.data;
  const { message, id } = ev;
  switch (ev.type) {
    case 'message': {
      switch (message.type) {
        case 'start': {
          console.log('START', message.start);
          const { args, connect, address } = message.start;
          let child = StartDebugger(args, connect, address);
          sleep(1000);
          sock = ConnectDebugger(address);
          break;
        }
        default: {
          console.log('From WORKER', ev);
          connection.sendMessage(message);
          break;
        }
      }
      break;
    }
    case 'num': {
      assert(ev.num, counter);
      counter++;
      if(counter == 10) {
        let sab = new SharedArrayBuffer(10);
        let buf = new Uint8Array(sab);
        worker.postMessage({ type: 'sab', buf: buf });
        counter = 0;
      }
      break;
    }
    case 'sab_done': {
      let buf = ev.buf;
      assert(buf[2], 10);
      worker.postMessage({ type: 'abort' });
      break;
    }
    case 'done': {
      break;
    }
  }
}

function send(id, body) {
  worker.postMessage({ type: 'send', id, body });
}*/

export function* GetArguments(node) {
  for(let param of node.params) {
    let name = '';

    if(param.type == 'AssignmentPattern') param = param.left;

    if(param.type == 'RestElement') {
      name += '...';
      param = param.argument;
    }

    if(param.type == 'Identifier') name += param.name;

    if(name !== '') yield name;
    else yield param;
  }
}

export function GetFunctionName(ast, p) {
  try {
    let ptr = new Pointer(p);
    let h = ptr.hier().filter(p => (n => !Array.isArray(n))(deep.get(ast, p)));
    // console.log('h',h);
    //
    let start = 1 + h.slice(0, -1).findLastIndex(ptr => (n => /Func/.test(n.type))(deep.get(ast, ptr)));
    h = h.slice(start);

    //
    return h
      .map(p => (n => n.id ?? n.key ?? n)(deep.get(ast, p)))
      .map(s => (s && s.name) || s)
      .filter(s => typeof s == 'string')
      .join('.');
  } catch(e) {}
  let name;
  try {
    if(p.last == 'value') name = deep.get(ast, p.slice(0, -1).concat(['key', 'name']));
  } catch(e) {}
  let parent = deep.get(ast, p.slice(0, -1));

  if(parent.type == 'Property') {
    if(parent.key.type == 'Literal') {
      name ??= parent.key.value;
    }
  }
  name ??= v.id?.name;
  return name;
}

export function* FindFunctions(ast) {
  FindFunctions.paths ??= new WeakMap();

  for(let [v, p] of deep.iterate(ast, v => /^(Arrow|)Func/.test(v.type))) {
    let name = GetFunctionName(ast, p);
    let parent = deep.get(ast, p.slice(0, -1));

    /*    try {
      if(p.last == 'value') name = deep.get(ast, p.slice(0, -1).concat(['key', 'name']));
    } catch(e) {}

    if(parent.type == 'Property') {
      if(parent.key.type == 'Literal') {
        name ??= parent.key.value;
      }
    }*/

    if(v.async) name = 'async ' + name;

    if(v.loc.start) {
      let { line, column } = v.loc.start;
      ++column;

      /*if(name)*/ yield [name, { line, column }, [...GetArguments(v)], v.expression, p];
    }
  }
}
