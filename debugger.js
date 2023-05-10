import { Console } from 'console';
import { toString } from './lib/misc.js';
import { Worker, close, exec, pipe, setReadHandler, sleep } from 'os';
//import child_process from './lib/childProcess.js';
import { assert, define, toString as ArrayBufferToString, btoa, keys, error } from './lib/misc.js';
import { DebuggerProtocol } from './debuggerprotocol.js';
import { readAll } from 'fs';
import { Spawn, WriteFile } from './io-helpers.js';
import { URLWorker } from './os-helpers.js';
import { Location, ECMAScriptDefines, ECMAScriptRules, ECMAScriptLexer, Lexer } from './quickjs/qjs-modules/lib/lexer/ecmascript.js';
import { consume as consumeSync } from './lib/iterator/helpers.js';
import { consume } from './lib/async/helpers.js';

var worker;
var counter;
let sockets = (globalThis.sockets ??= new Set());
let listeners = (globalThis.listeners = {});

export function ECMAScriptSyntaxHighlighter(input, filename) {
  const lexer = new ECMAScriptLexer(input, filename);
  let prev = 0,
    s = '';
  consumeSync(lexer.values(), ({ id, type, lexeme }) => {
    const color = {
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
      whitespace: 0
    }[type];

    if(color || lexeme.indexOf('\n') != -1) if (prev != color) lexeme = '\x1b[' + (color ? '1;' : '') + color + 'm' + lexeme;
    s += lexeme;
    if(color) prev = color;
  });

  if(prev) s += '\x1b[0m';
  return s;
}

export class DebuggerDispatcher {
  #seq = 0;
  #responses = {};
  #promise = null;
  onclose = () => console.log('CLOSED');
  onerror = ({ errno, message }) => console.log('ERROR', { errno, message });

  constructor(conn) {
    // const orig = sock.onmessage;

    console.log('DebuggerDispatcher', { conn });
    const copts = console.config({ maxStringLength: Infinity, maxArrayLength: Infinity, compact: 100 });

    conn
      .process(msg => {
        if(process.env.DEBUG) console.log('\x1b[38;5;220mRECEIVE\x1b[0m ', copts, msg);

        const { type, event, request_seq, body } = msg;

        switch (type) {
          case 'response':
            this.#responses[request_seq](msg);
            break;
          case 'event':
            const name = event.type.slice(0, event.type.indexOf('Event')).toLowerCase();

            for(let obj of [this, conn]) {
              const handler = obj['on' + name];

              if(handler) {
                if(handler.call(obj, event) === false) {
                  if(obj['on' + name] === handler) delete obj['on' + name];
                }
                break;
              }
            }
            break;
          default:
            //console.log('DebuggerDispatcher', { msg });
            if(sock.onmessage) sock.onmessage(msg);
            break;
        }
      })
      .then(ret => {
        if(ret == 0) this.onclose && this.onclose();
        if(ret < 0) this.onerror && this.onerror(error());
      });

    define(this, {
      sendMessage: msg => (process.env.DEBUG && console.log('\x1b[38;5;33mSEND\x1b[0m    ', copts, msg), conn.sendMessage((msg = JSON.stringify(msg))))
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

  sendRequest(command, args = {}) {
    const request_seq = ++this.#seq;

    this.sendMessage({ type: 'request', request: { request_seq, command, args } });

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

function TestWorker() {
  globalThis.console = new Console({
    colors: true,
    compact: 1,
    prefix: '\x1b[38;5;220mPARENT\x1b[0m'
  });
  console.log('scriptArgs', scriptArgs);
  worker = new Worker('./ws-worker.js');
  counter = 0;
  worker.onmessage = WorkerMessage;
  console.log('TestWorker', worker.onmessage);
  setReadHandler(0, () => {
    let line = process.stdin.getline();
    worker.postMessage({ line });
  });
}

let sock, connection;
function WorkerMessage(e) {
  console.log('WorkerMessage', e);
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
}

export async function LoadAST(source) {
  const script = `import * as std from 'std';
import { Worker } from 'os';
import { existsSync, readerSync } from 'fs';
import { Spawn } from './io-helpers.js';
import inspect from 'inspect';
import { Console } from 'console';
import { toString, gettid } from 'util';

globalThis.console = new Console({ inspectOptions: { compact: 2, customInspect: true, maxArrayLength: 200, prefix: '\\x1b[2K\\x1b[G\\x1b[1;33mWORKER\\x1b[0m ' } });

const worker = Worker.parent;

worker.onmessage = async ({ data }) => {
  const { type, source } = data;

  switch(type) {
    case 'gettid': {
      worker.postMessage({ tid: gettid() });
      break;
    }
    case 'quit': {
      worker.onmessage = null;
      console.log('quitting thread ('+gettid()+')...');
      break;
    }
    default: {
      const ast = await loadAST(source);
      worker.postMessage({ ast });
      break;
    } 
  }
};

async function loadAST(source) {
  if(!existsSync(source)) return null;
  const { stdout, wait } = Spawn('meriyah', [source], { block: false, stdio: ['inherit', 'pipe', 'inherit'] });
  
  let s = '';
  for(let chunk of readerSync(stdout))
    s += toString(chunk);

  const [pid, status] = wait();
  const { length } = s;
  //console.log('loadAST', { source, length, status });
  
  return JSON.parse(s);
}`;

  WriteFile('load-ast.js', script);

  let worker = new URLWorker(script);
  worker.postMessage({ source });
  const { value, done } = await worker.next();

  worker.postMessage({ type: 'quit' });

  console.log('worker.next()', console.config({ maxStringLength: 10 }), { value, done });
  const { data } = value;
  return ({ string: JSON.parse }[typeof data.ast] ?? (a => a))(data.ast);
}

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

export function* FindFunctions(ast) {
  for(let [v, p] of deep.iterate(ast, v => /^Func/.test(v.type))) {
    let name;

    try {
      if(p.last == 'value') name = deep.get(ast, p.slice(0, -1).concat(['key', 'name']));
    } catch(e) {}

    name ??= v.id?.name;

    if(name) yield [name, v.loc.start, [...GetArguments(v)], v.start];
  }
}
