import { existsSync, reader, readerSync, readSync, writeSync } from 'fs';
import { createServer, getSessions, LLL_INFO, LLL_NOTICE, LLL_USER, LLL_WARN, logLevels, setLog } from 'net';
import { kill, setReadHandler, SIGTERM, sleep, ttySetRaw, Worker } from 'os';
import { clearInterval, setInterval, setTimeout } from 'timers';
import { atexit, bindMethods, btoa, define, keys, filterKeys, getOpt, isObject, lazyProperties, memoize, mod, once, propertyLookup, quote, tryCatch, types, mapWrapper } from 'util';
import { List, Table } from './cli-helpers.js';
import { DebuggerDispatcher, FindFunctions, GetFunctionName, TrivialSyntaxHighlighter } from './debugger.js';
import { DebuggerProtocol } from './debuggerprotocol.js';
import { ReadFile, ReadJSON, WriteJSON } from './io-helpers.js';
import { consume, map } from './lib/async/helpers.js';
import { absolute, basename, extname, relative } from './lib/path.js';
import { Repeater } from './lib/repeater/repeater.js';
import { Spawn, WNOHANG } from './os-helpers.js';
import { F_GETFL, F_SETFL, fcntl, O_NONBLOCK } from './quickjs/qjs-ffi/lib/fcntl.js';
import { REPL } from './quickjs/qjs-modules/lib/repl.js';
import { Console } from 'console';
import { Location } from 'location';
import process from 'process';
import * as path from 'path';
import extendArray from 'extendArray';
import { AF_INET, AsyncSocket, IPPROTO_TCP, SOCK_STREAM, SockAddr } from 'sockets';
import { err as stderr } from 'std';
import { TextDecoder } from 'textcode';
import { codecs, RPCApi, RPCProxy, RPCObject, RPCFactory, Connection, RPCServer, RPCClient, RPCSocket, RPCConnect, RPCListen } from './quickjs/qjs-net/js/rpc.js';

function toString(buf, start, len) {
  if('buffer' in buf) buf = buf.buffer;
  if(start !== undefined || len !== undefined) {
    start ??= 0;
    len ??= buf.byteLength;
    buf = buf.slice(start, start + len);
  }
  return new TextDecoder().decode(buf);
}

function decorate(decorators, obj, ...args) {
  if(!Array.isArray(decorators)) decorators = [decorators];

  for(let decorator of decorators)
    for(let prop of keys(obj))
      if(typeof obj[prop] == 'function') {
        let newfn = decorator(obj[prop], obj, prop, ...args);
        if(obj[prop] !== newfn) obj[prop] = newfn;
      }

  return obj;
}

extendArray(Array.prototype);

const scriptName = (arg = scriptArgs[0]) => basename(arg, extname(arg));

const children = new Set();

atexit(() => {
  for(let pid of children) {
    console.log('atexit killing child', pid);
    kill(pid, SIGTERM);
  }
});

Object.assign(globalThis, {
  codecs,
  RPCApi,
  RPCProxy,
  RPCObject,
  RPCFactory,
  Connection,
  RPCServer,
  RPCClient,
  RPCSocket,
  RPCConnect,
  RPCListen,
  toString,
});

const signalName = n =>
  'SIG' +
  [
    ,
    'HUP',
    'INT',
    'QUIT',
    'ILL',
    'TRAP',
    'ABRT',
    'BUS',
    'FPE',
    'KILL',
    'USR1',
    'SEGV',
    'USR2',
    'PIPE',
    'ALRM',
    'TERM',
    'STKFLT',
    'CHLD',
    'CONT',
    'STOP',
    'TSTP',
    'TTIN',
    'TTOU',
    'URG',
    'XCPU',
    'XFSZ',
    'VTALRM',
    'PROF',
    'WINCH',
    'IO',
    'PWR',
    'SYS',
  ][n];

function checkChildExited(child) {
  const { exited, termsig, signaled, exitcode } = child;

  return exited ? (signaled ? `signalled ${signalName(termsig)}` : `exitcode ${exitcode}`) : null;
}

function GetLoc(node) {
  if(node.loc?.start?.line) {
    const { line, column } = node.loc.start;
    const [charOffset] = node.range ?? [node.start];
    return new Location(line, column + 1, charOffset);
  }
}

async function LoadAST(source) {
  if(!existsSync(source)) return null;
  const child = Spawn('meriyah', ['-l', source], { block: false, stdio: ['inherit', 'pipe', 'inherit'] });

  let s = '';
  for(let chunk of readerSync(child.stdout)) s += toString(chunk);

  const status = child.wait();
  const { length } = s;
  //console.log('loadAST', { source, length, status });

  return JSON.parse(s);
}

function StartREPL(prefix = scriptName(), suffix = '') {
  let repl = new REPL(`\x1b[38;5;165m${prefix} \x1b[38;5;39m${suffix}\x1b[0m`, false);
  repl.historyLoad(null);
  let { log } = console;

  console.log = repl.printFunction(log.bind(console, console.config({ compact: 2 })));
  let { show } = repl;

  repl.show = arg => {
    if(isObject(arg)) {
      if(arg[Symbol.for('print')]) return arg.toString ? arg.toString() : arg + '';

      //if(Array.isArray(arg) && typeof arg[0] == 'object' &&  Array.isArray(arg[0])) {
      if(Array.isArray(arg) && typeof arg[0] == 'object') {
        if(!Array.isArray(arg[0]) && (arg.length !== 2 || !Array.isArray(arg[1]))) {
          if(arg.length == 2 && Array.isArray(arg[1])) {
            const [event, stack] = arg;
            if(['type', 'reason'].every(k => k in event)) if (['id', 'name', 'line'].every(k => k in stack[0])) return [List([event]), List(stack)];
          }

          if(
            arg.length >= 2 /*Object.keys(arg[0]).some(key => arg.every(a => key in a)) ||*/ &&
            arg.map(item => Object.keys(item)).reduce((acc, keys, i) => (i == 0 ? keys : acc ? keys.equal(acc) && keys : false))
          )
            return repl.show(Table(arg));
        }
      }
    }

    return show.call(repl, arg);
  };

  repl.loadSaveOptions();
  //repl.printPromise = () => {};
  repl.run();
  return repl;
}

export function StartDebugger(args, connect, address) {
  let env = {};

  address ??= '127.0.0.1:9901';

  env['DISPLAY'] ??= ':0.0';

  if(connect) env['QUICKJS_DEBUG_ADDRESS'] = address;
  else env['QUICKJS_DEBUG_LISTEN_ADDRESS'] = address;

  const child = Spawn('qjsm', args, { block: false, env: { ...process.env, ...env }, stdio: ['pipe', 'pipe', 'pipe'] });

  if(!connect) listeners[address] = child;

  if(process.env.DEBUG) console.log('StartDebugger', { args, connect, address, env }, child);

  children.add(child.pid);

  return define(child, { args });
}

export async function ConnectDebugger(address, skipToMain = true, callback) {
  const addr = new SockAddr(AF_INET, ...address.split(':'));
  const sock = new AsyncSocket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

  const ret = await sock.connect(addr);

  if(typeof callback != 'function' && typeof callback == 'object') callback = callback.onMessage;

  if(process.env.DEBUG) console.log('ConnectDebugger', { address, skipToMain, sock });

  if(ret >= 0) {
    if(process.env.DEBUG) console.log('Connected', +sock, 'to', sock.remote);
    sockets.add(sock);
  }

  const dbg = this ?? {};

  define(dbg, {
    sock,
    addr,
    async process(callback) {
      if(process.env.DEBUG) console.log('Debugger process()', callback);

      let ret,
        lenBuf = new ArrayBuffer(9);

      try {
        while((ret = await sock.recv(lenBuf, 0, 9)) > 0) {
          let len = parseInt(toString(lenBuf, 0, ret), 16);

          let dataBuf = new ArrayBuffer(len);
          let offset = 0;

          while(offset < len) {
            ret = await sock.recv(dataBuf, offset, len - offset);

            if(ret <= 0) {
              sock.close();
              break;
            }
            offset += ret;
          }
          if(ret <= 0) break;

          let s = toString(dataBuf);
          console.log('s', s);
          let obj = JSON.parse(s);

          if(process.env.DEBUG) console.log('process() read:', obj);

          const funcName = '\x1b[38;5;208mPROCESS\x1b[0m';

          if(process.env.DEBUG) console.log(funcName + ' \x1b[38;5;196mbefore callback\x1b[0m', obj);
          let result = callback(obj);
          await result;
        }
      } catch(error) {
        console.log('Socket error:', error.message + '\n' + error.stack);
      } finally {
        sock.close();
        return ret;
      }
    },
    async sendMessage(msg) {
      if(typeof msg != 'string') msg = JSON.stringify(msg);
      const ret = sock.send(msg.length.toString(16).padStart(8, '0') + '\n' + msg);

      if(process.env.DEBUG) console.log('\x1b[38;5;33mSEND\x1b[0m[' + sock.fd + '] (' + ret + ') ' + msg);

      return ret;
    },
  });

  if(process.env.DEBUG) console.log('ConnectDebugger', console.config({ depth: 1, compact: 0 }), dbg);

  LaunchDebugger(dbg, skipToMain);

  return dbg;
}

function LaunchDebugger(dbg, skipToMain = true) {
  if(process.env.DEBUG) console.log('LaunchDebugger', console.config({ depth: 1, compact: 0 }), { dbg, skipToMain });

  if(skipToMain) {
    dbg.onstopped = once(async (...args) => {
      let st = await dispatch.stackTrace();

      script ??= st[0].filename;

      let fns = await files[script].match(/main$/gi);
      console.log('matched /main$/gi', fns /*.map(({ name }) => name)*/);

      dbg.onstopped = null;
      let resp;
      console.log('breakpoints()', console.config({ compact: 0 }), { script, fns });
      resp = await dispatch.breakpoints(script, fns);
      console.log('breakpoints() response:', console.config({ compact: 0 }), resp);

      setTimeout(async () => {
        resp = await dispatch.continue();
        console.log('continue() response:', console.config({ compact: 0 }), resp);
      }, 100);
    });
  }
  //dbg.onstopped ??= OnStopped;
  let dispatch = (dbg.dispatch = globalThis.dispatch = new DebuggerDispatcher(dbg));

  Object.assign(globalThis, bindMethods(dispatch, DebuggerDispatcher.prototype, {}));
  Object.assign(globalThis, {
    GetLoc,
    PrintStackFrame,
    PrintStack,
    async value(name) {
      let stack = await dispatch.stackTrace();

      for(let frame of stack) {
        let { local } = await dispatch.variables(frame.id, 1);

        let v = local.find(v => v.name == name);

        return v;
      }
    },
  });

  return define(dbg, { dispatch });
}

async function PrintStackFrame(frame) {
  if(frame === undefined) frame = 0;

  let { id, name, filename, line } = frame;
  let params;

  try {
    params = (await files[filename].functions).find(f => f.name == name)?.params;
  } catch(e) {}

  if(params) name += `(${params.join(', ')})`;
  let loc = line !== undefined ? new Location(filename, line) : undefined;
  let code = line !== undefined ? files[filename].line(line - 1) : undefined;
  return [`#${id}`, ` at ${name.padEnd(30)}`, loc ? ' in ' + loc : ''].concat(code ? [code] : []);
}

async function PrintStack(stack) {
  stack ??= await stackTrace();
  let frames = [];
  for(let frame of stack) {
    frames.push(await PrintStackFrame(frame));
  }
  return List(frames);
}

decorate(
  (member, obj, prop) =>
    ({
      async breakpoints(...args) {
        if(!(typeof args[0] == 'string')) args.unshift(globalThis.script);

        let [file, breakpoints] = args;

        file = absolute(file);

        if(types.isPromise(breakpoints)) breakpoints = await breakpoints;
        if(Array.isArray(breakpoints)) breakpoints = breakpoints.map(b => filterKeys(b, ['name', 'line', 'column']));

        let ret = await member.call(this, file, breakpoints);

        if(ret.path) ret.path = relative(ret.path);
        if(ret?.breakpoints?.path) ret.breakpoints.path = relative(ret.breakpoints.path);

        // console.log('breakpoints =', ret);

        return ret;
      },
      async stackTrace(frame) {
        let ret = (await member.call(this, frame)).map(frame => (typeof frame.filename == 'string' && (frame.filename = relative(absolute(frame.filename))), frame));
        console.log('stackTrace =', ret);
        return ret;
      },
      async scopes(n) {
        let stack = await this.stackTrace();
        if(n >= stack.length) return null;
        let scopes = [];

        for(let scope of await member.call(this, n)) {
          const variables = await this.variables(scope.reference);
          scope.variables = variables.length;
          scopes.push(scope);
        }

        return scopes;
      },
      async waitRun() {
        const [event, stack] = await member.call(this);
        define(globalThis, { event, stack });
        //console.log('waitRun', { event, stack });

        repl.printStatus((await PrintStackFrame(stack[0])).join(' ') + '\n');

        const { filename, line } = stack[0];

        define(globalThis, { file: filename, line });
        return [event, stack];
      },
      async variables(n, depth = 0) {
        const list = await member.call(this, n);
        const ret = [];
        const add = item => (item.variablesReference === 0 && delete item.variablesReference, ret.push(item));

        for(let item of list) {
          add(item);

          if(depth > 0) {
            if(item.variablesReference > 0) {
              let children = await this.variables(item.variablesReference, depth - 1);

              for(let child of children) {
                if(!isNaN(child.name)) child.name = '  [' + child.name + ']';
                else child.name = '  .' + child.name;

                if(child.value?.startsWith('function ')) continue;
                add(child);
              }
            }
          }
        }

        return define(ret, {
          [Symbol.for('print')]: true,
          toString() {
            return Table(this, ['name', 'value', 'type', 'variablesReference']);
          },
        });
      },
    })[prop] || member,

  DebuggerDispatcher.prototype,
);

const mkaddr = (
  (port = 8777) =>
  () =>
    `127.0.0.1:${port--}`
)();

async function NewDebugger(args, skipToMain = false, address) {
  address ??= mkaddr();

  const child = (globalThis.child = globalThis.listeners[address] || StartDebugger(args, false, address));
  let dispatch;

  globalThis.script = args[0];

  sleep(500);

  const dbg = this ?? {};

  define(dbg, {
    child,
    args,
    kill: () => (children.delete(child.pid), kill(child.pid, SIGTERM)),
  });

  await ConnectDebugger.call(dbg, address, skipToMain);

  return dbg; //dispatch;
}

async function OnStopped(msg) {
  const st = (globalThis.stack = await dispatch.stackTrace());
  let [top] = st;
  let { id, name, filename, line } = top;
  repl.printStatus(`#${id} ${name}@${filename}:${line}  ` + files[filename].line(line));
}

function URLWorker(script) {
  const dataURL = s => `data:application/javascript;charset=utf-8;base64,` + btoa(s).replaceAll('+', '-').replaceAll('/', '_').replaceAll('=', '');

  const url = dataURL(script);
  const w = new Worker(url);

  return define(new Repeater((push, stop) => (w.onmessage = push)), {
    postMessage: msg => w.postMessage(msg),
  });
}

function main(...args) {
  const base = scriptName().replace(/\.[a-z]*$/, '');

  const config = ReadJSON(`.${base}-config`) ?? {};

  globalThis.console = new Console(stderr, {
    inspectOptions: { depth: Infinity, compact: 1, maxArrayLength: Infinity, customInspect: true },
  });

  let params = getOpt(
    {
      verbose: [false, (a, v) => (v | 0) + 1, 'v'],
      listen: [false, null, 'l'],
      connect: [false, null, 'c'],
      client: [false, null, 'C'],
      server: [false, null, 'S'],
      debug: [false, null, 'x'],
      tls: [false, (v, pv, o) => ((o.tls = true), true), 't'],
      'no-tls': [false, (v, pv, o) => ((o.tls = false), true), 'T'],
      address: [true, null, 'a'],
      port: [true, null, 'p'],
      quiet: [false, null, 'q'],
      'ssl-cert': [true, null, 's'],
      'ssl-private-key': [true, null, 'k'],
      'ssl-ca': [true, null, 'A'],
      '@': 'address,port',
    },
    args,
  );
  if(params['no-tls'] === true) params.tls = false;
  const {
    address = '0.0.0.0',
    port = 8999,
    'ssl-cert': sslCert = 'localhost.crt',
    'ssl-private-key': sslPrivateKey = 'localhost.key',
    'ssl-ca': sslCA = '/etc/ssl/certs/ca-certificates.crt',
    quiet = false,
    debug = false,
    tls = true,
  } = params;

  const listen = params.connect && !params.listen ? false : true;

  let name = scriptArgs[0];
  name = name
    .replace(/.*\//, '')
    .replace(/-/g, ' ')
    .replace(/\.[^\/.]*$/, '');

  let [prefix, suffix] = name.split(' ');

  let protocol = new WeakMap();
  let ws2dbg = (globalThis.ws2dbg = mapWrapper(new WeakMap()));
  let dbg2ws = (globalThis.dbg2ws = mapWrapper(new WeakMap()));

  let sockets = (globalThis.sockets ??= new Set());
  //console.log(name, params['@']);

  function createWS(url, callbacks, listen) {
    //console.log('createWS', { url, callbacks, listen });

    setLog(
      quiet ? 0 : LLL_USER | (((debug > 1 ? LLL_INFO : LLL_WARN) << 1) - 1),
      quiet || params.debug <= 1
        ? () => {}
        : (level, str) => {
            if(/BIND_PROTOCOL|DROP_PROTOCOL|CHECK_ACCESS_RIGHTS|ADD_HEADERS/.test(str)) return;
            console.log(logLevels[level].padEnd(10), str.trim());
          },
    );

    let options;
    let dbg;
    console.log('createWS', { url });
    return createServer(
      url,
      (options = {
        tls: params.tls,
        sslCert,
        sslPrivateKey,
        sslCA,
        mimetypes: [
          ['.svgz', 'application/gzip'],
          ['.mjs', 'application/javascript'],
          ['.es', 'application/javascript'],
          ['.wasm', 'application/octet-stream'],
          ['.eot', 'application/vnd.ms-fontobject'],
          ['.lib', 'application/x-archive'],
          ['.bz2', 'application/x-bzip2'],
          ['.gitignore', 'text/plain'],
          ['.cmake', 'text/plain'],
          ['.hex', 'text/plain'],
          ['.md', 'text/plain'],
          ['.pbxproj', 'text/plain'],
          ['.wat', 'text/plain'],
          ['.c', 'text/x-c'],
          ['.h', 'text/x-c'],
          ['.cpp', 'text/x-c++'],
          ['.hpp', 'text/x-c++'],
          ['.filters', 'text/xml'],
          ['.plist', 'text/xml'],
          ['.storyboard', 'text/xml'],
          ['.vcxproj', 'text/xml'],
          ['.bat', 'text/x-msdos-batch'],
          ['.mm', 'text/x-objective-c'],
          ['.m', 'text/x-objective-c'],
          ['.sh', 'text/x-shellscript'],
        ],
        mounts: [
          ['/proxy', 'ipv4:127.0.0.1:22', null, 'proxy-ws-raw-ws'],
          ['/lws', 'https://www.google.ch/', null, ''],
          ['/', '.', 'debugger.html'],
          function* config(req, res) {
            const { body, headers } = req;
            console.log('/config', { req, res });
            console.log('*config', { body, headers });
            yield '{}';
          },
          function* files(req, res) {
            const { body, headers } = req;
            yield fs
              .readdirSync('.')
              .sort()
              .map(f => f + '\n')
              .join('');
          },
        ],
        ...url,
        ...callbacks,
        block: false,
        onConnect(ws, req) {
          console.log('onConnect', { ws, req }, req && req.headers);

          Object.defineProperties(ws, {
            sendMessage: {
              value: async function sendMessage(msg) {
                let ret = await this.send(JSON.stringify(msg));
                console.log(`ws.sendMessage(`, console.config({ compact: true }), msg, `) = ${ret}`);
                return ret;
              },
              enumerable: false,
            },
            dbg: { value: null, writable: true, enumerable: false },
          });

          sockets.add(ws);
        },
        onClose(ws) {
          console.log('onClose', { ws, dbg });
          dbg?.close();

          protocol.delete(ws);
          sockets.delete(ws);
        },
        onError(ws) {
          console.log('onError', ws);
        },
        onRequest(req, resp) {
          const { method, headers } = req;
          //console.log('\x1b[38;5;33monRequest\x1b[0m [\n  ', req, ',\n  ', resp, '\n]');
          const { body, url } = resp;

          const file = url.path.slice(1);
          const dir = file.replace(/\/[^\/]*$/g, '');
          console.log('\x1b[38;5;33monRequest\x1b[0m', { file, dir, body });

          if(file.endsWith('.js') && resp.body) {
            //console.log('onRequest', { file, dir });
            const re = /^(\s*(im|ex)port[^\n]*from ['"])([^./'"]*)(['"]\s*;[\t ]*\n?)/gm;

            resp.body = body.replaceAll(re, (match, p1, p0, p2, p3, offset) => {
              if(!/[\/\.]/.test(p2)) {
                let fname = `${p2}.js`;

                if(!existsSync(dir + '/' + fname)) return `/* ${match} */`;

                match = [p1, './' + fname, p3].join('');

                //console.log('args', { match, p1, p2, p3, offset });
              }
              return match;
            });
          }

          return resp;
        },
        onMessage(ws, data) {
          dbg = ws2dbg(ws);

          // showSessions();

          handleCommand(ws, data);

          async function handleCommand(ws, data) {
            let obj = JSON.parse(data);

            const { command, ...rest } = obj;
            // console.log('onMessage', command, rest);
            const { connect = true, address = '127.0.0.1:' + Math.round(Math.random() * (65535 - 1024)) + 1024, args = [] } = rest;

            switch (obj.type ?? command) {
              case 'start': {
                dbg = globalThis.dbg = {
                  child: StartDebugger(args, connect, address),
                  get ws() {
                    return dbg2ws(this);
                  },
                };

                ws2dbg(ws, dbg);
                dbg2ws(dbg, ws);

                const [stdin, stdout, stderr] = child.stdio;
                for(let fd of [stdout, stderr]) {
                  let flags = fcntl(fd, F_GETFL);
                  flags |= O_NONBLOCK;
                  fcntl(fd, F_SETFL, flags);
                }

                console.log('stdout flags', fcntl(stdout, F_GETFL) & O_NONBLOCK, 'O_NONBLOCK=', O_NONBLOCK);

                const forward = (fd, name) =>
                  consume(reader(fd), buf => {
                    let data = toString(buf.slice(0, r));
                    //console.log(`read(${fd}, buf) = ${r} (${quote(data, "'")})`);

                    ws.sendMessage({
                      type: 'output',
                      channel: name,
                      data,
                    });
                  });
                forward(stdout, 'stdout');
                forward(stderr, 'stderr');
                define(globalThis, { stdout, stderr, reader });

                sleep(1000);

                let tid, exited;

                tid = setInterval(() => {
                  let pid = child.wait(WNOHANG);

                  if((exited = checkChildExited(child))) {
                    ws.sendMessage({
                      type: 'error',
                      command: 'start',
                      message: `child process ${pid} ${exited}`,
                    });
                    clearInterval(tid);
                  }
                }, 1000);

                let pid = child.wait(WNOHANG);

                if((exited = checkChildExited(child))) {
                  ws.sendMessage({
                    type: 'error',
                    command: 'start',
                    message: `unable to start debugger: ${exited}`,
                  });
                  break;
                }

                const cwd = process.cwd();
                ws.sendMessage({
                  type: 'response',
                  response: {
                    command: 'start',
                    args,
                    cwd,
                    address,
                  },
                });

                break;
              }

              case 'connect': {
                dbg = globalThis.dbg = await ConnectDebugger.call(globalThis.dbg, address, false, (dbg, sock) => {
                  console.log('wait(WNOHANG) =', child.wait(WNOHANG));
                  console.log('child', child);
                });

                ws2dbg(ws, dbg);
                dbg2ws(dbg, ws);

                console.log('connect command', { ws, dbg });
                sockets.add(dbg.sock);

                const cwd = process.cwd();
                let connected;

                console.log('dbg', dbg);
                break;
              }

              case 'file': {
                const { path } = rest;
                const data = ReadFile(path, 'utf-8');
                //ws.send(JSON.stringify({ type: 'response', response: { command: 'file', path, data } }));

                const lexer = new Lexer(data, path);
                console.log('lexer', lexer);
                const lines = [];

                for(;;) {
                  const { pos, size } = lexer;
                  console.log('lexer', { pos, size });
                  let result = lexer.next();
                  if(result.done) break;
                  const token = result.value;
                  console.log('token', {
                    lexeme: token.lexeme,
                    id: token.id,
                    loc: token.loc + '',
                  });
                  const { type, id, lexeme, loc } = token;
                  const { line, column, file } = loc;
                  //console.log('token', {lexeme,id,line});

                  if(!lines[line - 1]) lines.push([]);
                  let a = lines[line - 1];
                  a.push([lexeme, id]);
                }
                console.log('lines', lines);
                break;
              }

              case 'breakpoints': {
                const { breakpoints } = obj;

                let response = await dbg.dispatch.breakpoints(...breakpoints);

                console.log('breakpoints', { breakpoints, response });

                ws.sendMessage(response);

                break;
              }

              case 'request': {
                const { request } = obj;
                const { request_seq, command, args } = request;

                let response = await dbg.dispatch.sendRequest(command, args, request_seq);

                if(command == 'stackTrace') {
                  response.body = response.body.map(frame => {
                    if(frame.filename) frame.filename = path.relative(frame.filename);

                    return frame;
                  });
                }
                console.log('Request', { request, response });

                ws.sendMessage(response);

                break;
              }

              default: {
                console.log('onMessage(x)', obj);
                const dbg = ws2dbg(ws);
                const { pid } = dbg.child;
                console.log('send to debugger', { pid, obj });

                dbg.sendMessage(obj);

                //DebuggerProtocol.send(dbg, data);
                break;
              }
            }
          }
        },
        ...(url && url.host ? url : {}),
      }),
    );
  }

  console.log('XX');

  delete globalThis.DEBUG;

  let inputBuf = new ArrayBuffer(10);
  ttySetRaw(0);

  setReadHandler(0, () => {
    let r = readSync(0, inputBuf, 0, inputBuf.byteLength);

    if(r > 0) {
      let a = new Uint8Array(inputBuf.slice(0, r));

      //console.log('a', a);

      for(let i = 0; i < a.length; i++) if(a[i] == 13) a[i] = 10;

      if(a.length == 1 && a[0] == 127) a = new Uint8Array([8, 0x20, 8]);

      if(a.length == 1 && a[0] == 27) showSessions();
      else writeSync(1, a.buffer);
    }
  });

  function showSessions() {
    let sessions = getSessions();
    console.log('sessions', console.config({ maxArrayLength: Infinity, depth: 4, customInspect: true, compact: 0 }), sessions);
  }

  //setInterval(() => console.log('interval'), 5000);

  globalThis.server = createWS(`wss://${address}:8998/ws`, {}, true);

  define(globalThis, {
    get connections() {
      return [...globalThis.sockets].map(ws => ws2dbg(ws));
    },
    get socklist() {
      return [...globalThis.sockets];
    },
    net: { setLog, LLL_USER, LLL_NOTICE, LLL_WARN, createServer },
    TrivialSyntaxHighlighter,
    NewDebugger,
    LaunchDebugger,
    StartDebugger,
    ConnectDebugger,
    DebuggerDispatcher,
    DebuggerProtocol,
    GetFunctionName,
    FindFunctions,
    LoadAST,
    Table,
    List,
    get file() {
      return this.files[this.script];
    },
    files: propertyLookup(
      (globalThis.fileCache = {}),
      memoize((file, source) => {
        source ??= tryCatch(
          () => TrivialSyntaxHighlighter(ReadFile(file)),
          s => s,
          () => ReadFile(file),
        );
        return define(
          {
            source,
            indexlist: [...source.matchAll(/^[^\n]*/gm)].map(m => m.index),
          },
          lazyProperties(
            {
              line(i, j) {
                if(i === undefined) return '';
                const { source, indexlist } = this;
                j ??= i + 1;
                const m = mod(indexlist.length - 1);
                const [start, end] = [indexlist[m(i)], indexlist[m(j)]];
                let line = source.slice(start, (end ?? 0) - 1);

                if([...line.matchAll(/\x1b([^A-Za-z]*[A-Za-z])/g)].last != '\x1b[0m') line += '\x1b[0m';

                return line;
              },
              match(re) {
                if(typeof re == 'string') re = new RegExp(re, 'gi');

                return this.functions.then(fns =>
                  define(
                    fns.filter(({ name }) => re.test(name)),
                    { [Symbol.toStringTag]: 'FunctionList', file },
                  ),
                );
              },
            },
            {
              // estree: () => ,
              async functions() {
                return (globalThis.functionCache = [...FindFunctions((globalThis.ast = await LoadAST(file)))].map(([name, loc, params, expression, path]) =>
                  define(
                    {
                      name,
                      params,
                      ...loc,
                      expression,
                    },
                    { path },
                  ),
                ));
              },
            },
            { async: false },
          ),
        );
      }),
    ),
    async repeat(cond, fn, ...args) {
      let r;
      if(typeof cond == 'number') {
        let n = cond;
        cond = (r, i) => i >= n || r === true;
      }
      for(let i = 0; ; i++) {
        r = await fn(...args);

        if(cond(r, i)) break;
      }
      return r;
    },
    repl: StartREPL(),
  });

  function quit(why) {
    console.log(`quit('${why}')`);

    let cfg = { inspectOptions: console.options };
    WriteJSON(`.${base}-config`, cfg);
    // repl.cleanup(why);
  }
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error?.message ?? error}\n${error?.stack}`);
} finally {
  //console.log('SUCCESS');
}