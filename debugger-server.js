import * as std from 'std';
import * as os from 'os';
import {setInterval} from 'timers';
import * as deep from './lib/deep.js';
import * as path from './lib/path.js';
import Util from './lib/util.js';
import { daemon, atexit, getpid, toArrayBuffer, toString, escape, quote, define, extendArray, getOpt } from 'util';
import { Console } from './quickjs/qjs-modules/lib/console.js';
import REPL from './quickjs/qjs-modules/lib/repl.js';
import inspect from './lib/objectInspect.js';
import * as Terminal from './terminal.js';
import * as fs from 'fs';
import { setLog, logLevels, getSessions, LLL_USER, LLL_NOTICE, LLL_WARN, client, server } from 'net';
import { DebuggerProtocol } from './debuggerprotocol.js';
import { StartDebugger, ConnectDebugger } from './debugger.js';
import { fcntl, F_GETFL, F_SETFL, O_NONBLOCK } from './quickjs/qjs-ffi/lib/fcntl.js';
import { IfDebug, LogIfDebug, ReadFile, LoadHistory, ReadJSON, ReadXML, MapFile, WriteFile, WriteJSON, WriteXML, ReadBJSON, WriteBJSON, DirIterator, RecursiveDirIterator, ReadDirRecursive, Filter, FilterImages, SortFiles, StatFiles, ReadFd, FdReader, CopyToClipboard, ReadCallback, LogCall, Spawn, FetchURL } from './io-helpers.js';

extendArray(Array.prototype);

const scriptName = (arg = scriptArgs[0]) => path.basename(arg, path.extname(arg));

atexit(() => {
  console.log('atexit', atexit);
  let stack = new Error('').stack;
  console.log('stack:', stack);
});

function StartREPL(prefix = scriptName(), suffix = '') {
  let repl = new REPL(`\x1b[38;5;165m${prefix} \x1b[38;5;39m${suffix}\x1b[0m`, false);
  repl.historyLoad(null, fs);
  repl.inspectOptions = { ...console.options, maxArrayLength: Infinity, compact: 2 };
  let { log } = console;

  repl.directives.i = [
    name =>
      import(name)
        .then(m => (globalThis[name.replace(/(.*\/|\.[^\/.]+$)/g, '')] = m))
        .catch(() => repl.printStatus(`ERROR: module '${name}' not found`)),
    'import a module'
  ];
  repl.directives.d = [() => globalThis.daemon(), 'detach'];
  console.log = repl.printFunction((...args) => {
    log('LOG', console.config(repl.inspectOptions), ...args);
  });
  repl.run();
  return repl;
}

function main(...args) {
  const base = scriptName().replace(/\.[a-z]*$/, '');

  const config = ReadJSON(`.${base}-config`) ?? {};

  globalThis.console = new Console(std.err, {
    inspectOptions: { compact: 2, maxArrayLength: Infinity, customInspect: true }
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
      'ssl-cert': [true, null],
      'ssl-private-key': [true, null],
      '@': 'address,port'
    },
    args
  );
  if(params['no-tls'] === true) params.tls = false;
  const {
    address = '0.0.0.0',
    port = 8999,
    'ssl-cert': sslCert = 'localhost.crt',
    'ssl-private-key': sslPrivateKey = 'localhost.key',
    quiet = false,
    debug = false,
    tls = true
  } = params;
  const listen = params.connect && !params.listen ? false : true;
  //const server = !params.client || params.server;
  let name = scriptArgs[0];
  name = name
    .replace(/.*\//, '')
    .replace(/-/g, ' ')
    .replace(/\.[^\/.]*$/, '');

  let [prefix, suffix] = name.split(' ');

  let protocol = new WeakMap();

  let sockets = (globalThis.sockets ??= new Set());

  const createWS = (globalThis.createWS = (url, callbacks, listen) => {
    console.log('createWS', { url, callbacks, listen });

    setLog(
      quiet ? 0 : (debug ? LLL_USER : 0) | (((debug ? LLL_NOTICE : LLL_WARN) << 1) - 1),
      quiet
        ? () => {}
        : (level, str) => {
            //if(level != LLL_USER && str.indexOf('\x1b') == -1 && /(lws_|^\s\+\+|^(\s+[a-z0-9_]+\s=|\s*[a-z0-9_]+:)\s)/.test(str)) return;
            //if(/WSI_(DESTROY|CREATE)|FILTER_NETWORK_CONNECTION/.test(str)) return;
            if(/BIND_PROTOCOL|DROP_PROTOCOL|CHECK_ACCESS_RIGHTS|ADD_HEADERS/.test(str)) return;
            if(debug) console.log(logLevels[level].padEnd(10), str.trim());
          }
    );

    let options;
    let child, dbg;
    let netfn = [client, server][+listen];
    console.log('createWS', { url, netfn });
    return netfn(
      url,
      (options = {
        tls: params.tls,
        sslCert,
        sslPrivateKey,
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
          ['.sh', 'text/x-shellscript']
        ],
        mounts: [
          ['/proxy', 'ipv4:127.0.0.1', null, 'proxy-ws-raw-ws'],
          ['/', '.', 'debugger.html']
        ],
        ...url,
        ...callbacks,
        block: false,
        onConnect(ws, req) {
          console.log('debugger-server', { ws, req });

          ws.sendMessage = function(msg) {
            let ret = this.send(JSON.stringify(msg));
            console.log(`ws.sendMessage(`, msg, `) = ${ret}`);
            return ret;
          };

          sockets.add(ws);
        },
        onClose(ws) {
          console.log('onClose', ws);
          dbg.close();

          protocol.delete(ws);
          sockets.delete(ws);
        },
        onError(ws) {
          console.log('onError', ws);
        },
        onHttp(req, rsp) {
          const { url, method, headers } = req;
          console.log('\x1b[38;5;33monHttp\x1b[0m [\n  ', req, ',\n  ', rsp, '\n]');
          return rsp;
        },
        onMessage(ws, data) {
          console.log('onMessage', ws, data);
          showSessions();

          handleCommand(ws, data);

          function handleCommand(ws, data) {
            let obj = JSON.parse(data);

            const { command, ...rest } = obj;
            // console.log('onMessage', command, rest);
            const {
              connect = true,
              address = '127.0.0.1:' + Math.round(Math.random() * (65535 - 1024)) + 1024,
              args = []
            } = rest;

            switch (command) {
              case 'start': {
                console.log('ws', ws);
                child = /*ws.child = */ StartDebugger(args, connect, address);
                const [, stdout, stderr] = child.stdio;
                for(let fd of [stdout, stderr]) {
                  //console.log(`fcntl(${fd}, F_GETFL)`);
                  let flags = fcntl(fd, F_GETFL);
                  //console.log(`fcntl(${fd}, F_SETFL, 0x${flags.toString(16)})`);
                  flags |= O_NONBLOCK;
                  fcntl(fd, F_SETFL, flags);
                }
                for(let i = 1; i <= 2; i++) {
                  let fd = child.stdio[i];
                  console.log('os.setReadHandler', fd);
                  os.setReadHandler(fd, () => {
                    let buf = new ArrayBuffer(1024);
                    let r = os.read(fd, buf, 0, buf.byteLength);

                    if(r > 0) {
                      let data = toString(buf.slice(0, r));
                      console.log(`read(${fd}, buf) = ${r} (${quote(data, "'")})`);

                      ws.sendMessage({
                        type: 'output',
                        channel: ['stdout', 'stderr'][i - 1],
                        data
                      });
                    }
                  });
                }
                console.log('child', child.pid);

                os.sleep(1000);
              }
              case 'connect': {
                dbg = ws.dbg = ConnectDebugger(address, (dbg, sock) => {
                  console.log('wait() =', child.wait());
                  console.log('child', child);
                });
                os.setWriteHandler(+dbg, async () => {
                  os.setWriteHandler(+dbg, null);
                  console.log(`connected to ${address}`, dbg);

                  sockets.add(dbg);

                  const cwd = process.cwd();
                  ws.sendMessage({
                    type: 'response',
                    response: {
                      command: 'start',
                      args,
                      cwd,
                      address
                    }
                  });

                  let msg;

                  while(dbg.open) {
                    try {
                      msg = await DebuggerProtocol.read(dbg);
                      console.log('DebuggerProtocol.read() =', escape(msg));
                      if(typeof msg == 'string') {
                        let ret;
                        ret = ws.send(msg);
                        console.log(`ws.send(${quote(msg, "'")}) = ${ret}`);
                      } else {
                        console.log('closed socket', dbg);
                        sockets.delete(dbg);
                        ws.sendMessage({
                          type: 'end',
                          reason: 'closed'
                        });
                      }
                    } catch(error) {
                      const { message, stack } = error;
                      ws.sendMessage({
                        type: 'error',
                        error: { message, stack }
                      });
                      dbg.close();
                      break;
                    }
                    if(msg === null) break;
                  }
                });
                console.log('dbg', dbg);
                break;
              }
              case 'file': {
                const { path } = rest;
                const data = fs.readFileSync(path, 'utf-8');
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
                    loc: token.loc + ''
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
              default: {
                console.log('send to debugger', data);
                DebuggerProtocol.send(dbg, data);
                break;
              }
            }
          }
          /*let p = new DebuggerProtocol();
        protocol.set(ws, p);*/
        },
        onFd(fd, rd, wr) {
          //  console.log('onFd', { fd, rd, wr });
          os.setReadHandler(fd, rd);
          os.setWriteHandler(fd, wr);
        },
        ...(url && url.host ? url : {})
      })
    );
  });
  console.log('XX');

  /*  define(globalThis, {
    get connections() {
      return [...globalThis.sockets];
    },
    get socklist() {
      return [...globalThis.sockets];
    },
    net: { setLog, LLL_USER, LLL_NOTICE, LLL_WARN, client, server },
    StartDebugger,
    ConnectDebugger,
    DebuggerProtocol,
    repl: StartREPL(),
    daemon() {
      repl.stop();
      std.puts('\ndetaching...');
      daemon(1, 0);
      std.puts(' PID ' + getpid() + '\n');
    }
  });
*/
  delete globalThis.DEBUG;

  let inputBuf = new ArrayBuffer(10);
  os.ttySetRaw(0);

  os.setReadHandler(0, () => {
    let r = fs.readSync(0, inputBuf, 0, inputBuf.byteLength);

    if(r > 0) {
      let a = new Uint8Array(inputBuf.slice(0, r));

      //console.log('a', a);

      for(let i = 0; i < a.length; i++) if(a[i] == 13) a[i] = 10;

      if(a.length == 1 && a[0] == 127) a = new Uint8Array([8, 0x20, 8]);

      if(a.length == 1 && a[0] == 27) showSessions();
      else fs.writeSync(1, a.buffer);
    }
  });

  function showSessions() {
    let sessions = getSessions();
    console.log(
      'sessions',
      console.config({ maxArrayLength: Infinity, depth: 4, customInspect: true, compact: 1 }),
      sessions
    );
  }

//setInterval(() => console.log('interval'), 5000);

  globalThis.ws = createWS(`wss://${address}:9000/ws`, {}, true);
  //  Object.defineProperty(globalThis, 'DEBUG', { get: DebugFlags });

  /* if(listen) cli.listen(createWS, os);
  else cli.connect(createWS, os);
*/
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
