import * as std from 'std';
import * as os from 'os';
import * as deep from './lib/deep.js';
import * as path from './lib/path.js';
import Util from './lib/util.js';
import { daemon, atexit, getpid, toArrayBuffer, toString, escape, quote, define, extendArray } from 'util';
import { Console } from 'console';
import REPL from './quickjs/qjs-modules/lib/repl.js';
import inspect from './lib/objectInspect.js';
import * as Terminal from './terminal.js';
import * as fs from 'fs';
import * as net from 'net';
import { DebuggerProtocol } from './debuggerprotocol.js';
import { StartDebugger, ConnectDebugger } from './debugger.js';
import { fcntl, F_GETFL, F_SETFL, O_NONBLOCK } from './quickjs/qjs-ffi/lib/fcntl.js';

globalThis.fs = fs;

extendArray();
const scriptName = () => scriptArgs[0].replace(/.*\//g, '').replace(/\.js$/, '');

atexit(() => {
   console.log('atexit', atexit);
 let { stack } = new Error('');
  console.log('stack:', stack);
});

function ReadJSON(filename) {
  let data = fs.readFileSync(filename, 'utf-8');

  if(data) console.debug(`${data.length} bytes read from '${filename}'`);
  return data ? JSON.parse(data) : null;
}

function WriteFile(name, data, verbose = true) {
  if(Util.isGenerator(data)) {
    let fd = fs.openSync(name, os.O_WRONLY | os.O_TRUNC | os.O_CREAT, 0x1a4);
    let r = 0;
    for(let item of data) {
      r += fs.writeSync(fd, toArrayBuffer(item + ''));
    }
    fs.closeSync(fd);
    let stat = fs.statSync(name);
    return stat?.size;
  }
  if(Util.isIterator(data)) data = [...data];
  if(Util.isArray(data)) data = data.join('\n');

  if(typeof data == 'string' && !data.endsWith('\n')) data += '\n';
  let ret = fs.writeFileSync(name, data);

  if(verbose) console.log(`Wrote ${name}: ${ret} bytes`);
}

function WriteJSON(name, data) {
  WriteFile(name, JSON.stringify(data, null, 2));
}

function StartREPL(prefix = scriptName(), suffix = '') {
  let repl = new REPL(`\x1b[38;5;165m${prefix} \x1b[38;5;39m${suffix}\x1b[0m`, fs, false);

  repl.historyLoad(null, fs);
  repl.inspectOptions = { ...console.options, compact: 2 };

   let { log } = console;
  //repl.show = arg => std.puts(arg);

  /* repl.cleanup = () => {
    repl.readlineRemovePrompt();
    Terminal.mousetrackingDisable();
    let numLines = repl.historySave();

    repl.printStatus(`EXIT (wrote ${numLines} history entries)`, false);

    std.exit(0);
  };*/
  repl.directives.i = [
    name =>
      import(name)
        .then(m => (globalThis[name.replace(/(.*\/|\.[^\/.]+$)/g, '')] = m))
        .catch(() => repl.printStatus(`ERROR: module '${name}' not found`)),
    'import a module'
  ];
  repl.directives.d = [() => globalThis.daemon(), 'detach'];

  console.log = repl.printFunction((...args) => {
    log(console.config(repl.inspectOptions), ...args);
  });

  repl.run();
  return repl;
}

function main(...args) {
  const base = scriptName().replace(/\.[a-z]*$/, '');

  const config = ReadJSON(`.${base}-config`) ?? {};
  globalThis.console = new Console(std.err, {
    inspectOptions: { compact: 2, customInspect: true }
  });
  let params = Util.getOpt(
    {
      verbose: [false, (a, v) => (v | 0) + 1, 'v'],
      listen: [false, null, 'l'],
      connect: [false, null, 'c'],
      client: [false, null, 'C'],
      server: [false, null, 'S'],
      debug: [false, null, 'x'],
      tls: [false, null, 't'],
      'no-tls': [false, (v, pv, o) => ((o.tls = false), true), 'T'],
      address: [true, null, 'a'],
      port: [true, null, 'p'],
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
    'ssl-private-key': sslPrivateKey = 'localhost.key'
  } = params;
  const listen = params.connect && !params.listen ? false : true;
  const server = !params.client || params.server;
  let name = Util.getArgs()[0];
  name = name
    .replace(/.*\//, '')
    .replace(/-/g, ' ')
    .replace(/\.[^\/.]*$/, '');

  let [prefix, suffix] = name.split(' ');

  let protocol = new WeakMap();
  let sockets = (globalThis.sockets ??= new Set());
  const createWS = (globalThis.createWS = (url, callbacks, listen) => {
    console.log('createWS', { url, callbacks, listen });

    net.setLog(
      (params.debug ? net.LLL_USER : 0) | (((params.debug ? net.LLL_NOTICE : net.LLL_WARN) << 1) - 1),
      (level, ...args) => {
        console.log(...args);
        if(params.debug)
          console.log(
            (
              [
                'ERR',
                'WARN',
                'NOTICE',
                'INFO',
                'DEBUG',
                'PARSER',
                'HEADER',
                'EXT',
                'CLIENT',
                'LATENCY',
                'MINNET',
                'THREAD'
              ][Math.log2(level)] ?? level + ''
            ).padEnd(8),
            ...args
          );
      }
    );

    let options;
    let child, dbg;

    return [net.client, net.server][+listen](
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
        mounts: [['/', '.', 'debugger.html']],
        ...url,

        ...callbacks,
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
                child = ws.child = StartDebugger(args, connect, address);
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
          os.setReadHandler(fd, rd);
          os.setWriteHandler(fd, wr);

          //  console.log('onFd', { fd, rd, wr });
        },
        ...(url && url.host ? url : {})
      })
    );
  });

  define(globalThis, {
    get connections() {
      return [...globalThis.sockets];
    },
    get socklist() {
      return [...globalThis.sockets];
    },
    net,
    StartDebugger,
    ConnectDebugger,
    DebuggerProtocol,
    repl: StartREPL(),
    daemon() {
      repl.stop();
      std.puts('\ndetaching...');
      daemon(1, 0);
      std.puts(' PID '+getpid()+'\n');
    }
  });

  delete globalThis.DEBUG;

  globalThis.ws = createWS(Util.parseURL(`wss://${address}:9000/ws`), {}, true);
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
  1;
  std.exit(1);
} finally {
  //console.log('SUCCESS');
}
