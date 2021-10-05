import * as std from 'std';
import * as os from 'os';
import * as deep from './lib/deep.js';
import * as path from './lib/path.js';
import Util from './lib/util.js';
import { toArrayBuffer, toString, escape, quote, define } from './lib/misc.js';
import { Console } from 'console';
import REPL from './quickjs/qjs-modules/lib/repl.js';
import inspect from './lib/objectInspect.js';
import * as Terminal from './terminal.js';
import * as fs from './lib/filesystem.js';
import * as net from 'net';
import { DebuggerProtocol } from './debuggerprotocol.js';
import { StartDebugger, ConnectDebugger } from './debugger.js';
import Lexer from './quickjs/qjs-modules/lib/jslexer.js';

globalThis.fs = fs;

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

function main(...args) {
  const base = path.basename(Util.getArgv()[1], '.js').replace(/\.[a-z]*$/, '');

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
  const { address = '0.0.0.0', port = 8999, 'ssl-cert': sslCert = 'localhost.crt', 'ssl-private-key': sslPrivateKey = 'localhost.key' } = params;
  const listen = params.connect && !params.listen ? false : true;
  const server = !params.client || params.server;
  let name = Util.getArgs()[0];
  name = name
    .replace(/.*\//, '')
    .replace(/-/g, ' ')
    .replace(/\.[^\/.]*$/, '');

  let [prefix, suffix] = name.split(' ');
  /*
  let repl = new REPL(`\x1b[38;5;165m${prefix} \x1b[38;5;39m${suffix}\x1b[0m`, fs, false);

  repl.historyLoad(null, false);

  repl.help = () => {};
  let { log } = console;
  repl.show = arg =>
    std.puts((typeof arg == 'string' ? arg : inspect(arg, globalThis.console.options)) + '\n');

  repl.cleanup = () => {
    repl.readlineRemovePrompt();
    Terminal.mousetrackingDisable();
    let numLines = repl.historySave();

    repl.printStatus(`EXIT (wrote ${numLines} history entries)`, false);

    std.exit(0);
  };

  console.log = repl.printFunction(log);
 
*/
  let protocol = new WeakMap();
  let connections = new Set();
  const createWS = (globalThis.createWS = (url, callbacks, listen) => {
    console.log('createWS', { url, callbacks, listen });

    net.setLog((params.debug ? net.LLL_USER : 0) | (((params.debug ? net.LLL_NOTICE : net.LLL_WARN) << 1) - 1), (level, ...args) => {
      console.log(...args);
      if(params.debug) console.log((['ERR', 'WARN', 'NOTICE', 'INFO', 'DEBUG', 'PARSER', 'HEADER', 'EXT', 'CLIENT', 'LATENCY', 'MINNET', 'THREAD'][Math.log2(level)] ?? level + '').padEnd(8), ...args);
    });

    let child, dbg;

    return [net.client, net.server][+listen]({
      tls: params.tls,
      sslCert,
      sslPrivateKey,
      mounts: [['/', '.', 'debugger.html']],
      ...url,

      ...callbacks,
      onConnect(ws, req) {
        console.log('debugger-server', { ws, req });

        connections.add(ws);
      },
      onClose(ws) {
        protocol.delete(ws);
        connections.delete(ws);
      },
      onHttp(req, rsp) {
        const { url, method, headers } = req;
        console.log('\x1b[38;5;33monHttp\x1b[0m [\n  ', req, ',\n  ', rsp, '\n]');
        return rsp;
      },
      onMessage(ws, data) {
        console.log('onMessage', ws, data);
        let obj = JSON.parse(data);

        const { command, ...rest } = obj;
        console.log('onMessage', command, rest);

        switch (command) {
          case 'start': {
            const { start } = rest;
            const { connect = true, address = '127.0.0.1:' + Math.round(Math.random() * (65535 - 1024)) + 1024, args = [] } = start;
            child = StartDebugger(args, connect, address);
            console.log('child', child.pid);
            os.sleep(1000);
            dbg = ConnectDebugger(address, (dbg, sock) => {
              console.log('wait() =', child.wait());
              console.log('child', child);
            });
            os.setWriteHandler(+dbg, async () => {
              os.setWriteHandler(+dbg, null);
              console.log('connected', dbg);
              const cwd = process.cwd();
              ws.send(JSON.stringify({ type: 'response', response: { command: 'start', args, cwd } }));

              let msg;

              while(dbg.open) {
                try {
                  msg = await DebuggerProtocol.read(dbg);
                  console.log('DebuggerProtocol.read() =', escape(msg));
                  if(typeof msg == 'string') {
                    ws.send(msg);
                  } else {
                    console.log('sock', dbg);
                  }
                } catch(error) {
                  const { message, stack } = error;
                  ws.send(JSON.stringify({ type: 'error', error: { message, stack } }));
                  dbg.close();
                  break;
                }
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
              console.log('token', { lexeme: token.lexeme, id: token.id, loc: token.loc + '' });
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

        /*           let p = new DebuggerProtocol();
        protocol.set(ws, p);*/
      },
      onFd(fd, rd, wr) {
        os.setReadHandler(fd, rd);
        os.setWriteHandler(fd, wr);

        //  console.log('onFd', { fd, rd, wr });
      },
      ...(url && url.host ? url : {})
    });
  });

  define(globalThis, {
    get connections() {
      return [...connections];
    }
  });

  delete globalThis.DEBUG;

  createWS(Util.parseURL('wss://127.0.0.1:9000/ws'), {}, true);
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

  // repl.runSync();
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
