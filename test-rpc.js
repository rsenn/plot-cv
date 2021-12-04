import * as std from 'std';
import * as os from 'os';
import * as deep from './lib/deep.js';
import   path from 'path';
import Util from './lib/util.js';
import { Console } from 'console';
import REPL from './quickjs/qjs-modules/lib/repl.js';
import inspect from './lib/objectInspect.js';
import * as Terminal from './terminal.js';
import * as fs from './lib/filesystem.js';
import * as net from 'net';
import { Socket } from './quickjs/qjs-ffi/lib/socket.js';
import { EventEmitter } from './lib/events.js';
import { Repeater } from './lib/repeater/repeater.js';

import rpc from './quickjs/qjs-net/rpc.js';
import * as rpc2 from './quickjs/qjs-net/rpc.js';

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
  globalThis.console = new Console({ inspectOptions: { compact: 2, customInspect: true } });
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
  console.log('params', params);
  const { address = '0.0.0.0', port = 8999, 'ssl-cert': sslCert = 'localhost.crt', 'ssl-private-key': sslPrivateKey = 'localhost.key' } = params;
  const listen = params.connect && !params.listen ? false : true;
  const server = !params.client || params.server;
  Object.assign(globalThis, { ...rpc2, rpc });
  let name = Util.getArgs()[0];
  name = name
    .replace(/.*\//, '')
    .replace(/-/g, ' ')
    .replace(/\.[^\/.]*$/, '');

  let [prefix, suffix] = name.split(' ');

  let repl = new REPL(`\x1b[38;5;165m${prefix} \x1b[38;5;39m${suffix}\x1b[0m`, fs, false);

  repl.historyLoad(null, false);

  repl.help = () => {};
  let { log } = console;
  repl.show = arg => std.puts((typeof arg == 'string' ? arg : inspect(arg, globalThis.console.options)) + '\n');

  repl.cleanup = () => {
    repl.readlineRemovePrompt();
    Terminal.mousetrackingDisable();
    let numLines = repl.historySave();

    repl.printStatus(`EXIT (wrote ${numLines} history entries)`, false);

    std.exit(0);
  };

  console.log = repl.printFunction(log);

  let cli = (globalThis.sock = new rpc.Socket(`${address}:${port}`, rpc[`RPC${server ? 'Server' : 'Client'}Connection`], +params.verbose));

  cli.register({ Socket, Worker: os.Worker, Repeater, REPL, EventEmitter });

  let connections = new Set();
  const createWS = (globalThis.createWS = (url, callbacks, listen) => {
    console.log('createWS', { url, callbacks, listen });

    net.setLog((params.debug ? net.LLL_USER : 0) | (((params.debug ? net.LLL_NOTICE : net.LLL_WARN) << 1) - 1), (level, ...args) => {
      repl.printStatus(...args);
      if(params.debug) console.log((['ERR', 'WARN', 'NOTICE', 'INFO', 'DEBUG', 'PARSER', 'HEADER', 'EXT', 'CLIENT', 'LATENCY', 'MINNET', 'THREAD'][Math.log2(level)] ?? level + '').padEnd(8), ...args);
    });

    return [net.client, net.server][+listen]({
      tls: params.tls,
      sslCert,
      sslPrivateKey,
      mounts: [
        ['/', '.', 'debugger.html'],
        function proxy(req, res) {
          const { url, method, headers } = req;
          const { status, ok, type } = res;

          console.log('proxy', { url, method, headers }, { status, ok, url, type });
        },

        function* config(req, res) {
          console.log('/config', { req, res });
          yield '{}';
        },
        function* files(req, resp) {
          const { body, headers } = req;
          const { 'content-type': content_type } = headers;
          const data = JSON.parse(body);

          resp.type = 'application/json';

          //console.log('\x1b[38;5;215m*files\x1b[0m', { headers, data, req, resp });
          let { dir = 'tmp', filter = '.(brd|sch|G[A-Z][A-Z])$', verbose = false, objects = false, key = 'mtime' } = data;

          if(fs.existsSync(dir)) dir = path.realpath(dir);
          
          console.log('\x1b[38;5;215m*files\x1b[0m', { dir });

          let names = fs.readdirSync(dir);
          //console.log('\x1b[38;5;215m*files\x1b[0m', { dir, names });
          if(filter) {
            const re = new RegExp(filter, 'gi');
            names = names.filter(name => re.test(name));
          }

          let entries = names.map(file => [file, fs.statSync(`${dir}/${file}`)]);

          entries = entries.reduce((acc, [file, st]) => {
            let obj = {
              name: file
            };

            acc.push([
              file + (st.isDirectory() ? '/' : ''),
              Object.assign(obj, {
                mtime: Util.toUnixTime(st.mtime),
                time: Util.toUnixTime(st.ctime),
                mode: `0${(st.mode & 0x09ff).toString(8)}`,
                size: st.size
              })
            ]);
            return acc;
          }, []);

          let cmp = {
            string(a, b) {
              return b[1][key].localeCompare(a[1][key]);
            },
            number(a, b) {
              return b[1][key] - a[1][key];
            }
          }[typeof entries[0][1][key]];

          entries = entries.sort(cmp);

          console.log('\x1b[38;5;215m*files\x1b[0m', { entries });
          names = entries.map(([name, obj]) => (objects ? obj : name));

          yield JSON.stringify(...[names, ...(verbose ? [null, 2] : [])]);
        }
      ],
      ...url,

      ...callbacks,
      onConnect(ws, req) {
        console.log('test-rpc', { ws, req });
        connections.add(ws);

        return callbacks.onConnect(ws, req);
      },
      onClose(ws) {
        connections.delete(ws);

        return callbacks.onClose(ws, req);
      },
      onHttp(req, rsp) {
        const { url, method, headers } = req;
        console.log('\x1b[38;5;33monHttp\x1b[0m [\n  ', req, ',\n  ', rsp, '\n]');
        return rsp;
      },
      onMessage(ws, data) {
        console.log('onMessage', ws, data);
        return callbacks.onMessage(ws, data);
      },
      onFd(fd, rd, wr) {
        //console.log('onFd',{fd,rd,wr});
        return callbacks.onFd(fd, rd, wr);
      },
      ...(url && url.host ? url : {})
    });
  });
  globalThis[['connection', 'listener'][+listen]] = cli;

  define(globalThis, {
    get connections() {
      return [...connections];
    }
  });

  Object.assign(globalThis, {
    repl,
    Util,
    ...rpc,
    quit,
    exit: quit,
    Socket,
    cli,
    net,
    std,
    os,
    deep,
    fs,
    path,
    ReadJSON,
    WriteFile,
    WriteJSON
  });

  define(globalThis, listen ? { server: cli, cli } : { client: cli, cli });
  delete globalThis.DEBUG;
  Object.defineProperty(globalThis, 'DEBUG', { get: DebugFlags });

  if(listen) cli.listen(createWS, os);
  else cli.connect(createWS, os);

  function quit(why) {
    console.log(`quit('${why}')`);

    let cfg = { inspectOptions: console.options };
    WriteJSON(`.${base}-config`, cfg);
    repl.cleanup(why);
  }

  repl.runSync();
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
