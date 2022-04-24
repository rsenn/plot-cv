import * as std from 'std';
import * as os from 'os';
import { setInterval } from 'timers';
import * as deep from './lib/deep.js';
import * as path from './lib/path.js';
import Util from './lib/util.js';
import { watch, IN_MODIFY, memoize, daemon, atexit, getpid, toArrayBuffer, toString, escape, quote, define, extendArray, getOpt, glob } from 'util';
import { Console } from './quickjs/qjs-modules/lib/console.js';
import REPL from './quickjs/qjs-modules/lib/repl.js';
import inspect from './lib/objectInspect.js';
import * as Terminal from './terminal.js';
import * as fs from 'fs';
import { setLog, logLevels, getSessions, LLL_USER, LLL_INFO, LLL_NOTICE, LLL_WARN, client, server } from 'net';
import { DebuggerProtocol } from './debuggerprotocol.js';
import { StartDebugger, ConnectDebugger } from './debugger.js';
import { fcntl, F_GETFL, F_SETFL, O_NONBLOCK } from './quickjs/qjs-ffi/lib/fcntl.js';
import { IfDebug, LogIfDebug, ReadFile, LoadHistory, ReadJSON, ReadXML, MapFile, WriteFile, WriteJSON, WriteXML, ReadBJSON, WriteBJSON, DirIterator, RecursiveDirIterator, ReadDirRecursive, Filter, FilterImages, SortFiles, StatFiles, ReadFd, FdReader, CopyToClipboard, ReadCallback, LogCall, Spawn, FetchURL } from './io-helpers.js';
import { quarterDay, Time, TimeToStr, FilenameToTime, NextFile, DailyPhase, PhaseFile, DateToUnix, CurrentFile } from './adsb-common.js';
import { GetTimes, TimesForPhase, ReadRange, StateFiles, StatePhases, GetStates, GetNearestTime, GetStateArray, GetStateIndex, DumpState, GetStateByTime, IsRange, GetRange, ResolveRange } from './adsb-store.js';
import { LogWrap, VfnAdapter, VfnDecorator, Mapper, DefaultConstructor, EventLogger, MessageReceiver, MessageTransmitter, MessageTransceiver, RPCApi, RPCProxy, RPCObject, RPCFactory, Connection, RPCServer, RPCClient, RPCSocket, GetProperties, GetKeys, MakeListCommand, SerializeValue, DeserializeSymbols, DeserializeValue, RPCConnect, RPCListen } from './quickjs/qjs-net/rpc.js';

extendArray(Array.prototype);

const scriptName = (arg = scriptArgs[0]) => path.basename(arg, path.extname(arg));

atexit(() => {
  console.log('atexit', atexit);
  let stack = new Error('').stack;
  console.log('stack:', stack);
});

const commands = {
  GetTimes,
  TimesForPhase,
  ReadRange,
  StateFiles,
  StatePhases,
  GetStates,
  GetNearestTime,
  GetStateArray,
  GetStateIndex,
  DumpState,
  GetStateByTime,
  IsRange,
  GetRange,
  ResolveRange,
  CurrentFile
};

let inotify_fd, watch_fd, watch_file;

function StartWatch() {
  if(inotify_fd == undefined) {
    os.setReadHandler(inotify_fd, () => {
      let ret = os.read(inotify_fd, buf, 0, buf.byteLength);
      console.log('ret', ret);
    });
    inotify_fd ??= watch();
  }
}

function WatchFile(filename) {
  let ev = new Uint32Array(4);
  let wd,
    ret,
    buf = ev.buffer;

  if(watch_file == filename) return;

  if(typeof watch_fd == 'number') watch(inotify_fd, watch_fd);

  return (watch_fd = watch(inotify_fd, filename, IN_MODIFY));
}

function PeriodicCheck() {
  let file = CurrentFile();
  console.log('PeriodicCheck', { file });

  WatchFile(file);

  os.setTimeout(PeriodicCheck, 10000);
}

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
    inspectOptions: { depth: Infinity, compact: 2, maxArrayLength: Infinity, maxStringLength: 30, customInspect: true }
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
      'ssl-ca': [true, null],
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
    'ssl-ca': sslCA = '/etc/ssl/certs/ca-certificates.crt',
    quiet = false,
    debug = false,
    tls = true
  } = params;

  StartWatch();
  PeriodicCheck();

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
      quiet ? 0 : LLL_USER | (((debug ? LLL_INFO : LLL_WARN) << 1) - 1),
      quiet
        ? () => {}
        : (level, str) => {
            if(/BIND_PROTOCOL|DROP_PROTOCOL|CHECK_ACCESS_RIGHTS|ADD_HEADERS/.test(str)) return;
            console.log(logLevels[level].padEnd(10), str.trim());
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
          ['.sh', 'text/x-shellscript']
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
          }
        ],
        ...url,
        ...callbacks,
        block: false,
        onConnect(ws, req) {
          console.log('onConnect', { ws, req }, req && req.headers);

          Object.defineProperties(ws, {
            sendMessage: {
              value: function sendMessage(msg) {
                let ret = this.send(JSON.stringify(msg));
                console.log(`ws.sendMessage(`, msg, `) = ${ret}`);
                return ret;
              },
              enumerable: false
            },
            dbg: { value: null, enumerable: false }
          });

          sockets.add(ws);

          ws.sendMessage = value => ws.send(JSON.stringify(value));
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
        onHttp(req, resp) {
          const { method, headers } = req;
          console.log('\x1b[38;5;33monHttp\x1b[0m [\n  ', req, ',\n  ', resp, '\n]');
          const { body, url } = resp;
          console.log('\x1b[38;5;33monHttp\x1b[0m', { body });

          const file = url.path.slice(1);
          const dir = file.replace(/\/[^\/]*$/g, '');

          if(file.endsWith('.js')) {
            console.log('onHttp', { file, dir });
            const re = /^(\s*(im|ex)port[^\n]*from ['"])([^./'"]*)(['"]\s*;[\t ]*\n?)/gm;

            resp.body = body.replaceAll(re, (match, p1, p0, p2, p3, offset) => {
              if(file == 'rbush.js') {
                console.log('RBUSH', resp.body);
              }

              if(!/[\/\.]/.test(p2)) {
                let fname = `${p2}.js`;

                if(!fs.existsSync(dir + '/' + fname)) return `/* ${match} */`;

                match = [p1, './' + fname, p3].join('');

                console.log('args', { match, p1, p2, p3, offset });
              }
              return match;
            });
          }

          return resp;
        },
        onMessage(ws, data) {
          console.log('onMessage', ws, data);
          let response;

          try {
            let obj;

<<<<<<< HEAD
            let cmd = data.substring(0, idx);
            let args = idx == data.length ? [] : data.substring(idx + 1).split(/\s+/g);
            console.log('onMessage', { cmd, args });
=======
            if(/^[A-Z]/.test(data)) {
              let idx = data.indexOf(' ');
              if(idx == -1) idx = data.length;
>>>>>>> bb2565ec2c0280276022101d2498ad8b0a3be31b

              let cmd = data.substring(0, idx);
              let args = idx == data.length ? [] : data.substring(idx + 1).split(/\s+/g);

              obj = { cmd, args };
            } else if(data[0] == '{') {
              obj = JSON.parse(data);
            }

<<<<<<< HEAD
          if(data[0] == 'l') {
            ws.sendMessage({ type: 'list', times: StatePhases() });
            return;
          }
=======
            if(obj !== undefined) {
              const { cmd, args } = obj;
              console.log('onMessage', { cmd, args });

              if(commands[cmd]) {
                let value = commands[cmd](...args);
                let response = { type: cmd, value };
                console.log('Sending response to ' + cmd + '()', response);
                ws.sendMessage(response);
                return;
              } else {
                throw new Error(`ERROR: Command not found: ${cmd}`);
              }
            }

            if(data[0] == 'l') {
              ws.sendMessage({ type: 'list', times: StateFiles().map(file => FilenameToTime(file)) });
              return;
            }
>>>>>>> bb2565ec2c0280276022101d2498ad8b0a3be31b

            let matches = [...data.matchAll(/\d+(-\d+)?/g)].map(([m]) => m);
            let states = [];
            console.log('matches', matches);
            for(let match of matches) {
              if(IsRange(match)) {
                let range = GetRange(match);
                console.log('range', range);

                states = states.concat(ResolveRange(...range));
              } else {
                let state = GetStateByTime(match);
                if(state) states.push(state);
              }
            }
            let arr = states.map(([time, obj]) => [+time, JSON.parse(obj).states]);
            //console.log('arr', arr);
            response = arr;
          } catch(error) {
            console.log('onMessage ERROR', error);
            response = { type: 'error', error: error.message };
          }
          if(Array.isArray(response)) response = { type: 'array', array: response };
          ws.send(JSON.stringify(response));
        },
        onFd(fd, rd, wr) {
          //console.log('onFd', { fd, rd, wr });
          os.setReadHandler(fd, rd);
          os.setWriteHandler(fd, wr);
        },
        ...(url && url.host ? url : {})
      })
    );
  });
  console.log('XX');

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
  console.log(`FAIL: ${error?.message ?? error}\n${error?.message}`);
}
