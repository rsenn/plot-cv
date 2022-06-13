import * as std from 'std';
import * as os from 'os';
import { setInterval } from 'timers';
import * as deep from './lib/deep.js';
import * as path from './lib/path.js';
 import { randStr,watch, IN_MODIFY, memoize, daemon, atexit, getpid, toArrayBuffer, toString, escape, quote, define, extendArray, getOpt, glob } from 'util';
import { Console } from './quickjs/qjs-modules/lib/console.js';
import REPL from './quickjs/qjs-modules/lib/repl.js';
import inspect from './lib/objectInspect.js';
import * as Terminal from './terminal.js';
import * as fs from 'fs';
import { setLog, logLevels, getSessions, LLL_USER, LLL_INFO, LLL_NOTICE, LLL_WARN, client, server } from 'net';
import { IfDebug, LogIfDebug, ReadFile, LoadHistory, ReadJSON, ReadXML, MapFile, WriteFile, WriteJSON, WriteXML, ReadBJSON, WriteBJSON, DirIterator, RecursiveDirIterator, ReadDirRecursive, Filter, FilterImages, SortFiles, StatFiles, ReadFd, FdReader, CopyToClipboard, ReadCallback, LogCall, Spawn, FetchURL } from './io-helpers.js';

extendArray(Array.prototype);

const scriptName = (arg = scriptArgs[0]) => path.basename(arg, path.extname(arg));

atexit(() => {
  console.log('atexit', atexit);
  let stack = new Error('').stack;
  console.log('stack:', stack);
});

let inotify_fd, watch_fd, watch_file, watch_offset;

function StartWatch() {
  if(inotify_fd == undefined) {
    let ev = new Uint32Array(4);
    let ret,
      buf = ev.buffer;
    inotify_fd = watch();

    os.setReadHandler(inotify_fd, () => {
      let ret = os.read(inotify_fd, buf, 0, buf.byteLength);
      //console.log('inotify', { ret, ev });
      let new_offset = fs.sizeSync(watch_file);
      let size = new_offset - watch_offset;
      if(size) {
        let data = (globalThis.lastData = ReadRange(watch_file, watch_offset, size));
        console.log('send', data);
        sockets.forEach(ws => ws.send(data));
      }
      watch_offset = new_offset;
    });
  }
}

function WatchFile(filename) {
  console.log('WatchFile', { filename });
  let ev = new Uint32Array(4);
  let wd;

  //if(watch_file == filename) return;

  if(typeof watch_fd == 'number') watch(inotify_fd, watch_fd);

  watch_fd = watch(inotify_fd, (watch_file = filename), IN_MODIFY);
  watch_offset = fs.sizeSync(filename);

  return watch_fd;
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

function GetHeader(resp, key) {
  const { headers } = resp;
  let value;
  if(typeof headers == 'object' && headers != null) value = headers[key] ?? headers[key.toLowerCase()];
  return value;
}

function GetNonce(resp) {
  let csp, ret;

  if((csp = GetHeader(resp, 'Content-Security-Policy'))) {
    let m = /'nonce-([^']*)'/.exec(csp) ?? [];
    ret = m[1];
  }

  if(!ret) ret = SetNonce(resp);

  return ret;
}

function SetNonce(resp, nonce = randStr(32)) {
  resp.headers = { ['content-security-policy']: `script-src ${resp.url.host} 'nonce-${nonce}'` };
  return nonce;
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

  /*StartWatch();
  PeriodicCheck();*/

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
         if(level == LLL_INFO) return;
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
        errorDocument: '/404.html',
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
          ['/', '.', 'index.html'],
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

          if(globalThis.lastData) ws.sendMessage(globalThis.lastData);
        },
        onClose(ws) {
          console.log('onClose', ws);
          protocol.delete(ws);
          sockets.delete(ws);

          dbg.close();
        },
        onError(ws) {
          console.log('onError', ws);
          protocol.delete(ws);
          sockets.delete(ws);
        },

        onMessage(ws, data) {
          console.log('onMessage', ws, data);
        },
        onHttp(req, resp) {
          const { method, headers } = req;
          console.log('\x1b[38;5;33monHttp\x1b[0m [\n  ', req, ',\n  ', resp, '\n]');
          const { url } = resp;

          //if(url.path == '' || url.path == '/') url.path = '/index.html';

          const { path, host } = url;
          console.log('\x1b[38;5;33monHttp\x1b[0m', { path, host });

          const file = path.slice(1);
          const dir = path.replace(/\/[^\/]*$/g, '');

          console.log('\x1b[38;5;33monHttp\x1b[0m', { file, dir });

          let nonce;

          let { body } = resp;
          if(body === undefined) body = resp.body = ReadFile(file);

          if(file.endsWith('.html') || file == '' || file == '/') {
            nonce = GetNonce(resp);

            console.log('\x1b[38;5;33monHttp\x1b[0m', { body, nonce });
            resp.body = body.replaceAll('@@=AAABBBCCCZZZ=@@', 'nonce-' + nonce);
            console.log('resp.body', escape(body));
          }
          console.log('resp.headers (1)', resp.headers);

          /*    if(nonce) {
            let headers = { ['content-security-policy']: `script-src ${host} 'nonce-${nonce}'` };

            console.log('resp.headers (2)', (resp.headers = headers));
          }*/

          return resp;
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
