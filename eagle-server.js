import * as fs from 'fs';
import { client, getSessions, LLL_INFO, LLL_USER, LLL_WARN, logLevels, createServer, setLog } from 'net';
import * as os from 'os';
import { atexit, daemon, getOpt, randStr } from 'util';
import { ReadFile, ReadJSON, WriteJSON } from './io-helpers.js';
import * as path from './lib/path.js';
import { Console } from './quickjs/qjs-modules/lib/console.js';
import { REPL } from './quickjs/qjs-modules/lib/repl.js';
import { VirtFS } from './virtfs.js';
import extendArray from 'extendArray';
import * as std from 'std';
#!/usr/bin/env qjsm
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
  repl.loadSaveOptions();
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
  resp.headers = {
    ['content-security-policy']: `default-src 'self'; script-src ${resp.url.host}:${resp.url.port} 'nonce-${nonce}'`
  };
  //resp.headers = { ['content-security-policy']: `all-src self 'nonce-${nonce}';` };
  return nonce;
}

function GetArgs(argStr) {
  const re = /([^/:&?=]+)=([^/:&?]+)/g;
  let entries = [];

  for(;;) {
    let [, key, value] = re.exec(argStr) ?? [];

    if(!key) break;

    //console.log('GetArgs', {key,value});
    entries.push([key, value]);
  }
  return entries.reduce((acc, [k, v]) => {
    acc[k] = v;
    return acc;
  }, {});
}

function main(...args) {
  const base = scriptName().replace(/\.[a-z]*$/, '');

  const config = ReadJSON(`.${base}-config`) ?? {};

  globalThis.console = new Console(std.err, {
    inspectOptions: { depth: Infinity, compact: 1, maxArrayLength: Infinity, maxStringLength: 300, customInspect: true }
  });

  let vfs;

  try {
    vfs = new VirtFS(['data', 'tmp', '../an-tronics/eagle', '../insider/eagle', '../lc-meter/eagle', '../pictest/eagle']);

    /*  let testFiles = [...vfs.readdirSync('.')];
    console.log('vfs.readdirSync', console.config({ compact: false }), testFiles);*/
  } catch(err) {
    console.log('err', err);
  }

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
    let netfn = [client, createServer][+listen];
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
        mounts: {
          ['/']: ['/', '.', 'index.html'],
          ['/config']: function* config(req, res) {
            const { body, headers } = req;
            console.log('/config', { req, res });
            console.log('*config', { body, headers });
            yield '{}';
          },
          ['/files']: function* files(req, res) {
            const { body, headers, url } = req;
            const { query } = url;
            const params = (typeof body == 'string' ? JSON.parse(body) : typeof body == 'object' ? body : null) ?? {};
            console.log('/files', console.config({ compact: 0 }), { params });
            let ret,
              argObj = {},
              args = url.path.replace(/.*files./g, '');
            if(args) {
              argObj = GetArgs(args);
              console.log('/files', console.config({ compact: 0 }), { argObj });
            }
            ret = vfs.readdirSync('.', (fileName, filePath) => {
              let st = fs.statSync(filePath);
              return Object.assign({ name: fileName, dir: path.dirname(filePath), ...st });
            });
            res.type = 'application/json';
            res.status = 200;

            console.log('/files', console.config({ compact: 0 }), { args, res });
            if(params.filter) {
              let re = new RegExp(params.filter, 'gi');
              ret = ret.filter(({ name }) => re.test(name));
            }

            yield JSON.stringify(ret.filter(({ mode }) => mode & os.S_IFREG));
          }
        },
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
        onRequest(req, resp) {
          const { method, headers } = req;
          //console.log('\x1b[38;5;33monRequest\x1b[0m [\n  ', req, ',\n  ', resp, '\n]');
          const { url } = resp;

          const { path, host } = url;
          //console.log('\x1b[38;5;33monRequest\x1b[0m', { path, host });

          const file = path.slice(1);
          const dir = path.replace(/\/[^\/]*$/g, '');

          // console.log('\x1b[38;5;33monRequest\x1b[0m', { file, dir });

          let nonce;

          let { body } = resp;
          if(body === undefined) body = resp.body = ReadFile(file);

          nonce = GetNonce(resp);

          if(file.endsWith('.html') || file == '' || file == '/') {
            //console.log('\x1b[38;5;33monRequest\x1b[0m', { body, nonce });
            resp.body = body.replaceAll('@@=AAABBBCCCZZZ=@@', 'nonce-' + nonce);
            // console.log('resp.body', escape(body));
          }
          // console.log('resp.headers (1)', resp.headers);

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
    let r = os.read(0, inputBuf, 0, inputBuf.byteLength);

    if(r > 0) {
      let a = new Uint8Array(inputBuf.slice(0, r));

      //console.log('a', a);

      for(let i = 0; i < a.length; i++) if(a[i] == 13) a[i] = 10;

      if(a.length == 1 && a[0] == 127) a = new Uint8Array([8, 0x20, 8]);

      if(a.length == 1 && a[0] == 27) showSessions();
      else os.write(1, a.buffer, 0, a.buffer.byteLength);
    }
  });

  function showSessions() {
    let sessions = getSessions();
    console.log('sessions', console.config({ maxArrayLength: Infinity, depth: 4, customInspect: true, compact: 1 }), sessions);
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