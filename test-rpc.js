import { client, FormParser, LLL_NOTICE, LLL_USER, LLL_WARN, createServer, setLog } from 'net';
import * as os from 'os';
import * as path from 'path';
import { dateToObject, parseDate } from './date-helpers.js';
import { ReadBJSON, ReadFile, ReadJSON, WriteBJSON, WriteFile, WriteJSON } from './io-helpers.js';
import * as deep from './lib/deep.js';
import { EventEmitter } from './lib/events.js';
import * as fs from './lib/filesystem.js';
import inspect from './lib/objectInspect.js';
import { Repeater } from './lib/repeater/repeater.js';
import { Socket } from './quickjs/qjs-ffi/lib/socket.js';
import { REPL } from './quickjs/qjs-modules/lib/repl.js';
import * as rpc from './quickjs/qjs-net/js/rpc.js';
import * as rpc2 from './quickjs/qjs-net/js/rpc.js';
import * as Terminal from './terminal.js';
import { Console } from 'console';
import { toString, getOpt } from 'util';
import require from 'require';
import * as std from 'std';
globalThis.fs = fs;

function main(...args) {
  const base = path.basename(scriptArgs[0], '.js').replace(/\.[a-z]*$/, '');
  const config = ReadJSON(`.${base}-config`) ?? {};
  globalThis.console = new Console({ inspectOptions: { compact: 2, customInspect: true, maxArrayLength: 200 } });
  let params = getOpt(
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
  //console.log('params', params);
  const { address = '0.0.0.0', port = 8999, 'ssl-cert': sslCert = 'localhost.crt', 'ssl-private-key': sslPrivateKey = 'localhost.key' } = params;
  const listen = params.connect && !params.listen ? false : true;
  const is_server = !params.client || params.server;
  Object.assign(globalThis, { ...rpc2, rpc });
  let name = process.env['NAME'] ?? process.argv[1];
  /*console.log('argv[1]',process.argv[1]);*/
  name = name
    .replace(/.*\//, '')
    .replace(/-/g, ' ')
    .replace(/\.[^\/.]*$/, '');

  let [prefix, suffix] = name.split(' ');

  let repl = new REPL(`\x1b[38;5;165m${prefix} \x1b[38;5;39m${suffix}\x1b[0m`, false);
  const histfile = '.test-rpc-history';
  repl.historyLoad(histfile, false);
  repl.loadSaveOptions();
  repl.directives.i = [
    (module, ...args) => {
      console.log('args', args);
      try {
        return require(module);
      } catch(e) {}
      import(module).then(m => (globalThis[module] = m));
    },
    'import module'
  ];

  //repl.help = () => {};
  let { log } = console;
  repl.show = arg => std.puts((typeof arg == 'string' ? arg : inspect(arg, globalThis.console.options)) + '\n');

  repl.cleanup = () => {
    repl.readlineRemovePrompt();
    Terminal.mousetrackingDisable();
    let numLines = repl.historySave(histfile);

    repl.printStatus(`EXIT (wrote ${numLines} history entries)`, false);

    std.exit(0);
  };

  repl.inspectOptions = { ...(repl.inspectOptions ?? console.options), depth: 4, compact: false };

  console.log = (...args) => repl.printStatus(() => log(console.config(repl.inspectOptions), ...args));

  let cli = (globalThis.sock = new rpc.RPCSocket(`${address}:${port}`, rpc[`RPC${is_server ? 'Server' : 'Client'}Connection`], +params.verbose));

  cli.register({ Socket, Worker: os.Worker, Repeater, REPL, EventEmitter });
  let logFile =
    {
      puts(s) {
        repl.printStatus(() => std.puts(s));
      }
    } ?? std.open('test-rpc.log', 'w+');

  let connections = new Set();
  const createWS = (globalThis.createWS = (url, callbacks, listen) => {
    console.log('createWS', { url, callbacks, listen });

    const out = s => logFile.puts(s + '\n');
    setLog((params.debug ? LLL_USER : 0) | (((params.debug ? LLL_NOTICE : LLL_WARN) << 1) - 1), (level, message) => {
      //repl.printStatus(...args);
      if(/__lws/.test(message)) return;
      if(/(Unhandled|PROXY-|VHOST_CERT_AGING|BIND|HTTP_BODY)/.test(message)) return;
      //
      if(params.debug)
        out((['ERR', 'WARN', 'NOTICE', 'INFO', 'DEBUG', 'PARSER', 'HEADER', 'EXT', 'CLIENT', 'LATENCY', 'MINNET', 'THREAD'][Math.log2(level)] ?? level + '').padEnd(8) + message.replace(/\n/g, '\\n'));
    });

    return [client, createServer][+listen]({
      tls: params.tls,
      sslCert,
      sslPrivateKey,
      mimetypes: [
        ['.svgz', 'application/gzip'],
        ['.mjs', 'application/javascript'],
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
      options: {
        'upload-dir': './uploads',
        'max-size': 10000000,
        'basic-auth': 'quickjs/qjs-net/libwebsockets/minimal-examples/http-server/minimal-http-server-deaddrop/ba-passwords'
      },
      mounts: [
        ['/', '.', 'debugger.html'],
        ['/upload', 'lws-deaddrop', null, 'lws-deaddrop'],
        ['/get', './uploads', ''],
        function proxy(req, res) {
          console.log('proxy', { req, res });
          const { url, method, headers } = req;
          console.log('proxy', { url, method, headers });
          const { status, ok, type } = res;

          console.log('proxy', { status, ok, url, type });
        },

        function* config(req, res) {
          const { body, headers } = req;
          console.log('/config', { req, res });
          console.log('*config', { body, headers });
          yield '{}';
        },
        function* files(req, resp) {
          const { body, headers } = req;
          const { 'content-type': content_type } = headers;
          //console.log('*files', { body });
          const data = JSON.parse(body);
          resp.type = 'application/json';
          let { dir = '.' ?? 'tmp', filter = '.([ch]|js)$' ?? '.(brd|sch|G[A-Z][A-Z])$', verbose = false, objects = false, key = 'mtime', limit = null } = data ?? {};
          let absdir = path.realpath(dir);
          let components = absdir.split(path.sep);
          if(components.length && components[0] === '') components.shift();
          if(components.length < 2 || components[0] != 'home') throw new Error(`Access error`);
          //console.log('\x1b[38;5;215m*files\x1b[0m', { dir, components, absdir });
          //console.log('\x1b[38;5;215m*files\x1b[0m', { absdir, filter });
          let names = fs.readdirSync(absdir) ?? [];
          //console.log('\x1b[38;5;215m*files\x1b[0m', { names });
          if(filter) {
            const re = new RegExp(filter, 'gi');
            names = names.filter(name => re.test(name));
          }
          if(limit) {
            let [offset = 0] = limit;
            let [, length = names.length - start] = limit;
            names = names.slice(offset, offset + length);
          }
          let entries = names.map(file => [file, fs.statSync(`${dir}/${file}`)]);
          entries = entries.reduce((acc, [file, st]) => {
            let name = file + (st.isDirectory() ? '/' : '');
            let obj = {
              name
            };
            acc.push([
              name,
              Object.assign(obj, {
                mtime: toUnixTime(st.mtime),
                time: toUnixTime(st.ctime),
                mode: `0${(st.mode & 0x09ff).toString(8)}`,
                size: st.size
              })
            ]);
            return acc;
          }, []);
          //console.log('\x1b[38;5;215m*files\x1b[0m', console.config({ depth: 3 }), { entries });
          if(entries.length) {
            let cmp = {
              string(a, b) {
                return b[1][key].localeCompare(a[1][key]);
              },
              number(a, b) {
                return b[1][key] - a[1][key];
              }
            }[typeof entries[0][1][key]];
            entries = entries.sort(cmp);
          }
          names = entries.map(([name, obj]) => (objects ? obj : name));
          console.log('\x1b[38;5;215m*files\x1b[0m', names);
          yield JSON.stringify(...[names, ...(verbose ? [null, 2] : [])]);
        }
      ],
      ...url,

      ...callbacks,
      onConnect(ws, req) {
        console.log('test-rpc', { ws, req });

        console.log('req.url.path', req.url.path);

        connections.add(ws);
        if(req.url.path.endsWith('uploads')) {
        } else {
          return callbacks.onConnect(ws, req);
        }
      },
      onClose(ws) {
        connections.delete(ws);

        return callbacks.onClose(ws, req);
      },
      onRequest(ws, req, resp) {
        const { method, headers } = req;

        if(req.method != 'GET') console.log('\x1b[38;5;33monRequest\x1b[0m [\n  ', req, ',\n  ', resp, '\n]');

        if(req.method != 'GET') {
          console.log(req.method + ' body:', /*typeof req.body, req.body.length, */ req.body);
          console.log('ws', ws);

          let fp = new FormParser(ws, ['files'], {
            chunkSize: 8192 * 256,
            onContent(name, data) {
              console.log(`onContent(${name})`, this.filename, data.byteLength);
              fs.writeSync(this.file, data);
            },
            onOpen(name, filename) {
              this.name = name;
              this.filename = filename;
              this.file = fs.openSync('uploads/' + filename, 'w+', 0o644);
              console.log(`onOpen(${name})`, filename);
            },
            onClose(name) {
              fs.closeSync(this.file);
              console.log(`onClose(${this.name})`, this.filename);
            }
          });
          console.log('fp.socket', fp.socket);
          console.log('fp.params', fp.params);

          /*  (async function() {
            let r,
              buffers = [];

            while((r = req.body.next())) {
              console.log('r:', r);
              const { value, done } = await r;
              console.log('value:', value);
              //console.log('toString(value)', toString(value));
              console.log('done:', done);
              if(done) break;
              buffers.push(value);
            }
            console.log('req.headers:', req.headers);
            console.log(
              'buffers:',
              buffers.map(b => b.byteLength)
            );
            let data = concat(...buffers);
            console.log('data:', data);
            console.log('data.byteLength:', data.byteLength);
            let pos1 = searchArrayBuffer(data, new Uint8Array([13, 10]).buffer);
            let pos = searchArrayBuffer(data, new Uint8Array([13, 10, 13, 10]).buffer);
            console.log('pos:', pos);

            console.log('header:', toString(data.slice(0, pos)));
            let body = data.slice(pos + 4);
            let pos2 = searchArrayBuffer(body, data.slice(0, pos1));
            console.log('pos2:', pos2);

            WriteFile('out.bin', data);
          })();*/
        }

        const { body, url } = resp;
        console.log('\x1b[38;5;33monRequest\x1b[0m', req, resp, { body });

        const file = url.path.slice(1);
        const dir = file.replace(/\/[^\/]*$/g, '');

        if(file.endsWith('.txt')) {
          resp.body = ReadFile(file, 'utf-8');
        }
        if(file.endsWith('.js')) {
          console.log('onRequest', { file, dir });
          const re = /^(\s*(im|ex)port[^\n]*from ['"])([^./'"]*)(['"]\s*;[\t ]*\n?)/gm;

          resp.body = body.replaceAll(re, (match, p1, p0, p2, p3, offset) => {
            if(!/[\/\.]/.test(p2)) {
              let fname = `${p2}.js`;

              if(!fs.existsSync(dir + '/' + fname)) return `/* ${match} */`;

              match = [p1, './' + fname, p3].join('');

              console.log('args', { match, p1, p2, p3, offset });
            }
            return match;
          });
        }
        /*   if(req.url.path.endsWith('upload')) {
          resp.status = 302;
          resp.headers['Location'] = '/upload.html';
          resp.headers = { ['Location']: '/upload.html' };
        }*/
        return resp;
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
    std,
    os,
    deep,
    fs,
    path,
    ReadJSON,
    WriteJSON,
    ReadFile,
    WriteFile,
    ReadBJSON,
    WriteBJSON,
    parseDate,
    dateToObject
  });

  define(globalThis, listen ? { server: cli, cli } : { client: cli, cli });
  delete globalThis.DEBUG;
  //Object.defineProperty(globalThis, 'DEBUG', { get: DebugFlags });

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