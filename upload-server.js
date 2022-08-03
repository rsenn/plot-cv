import * as std from 'std';
import * as os from 'os';
import * as deep from './lib/deep.js';
import require from 'require';
import path from 'path';
import Util from './lib/util.js';
import { Console } from 'console';
import REPL from './quickjs/qjs-modules/lib/repl.js';
import inspect from './lib/objectInspect.js';
import * as Terminal from './terminal.js';
import * as fs from './lib/filesystem.js';
import { toString, define } from 'util';
import { setLog, LLL_USER, LLL_NOTICE, LLL_WARN, client, server, FormParser } from 'net';
import { ReadFile, WriteFile, ReadJSON, WriteJSON, ReadBJSON, WriteBJSON } from './io-helpers.js';
import { parseDate, dateToObject } from './date-helpers.js';

globalThis.fs = fs;

function main(...args) {
  const base = path.basename(Util.getArgv()[1], '.js').replace(/\.[a-z]*$/, '');
  const config = ReadJSON(`.${base}-config`) ?? {};
  globalThis.console = new Console({ inspectOptions: { compact: 2, customInspect: true, maxArrayLength: 200 } });
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
  const is_server = !params.client || params.server;

  let name = process.env['NAME'] ?? Util.getArgs()[0];

  name = name
    .replace(/.*\//, '')
    .replace(/-/g, ' ')
    .replace(/\.[^\/.]*$/, '');

  let [prefix, suffix] = name.split(' ');

  let repl = new REPL(`\x1b[38;5;165m${prefix} \x1b[38;5;39m${suffix}\x1b[0m`, false);
  const histfile = '.upload-server-history';
  repl.historyLoad(histfile, false);
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

  let logFile =
    {
      puts(s) {
        repl.printStatus(() => std.puts(s));
      }
    } ?? std.open('upload-server.log', 'w+');

  let connections = new Set();
  const createWS = (globalThis.createWS = (url, callbacks, listen) => {
    //console.log('createWS', { url, callbacks, listen });

    const out = s => logFile.puts(s + '\n');
    setLog((params.debug ? LLL_USER : 0) | (((params.debug ? LLL_NOTICE : LLL_WARN) << 1) - 1), (level, message) => {
      if(/__lws/.test(message)) return;
      if(/(Unhandled|PROXY-|VHOST_CERT_AGING|BIND|HTTP_BODY|EVENT_WAIT)/.test(message)) return;

      if(params.debug)
        out(
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
          ).padEnd(8) + message.replace(/\n/g, '\\n')
        );
    });

    return [client, server][+listen]({
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
      mounts: [
        ['/', '.', 'upload.html'],
        ['/get', './uploads', ''],
        function proxy(req, res) {
          console.log('proxy', { req, res });
          const { url, method, headers } = req;
          console.log('proxy', { url, method, headers });
          const { status, ok, type } = res;

          console.log('proxy', { status, ok, url, type });
        },

        function* files(req, resp) {
          const { body, headers } = req;

          const data = JSON.parse(/*body ??*/ '{}');
          resp.type = 'application/json';
          let {
            dir = './uploads',
            filter = '[^.].*' ?? '.(brd|sch|G[A-Z][A-Z])$',
            verbose = false,
            objects = false,
            key = 'mtime',
            limit = null
          } = data ?? {};
          let absdir = path.realpath(dir);
          let components = absdir.split(path.sep);
          if(components.length && components[0] === '') components.shift();
          if(components.length < 2 || components[0] != 'home') throw new Error(`Access error`);

          let names = fs.readdirSync(absdir) ?? [];

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
                mtime: Util.toUnixTime(st.mtime),
                time: Util.toUnixTime(st.ctime),
                mode: `0${(st.mode & 0x09ff).toString(8)}`,
                size: st.size
              })
            ]);
            return acc;
          }, []);

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
          // console.log('req.headers', req.headers);
          //resp.headers = { ['Content-Type']: 'application/json' };
          //        resp.headers['Content-Type']= 'application/json';

          // console.log('resp.headers', resp.headers);
          yield JSON.stringify(...[names, ...(verbose ? [null, 2] : [])]);
        }
      ],
      ...url,

      ...callbacks,
      onConnect(ws, req) {
        console.log('upload-server', { ws, req });

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
      onHttp(ws, req, resp) {
        const { method, headers } = req;

        if(req.url.path.endsWith('files')) {
          //resp.headers= { ['Content-Type']: 'application/json' };
          resp.type = 'application/json';
        }

        if(req.method != 'GET') console.log('\x1b[38;5;33monHttp\x1b[0m [\n  ', req, ',\n  ', resp, '\n]');

        if(req.method != 'GET') {
          //  console.log(req.method + ' body:',  req.body);

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
        }

        const { body, url } = resp;
        console.log('\x1b[38;5;33monHttp\x1b[0m', req, resp, { body });

        const file = url.path.slice(1);
        const dir = file.replace(/\/[^\/]*$/g, '');

        if(file.endsWith('.txt')) {
          resp.body = fs.readFileSync(file, 'utf-8');
        }
        if(file.endsWith('.js')) {
          console.log('onHttp', { file, dir });
          const re = /^(\s*(im|ex)port[^\n]*from ['"])([^./'"]*)(['"]\s*;[\t ]*\n?)/gm;

          resp.body = body.replaceAll(re, (match, p1, p0, p2, p3, offset) => {
            if(!/[\/\.]/.test(p2)) {
              let fname = `${p2}.js`;

              if(!fs.existsSync(dir + '/' + fname)) return ``;

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
        return callbacks.onMessage(ws, data);
      },
      onFd(fd, rd, wr) {
        return callbacks.onFd(fd, rd, wr);
      },
      ...(url && url.host ? url : {})
    });
  });

  define(globalThis, {
    get connections() {
      return [...connections];
    }
  });

  Object.assign(globalThis, {
    repl,
    Util,
    quit,
    exit: quit,
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

  delete globalThis.DEBUG;

  createWS(
    { protocol: 'ws', host: '0.0.0.0', port: 8999 },
    {
      onFd(fd, rd, wr) {
        os.setReadHandler(fd, rd);
        os.setWriteHandler(fd, wr);
      }
    },
    true
  );

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
}
