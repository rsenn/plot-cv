import * as std from 'std';
import * as os from 'os';
import * as deep from 'deep';
import * as xml from 'xml';
import * as path from 'path';
import { Console } from 'console';
import { Directory, NAME, BOTH, TYPE_DIR, TYPE_MASK, TYPE_LNK, TYPE_REG } from 'directory';
import { REPL } from 'repl';
import inspect from 'inspect';
import * as Terminal from 'terminal';
import * as fs from 'fs';
import { unlink, error, fnmatch } from 'misc';
import { keys, toString, define, toUnixTime, getOpt, randStr, isObject, isArrayBuffer, glob, GLOB_BRACE, waitFor } from 'util';
import { createServer, setLog, LLL_USER, LLL_NOTICE, LLL_WARN, LLL_INFO, logLevels, FormParser, Hash, Response, Socket } from 'net';
import { parseDate, dateToObject } from './date-helpers.js';
import { ReadFile, ReadJSON, WriteFile, WriteJSON, ReadBJSON, WriteBJSON } from './io-helpers.js';
import { ExecTool } from './os-helpers.js';
import { h } from './lib/preact.mjs';
import renderToString from './lib/preact-render-to-string.js';
import { spawn } from 'child_process';
import trkl from './lib/trkl.js';
import extendArray from 'extendArray';
import extendGenerator from 'extendGenerator';
import extendAsyncGenerator from 'extendAsyncGenerator';
import { RecursiveDirIterator } from './dir-helpers.js';
import { MessageReceiver, MessageTransmitter, MessageTransceiver, codecs, RPCApi, RPCProxy, RPCObject, RPCFactory, Connection, RPC_PARSE_ERROR, RPC_INVALID_REQUEST, RPC_METHOD_NOT_FOUND, RPC_INVALID_PARAMS, RPC_INTERNAL_ERROR, RPC_SERVER_ERROR_BASE, FactoryEndpoint, RPCServer, RPCClient, FactoryClient, RPCSocket, GetProperties, GetKeys, SerializeValue, DeserializeSymbols, DeserializeValue, RPCConnect, RPCListen, } from './quickjs/qjs-net/js/rpc.js';
import { PromiseWorker } from './promise-worker.js';

const DEBUG = false;
const isin = (other, p) => path.slice(other, 0, path.length(p)) == p;

extendArray();
extendGenerator();
extendGenerator(Object.getPrototypeOf(new Map().keys()));
//extendGenerator(Object.getPrototypeOf(new Directory('.')));
extendAsyncGenerator();

class Match {
  constructor(pattern) {
    this.pattern = pattern;
  }

  apply() {
    return glob(this.pattern).map(name => {
      return [name, ReadFile(name)];
    });
  }
}

globalThis.worker = new PromiseWorker(new os.Worker('./upload-worker.js'));

globalThis.fs = fs;
globalThis.logFilter = /(ws_set_timeout: on immortal stream|Unhandled|PROXY-|VHOST_CERT_AGING|BIND|EVENT_WAIT|WRITABLE)/;

trkl.property(globalThis, 'logLevel').subscribe(value => {
  setLog(value, (level, message) => {
    if(/__lws|serve_(resolved|xgenerator|promise|response)|XXbl(\([123]\).*writable|x\([/]\).*WRITEABLE)|lws_/.test(message)) return;
    if(level == LLL_INFO && !/proxy/.test(message)) return;
    if(logFilter.test(message)) return;

    out(logLevels[level].padEnd(8) + message.replace(/\n/g, '\\n').replace(/\r/g, '\\r'));
  });
});

async function AsyncCollect(iter) {
  let ret = [];
  for await(let chunk of await iter) ret.push(chunk);
  return ret;
}

function GetMime(file) {
  let output = ExecTool('file', '-i', file);
  output = output.replace(/[\r\n]*$/g, '');
  output = output.replace(new RegExp(file + ':\\s+'), '');
  output = output.replace(/\;\s*.*/g, '');
  return output;
}

function Matcher(pattern, t = arg => arg) {
  return (...args) => pattern == t(...args) || 0 == fnmatch(pattern, t(...args), 0);
}

function KeyOrValueMatcher(pattern) {
  let matcher = Matcher(pattern);
  return ([key, value]) => matcher(key) || matcher(value);
}

function GetRootDirectories(pattern = '*') {
  return allowedDirs.keys().filter(Matcher(pattern));
}

const MakeUUID = (rng = Math.random) => [8, 4, 4, 4, 12].map(n => randStr(n, '0123456789abcdef'), rng).join('-');

const defaultDirs = (globalThis.defaultDirs = [
  '.',
  ...glob('../*/eagle'),
  './uploads/*.{sch,brd,lbr}',
  '/mnt/extext/Photos/*APPLE/*.{JPG,PNG,GIF,AAE,MOV,HEIC,MP4,WEBP}',
  ['/home/roman/Bilder', new RegExp('.(jpg|jpeg|png|heic|tif|tiff)$', 'i')],
]);

const allowedDirs = (globalThis.allowedDirs = new Map(
  defaultDirs
    .map(dd => GetDir(Array.isArray(dd) ? dd[0] : dd))
    .map(d => path.resolve(d))
    .map(d => path.relative(d))
    .map(d => [DirName(d), d]),
));

function GetDir(dir) {
  let a = dir.split(new RegExp(path.sep + '+', 'g'));
  let i = a.findIndex(n => /[*{}]/.test(n));
  return i != -1 ? path.slice(dir, 0, i) : dir;
}

function DirName(name) {
  let p = path.relative(name);

  p = path.slice(
    p,
    p.split(new RegExp(path.sep + '+', 'g')).findIndex(it => it != '..'),
  );
  return p;
}

function DateStr(date) {
  let str = date.toISOString();
  let ti = str.indexOf('T');
  return (
    str.substring(0, ti) /*.replace(/-/g, '')*/ + ' ' + str.substring(ti + 1, ti + 1 + 8) //.replace(/:/g, '')
  );
}

function ModeStr(mode) {
  return (
    (mode & (0o120000 == 0o120000) ? 'l' : mode & 0o40000 ? 'd' : '-') +
    (mode & 0b100000000 ? 'r' : '-') +
    (mode & 0b010000000 ? 'w' : '-') +
    (mode & 0b001000000 ? 'x' : '-') +
    (mode & 0b100000 ? 'r' : '-') +
    (mode & 0b010000 ? 'w' : '-') +
    (mode & 0b001000 ? 'x' : '-') +
    (mode & 0b100 ? 'r' : '-') +
    (mode & 0b010 ? 'w' : '-') +
    (mode & 0b001 ? 'x' : '-')
  );
}

const HTMLPage = ({ title, style, scripts = [], children, ...props }) => {
  return h('html', {}, [
    h(
      'head',
      {},
      [
        h('title', {}, [title]),
        ...(style ? [h('style', { type: 'text/css' }, [style])] : []),
        h(
          'link',
          {
            rel: 'stylesheet',
            href: 'static/css/list.css',
          },
          [],
        ),
      ].concat(scripts.map(script => h('script', { type: 'module', src: script }))),
    ),
    h('body', {}, children ?? []),
  ]);
};

const FilterForm = ({ ...props }) =>
  h('form', { action: 'file', method: 'POST' }, [
    h(
      'input',
      {
        type: 'text',
        size: 20,
        name: 'filter',
        value: '.*' /*,
          onchange: e => {
            console.log('onchange', e);
          }*/,
      },
      [],
    ),
    h('input', { type: 'submit', value: 'Submit', class: 'hidden' }, []),
  ]);

const HTMLTable = ({ rows, ...props }) => h('table', props, rows);
const HTMLTableRow = ({ columns, ...props }) => h('tr', props, columns);
const HTMLTableColumn = ({ columns, children, tag = 'td', ...props }) => h(tag, props, children);

const FileLink = ({ file, ...props }) => {
  return h('a', { href: 'uploads/' + file }, [file]);
};
const FileObject = ({ file, stat = {}, ...props }) => {
  stat ??= fs.lstatSync('uploads/' + file);
  // console.log('FileObject', { file, stat });
  let isDir = stat.isDirectory();

  return h(HTMLTableRow, {}, [
    h(HTMLTableColumn, { class: 'mode' }, [
      //(stat.mode & 0b111111111).toString(8),
      ModeStr(stat.mode),
    ]),
    h(HTMLTableColumn, { class: 'file' }, [h(FileLink, { file: file + (isDir ? '/' : '') }, [])]),
    h(HTMLTableColumn, { class: 'size' }, [stat.size + '']),
    h(HTMLTableColumn, { class: 'date' }, [DateStr(new Date(stat.atime))]),
  ]);
};

const FileTable = ({ files, ...props }) => {
  //console.log('FileTable', files);

  let fields = ['mode', 'file', 'size', 'atime'];

  files = files.sort((a, b) => a.file.localeCompare(b.file));

  return h(
    HTMLTable,
    {
      ...props,
      cellspacing: 0,
      cellpadding: 0,
      border: 0,
      rows: [
        h(
          HTMLTableRow,
          {
            columns: fields.map(field => h(HTMLTableColumn, { tag: 'th' }, [field])),
          },
          [],
        ),
      ].concat(files.map(file => h(FileObject, file, []))),
    },
    [],
  );
};

function ReadExiv2(file) {
  console.log('ReadExiv2', file);
  let xmpdat = ExecTool('exiv2', '-e', 'X-', 'ex', file);

  // console.log('xmpdat', xmpdat);
  let xmp = xml.read(xmpdat);
  // console.log('xmp', xmp);
  let flat = Object.fromEntries(
    deep
      .flatten(xmp, [])
      .filter(([k, v]) => v !== '' && /attributes.*:/.test(k) && !/\.xmlns/.test(k) && !isObject(v))
      .filter(([k, v]) => /(GPS|[XY]Dim|[XY]Res|Date$|Make$|Model$)/.test(k))
      .map(([k, v]) => [k.replace(/.*\.attributes\./g, ''), v])
      .sort((a, b) => a[0].localeCompare(b[0]))
      .map(([k, v]) => [k, isNaN(+v) ? (isNaN(Date.parse(v)) ? v : new Date(v)) : +v]),
  );
  return flat;
}

function ReadExiftool(file) {
  console.log('ReadExiftool', file);

  let out = ExecTool('exiftool', '-S', '-ee', file);

  let a = out.split(/\r?\n/g).filter(l => l != '');

  a = a.map(line => [line, line.indexOf(': ')]).map(([line, idx]) => [line.slice(0, idx), line.slice(idx + 2)]);
  let o = Object.fromEntries(a);

  //console.log('ReadExiftool',o);
  return o;
}

function HeifConvert(src, dst, quality = 100) {
  console.log('HeifConvert', src, dst);
  let child = spawn('heif-convert', ['-q', quality + '', src, dst], {
    block: false,
    stdio: ['inherit', 'inherit', 'inherit'],
  });

  console.log('HeifConvert', child);
  child.wait();
}

function MagickResize(src, dst, rotate = 0, width, height) {
  console.log('MagickResize', {
    width,
    height,
    dst,
    rotate,
  });
  let child = spawn('convert', [src, '-resize', width + 'x' + height, ...(rotate ? ['-rotate', '-' + rotate] : []), dst], { block: false, stdio: ['inherit', 'inherit', 'inherit'] });

  console.log('MagickResize', { child });
  child.wait();
}

function main(...args) {
  const base = path.basename(scriptArgs[0], '.js').replace(/\.[a-z]*$/, '');
  const config = ReadJSON(`.${base}-config`) ?? {};

  globalThis.console = new Console({
    inspectOptions: {
      compact: 2,
      depth: Infinity,
      customInspect: true,
      maxArrayLength: 200,
      protoChain: true,
    },
  });
  let params = (globalThis.params = getOpt(
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
      'ssl-cert': [true, null, 's'],
      'ssl-private-key': [true, null, 'k'],
      '@': 'address,port',
    },
    args,
  ));
  if(params['no-tls'] === true) params.tls = false;

  const { address = '0.0.0.0', port = 8999, 'ssl-cert': sslCert = 'localhost.crt', 'ssl-private-key': sslPrivateKey = 'localhost.key' } = params;
  const listen = params.connect && !params.listen ? false : true;
  const is_server = !params.client || params.server;

  let name = process.env['NAME'] ?? base;

  name = name
    .replace(/.*\//, '')
    .replace(/-/g, ' ')
    .replace(/\.[^\/.]*$/, '');

  let [prefix, suffix] = name.split(' ');

  let repl = new REPL(`\x1b[38;5;165m${prefix} \x1b[38;5;39m${suffix}\x1b[0m`, false);
  const histfile = '.upload-server-history';
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
    'import module',
  ];

  let { log } = console;
  repl.show = arg => (typeof arg == 'string' ? arg : inspect(arg, globalThis.console.options) + '\n');

  repl.cleanup = () => {
    repl.readlineRemovePrompt();
    Terminal.mousetrackingDisable();
    let numLines = repl.historySave(histfile);

    repl.printStatus(`EXIT (wrote ${numLines} history entries)`, false);

    std.exit(0);
  };

  console.options = Object.assign(
    repl.inspectOptions,
    { ...console.options },
    {
      depth: Infinity,
      compact: 1,
    },
  );
  repl.inspectOptions.hideKeys.push(Symbol.inspect);

  console.log = (...args) => repl.printStatus(() => log(console.config(repl.inspectOptions), ...args));

  let logFile =
    {
      puts(s) {
        repl.printStatus(() => std.puts(s));
      },
    } ?? std.open('upload-server.log', 'w+');

  let connections = new Set();
  let by_uuid = (globalThis.by_uuid = {});

  function ParseBody(gen) {
    let prom,
      o = '',
      x;

    while((x = gen.next())) {
      let { value, done } = x;
      o += value;
    }
    return o;
  }

  const createWS = (globalThis.createWS = (url, callbacks) => {
    //console.log('createWS', { url, callbacks, listen });

    globalThis.out = s => logFile.puts(s + '\n');

    logLevel = params.debug ? LLL_USER : 0 /*| (((params.debug ? LLL_INFO : LLL_WARN) << 1) - 1)*/;
    console.log('createWS', { logLevel }, createServer);

    return createServer({
      block: false,
      tls: params.tls,
      sslCert,
      sslPrivateKey,
      mimetypes: [
        ['.svgz', 'application/gzip'],
        ['.mjs', 'application/javascript'],
        ['.js', 'application/javascript'],
        ['.wasm', 'application/wasm'],
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
        ['/', '.', 'directory.html'],
        ['/get', './uploads', ''],
        ['/warmcat', 'https://warmcat.com/', 'index.html'],
        ['/distrelec', 'https://www.distrelec.ch/', 'login'],
        ['/hasura', 'http://wild-beauty.herokuapp.com/v1/', 'graphql'],
        // ['/upload', 'lws-deaddrop', null, 'lws-deaddrop'],
        async function* test(req, resp) {
          resp.type = 'text/plain';

          console.log('*test', { req, resp });

          let bodyStr = '';
          if(req.method == 'POST') {
            for await(let chunk of await req.body) {
              console.log('chunk', chunk);
              bodyStr += toString(chunk);
            }
          }
          console.log('bodyStr', bodyStr);

          for(let i = 0; i < 10; i++) {
            yield `line #${i}\n`;
            await waitFor((10 - i + 1) * 10);
          }

          yield 'done!';
        },
        function proxy(req, resp) {
          console.log('proxy', { req, resp });
          const { url, method, headers } = req;
          console.log('proxy', { url, method, headers });
          const { status, ok, type } = resp;

          console.log('proxy', { status, ok, url, type });
        },
        function* config(req, resp) {
          console.log('*config', { req, resp });
          yield '{}';
        },
        function* file(req, resp) {
          let { body, headers, json, url } = req;
          let { query } = url;
          console.log('*file', { req, resp });

          if(typeof body == 'string')
            query = {
              ...query,
              ...(JSON.parse(body) ?? {}),
            };

          let { action = 'list', charset = 'utf-8', binary = false, file, contents } = query ?? {};

          if(/^\/file\/./.test(url.path)) {
            file = url.path.replace(/^\/file\//, '');
            action = 'load';
          }
          console.log('*file', { query, body, file });

          console.log(`*file`, { action, file });

          let dir = path.dirname(file);
          let base = path.basename(file);

          if(file) {
            file = path.absolute(file);
            file = path.normalize(file);
            console.log(
              `allowedDirs:`,
              allowedDirs, //.map(dir => path.normalize(dir))
            );

            let allowed = [...allowedDirs.values()].map(path.absolute).some(dir => isin(file, dir));

            if(!allowed) {
              console.log(`Not allowed: '${file}'`);
              throw new Error(`Not allowed: '${file}'`);
            }
          }

          dir = allowedDirs.get(dir);

          file = path.join(dir, base);

          switch (action) {
            case 'load':
              let mime = GetMime(file);
              console.log(`*file.load`, { mime });

              let data = ReadFile(file, true);

              console.log(`*file.load`, { data });

              yield data;

              /*              if(resp) {
                resp.type = mime;
                resp.headers = { 'content-type': mime };
                resp.body = data;
              } else {
                yield new Response(data, { type: mime, headers: { 'content-type': mime } });
              }*/

              break;

            case 'save':
              WriteFile(file, contents);
              yield 'done!\r\n';
              break;

            case 'list':
              const { dir = 'uploads', mask = TYPE_MASK, filter = '*' } = query ?? {};
              const d = path.absolute(dir);
              const allowed = path.isRelative(dir) && [...allowedDirs.values()].map(path.absolute).some(x => isin(d, x));

              if(!allowed) {
                console.log('NOT ALLOWED');

                if(resp) resp.status = 403;

                yield 'ERROR\r\n';

                //yield globalThis.resp = new Response('ERROR', { status: 403 });
                break;
              }

              const it = new Directory(dir, BOTH, +mask);

              for(let [file, type] of it) if(0 == path.fnmatch(filter, file)) yield `${file}${type == TYPE_DIR ? '/' : ''}\n`;

              break;

              let component = h(
                HTMLPage,
                {
                  title: 'File list',
                  style: `body, * { font-family: MiscFixedSC613,Fixed,"Courier New"; }`,
                  scripts: ['filelist.js'],
                },
                [
                  h(FilterForm, {}, []),
                  h(FileTable, {
                    files: files.map(file => ({
                      file,
                      stat: fs.statSync(file),
                    })),
                  }),
                ],
              );
              yield renderToString(component);
              break;
              yield files.join('\n');

              break;
            default:
              throw new Error(`No such command: '${action}'`);
          }
        },
        /* function* uploads(req, resp) {
          if(resp && resp?.type) resp.type = 'application/json';

          console.log('uploads', req, resp);
          const { limit = '0,100', pretty = 0 } = req.url.query ?? {};
          let [start, end] = limit.split(/,/g).map(s => +s);
          console.log('uploads', { start, end });
          let result = [],
            entries = glob('uploads/*.json');
          for(let entry of (Array.isArray(entries) ? entries : []).slice(start, end)) {
            let json = ReadJSON(entry);
            const { storage, filename, uploaded, address, exif, thumbnail } = json;
            json.size = fs.sizeSync(storage);
            result.push(json);
          }

          console.log('uploads', console.config({ depth: 1, compact: 2, maxArrayLength: 10 }), result);
          yield JSON.stringify(result, ...(+pretty ? [null, 2] : []));
        },*/
        async function* files(req, resp) {
          const { url, method, body } = req;
          console.log('*files', { body });
          console.log('*files query =', url.query);
          const { filter = '*', root, type = TYPE_DIR | TYPE_REG | TYPE_LNK, limit = '0' } = url.query ?? {};

          console.log('*files', { root, filter, type });

          const [offset = 0, size = Infinity] = limit.split(',').map(n => +n);

          console.log('*files', { offset, size });
          let i = 0;
          let f = Matcher(filter);
          if(!root) {
            for(let name of allowedDirs.keys().filter(f)) yield name + '/\r\n';
          } else {
            for(let [key, value] of allowedDirs.entries().filter(KeyOrValueMatcher(root))) {
              let dir = new Directory(value, BOTH, +type);
              yield key + ':\r\n';

              console.log('dir', dir, keys(dir, 0, 2), dir + '');

              for(let [name, type] of dir) if(f(name)) yield name + (+type == TYPE_DIR ? '/' : '') + '\r\n';
            }
          }
          console.log('*files', { i, f });
        },
        function* files2(req, resp) {
          let { body, headers, json, url } = req;
          let { query } = url;
          define(globalThis, { filesRequest: { req, resp, body, query } });
          console.log('*files', { req, resp, body, query });
          const data = query ?? {};
          // XXX: resp.type = 'application/json';
          let { dirs = defaultDirs, filter = '[^.].*' ?? '.(brd|sch|G[A-Z][A-Z])$', verbose = false, objects = true, key = 'mtime', limit = null, flat = false } = data ?? {};
          let results = [];
          for(let dir of dirs) {
            let st,
              names = [];
            if(Array.isArray(dir)) {
              let [, re] = dir;
              let absdir = path.realpath(dir[0]);
              names = [...RecursiveDirIterator(absdir, n => re.test(n))];
              dir = path.relative(absdir, path.getcwd());
            } else if((st = fs.statSync(dir)) && st.isDirectory()) {
              let absdir = path.realpath(dir);
              let components = absdir.split(path.sep);
              if(components.length && components[0] === '') components.shift();
              if(components.length < 2 || components[0] != 'home') throw new Error(`Access error`);
              names = fs.readdirSync(absdir) ?? [];
              dir = path.relative(absdir, path.getcwd());
            } else {
              names = glob(dir, GLOB_BRACE);
              if(!Array.isArray(names)) names = [];
              let a = path.toArray(dir);
              let i = a.findIndex(n => /[*{}]/.test(n));
              dir = path.slice(dir, 0, i);
              names = names.map(n => n.slice(dir.length + 1));
            }
            if(!Array.isArray(names)) continue;
            names = names.sort((a, b) => '' + b < '' + a);
            if(filter) {
              const re = new RegExp(filter, 'gi');
              names = names.filter(name => re.test(name));
            }
            if(limit) {
              let [offset = 0] = limit;
              let [, length = names.length - start] = limit;
              names = names.slice(offset, offset + length);
            }
            let entries = names
              .map(file => (fs.existsSync(`${dir}/${file}`) ? `${dir}/${file}` : file))
              .map(file => [file, path.relative(file, path.getcwd())])
              .map(([file, rel]) => [file, fs.statSync(rel)]);
            entries = entries.reduce((acc, [file, st]) => {
              let name = file + (st && st.isDirectory() ? '/' : '');
              let obj = {
                name,
              };
              acc.push([
                name,
                Object.assign(
                  obj,
                  st
                    ? {
                        mtime: toUnixTime(st.mtime),
                        time: toUnixTime(st.ctime),
                        mode: `0${(st.mode & 0x09ff).toString(8)}`,
                        size: st.size,
                      }
                    : {},
                ),
              ]);
              return acc;
            }, []);
            if(entries.length) {
              let cmp = {
                string(a, b) {
                  return a[1][key].localeCompare(b[1][key]);
                },
                number(a, b) {
                  return a[1][key] - b[1][key];
                },
              }[typeof entries[0][1][key]];
              entries = entries.sort(cmp);
            }
            names = entries.map(([name, obj]) => (objects ? obj : name));
            if(names.length > 0) {
              if(flat) names.map(({ name }) => results.push({ name: path.normalize(path.join(dir, name)) }));
              else results.push({ dir, names });
            }
          }
          const s = JSON.stringify(...[results, ...(verbose ? [null, 2] : [])]);
          console.log('files2 reply length:', s.length);
          console.log('files2 reply:', s.slice(-100));
          yield s;
        },
      ],
      ...url,

      ...callbacks,
      onConnect(ws, req) {
        const { peer, address, port, protocol, tls } = ws;

        console.log('\x1b[38;5;33monConnect\x1b[0m', { address, port, protocol, tls });
        define(ws, {
          sendCommand(data) {
            if(!isArrayBuffer(data) /*&& isObject(data)*/) data = JSON.stringify(data);

            return this.send(data);
          },
        });

        if(!ws.uuid) {
          let data = (ws.uuid = MakeUUID());

          ws.sendCommand({ type: 'uuid', data });
          by_uuid[data] = ws;
        }
        connections.add(ws);

        if(callbacks.onConnect) return callbacks.onConnect(ws, req);
      },
      onClose(ws, code, reason) {
        console.log('\x1b[1;31monClose\x1b[0m', { code, reason });

        connections.delete(ws);

        return callbacks.onClose(ws, reason);
      },

      onRequest(req, resp) {
        if(DEBUG) console.log('onRequest', console.config({ compact: 0 }), req, resp);
        const ws = this;

        const { address } = ws;

        define(globalThis, { req, resp, address });

        const { method, headers } = req;
        if(resp && resp.headers) resp.headers['Server'] = 'upload-server';

        if(DEBUG) console.log('onRequest', { headers: Object.fromEntries(headers.entries()) });

        //
        if(globalThis.onRequest) globalThis.onRequest(req, resp);

        if((req.url.path ?? '').endsWith('files')) {
          return;
          //resp.type = 'application/json';
        } else if(req.method != 'GET' && (req.headers.get('content-type') == 'application/x-www-form-urlencoded' || (req.headers.get('content-type') ?? '').startsWith('multipart/form-data'))) {
          let fp,
            hash,
            tmpnam,
            ext,
            progress = 0;
          if(req.url.path.endsWith('upload')) resp.status = 200;

          resp.headers['content-type'] = 'text/raw';

          console.log(req.method, headers);

          const ws = (globalThis.ws = this);
          fp = new FormParser(ws, ['files', 'uuid'], {
            chunkSize: 8192 /** 256*/,
            onOpen(name, filename) {
              console.log(`onOpen(${name}, ${filename})`);
              if(this.file) {
                this.onclose.call(this, name);
              }

              this.name = name;
              this.filename = filename;
              ext = path.extname(filename).toLowerCase();

              this.file = fs.openSync((this.temp = 'uploads/' + (tmpnam = randStr(20) + '.tmp')), 'w+', 0o644);
              hash = new Hash(Hash.TYPE_SHA1);
            },
            onContent(name, data) {
              // console.log(`onContent(${this.filename})`,data.byteLength);
              progress += data.byteLength;

              let ws2 = by_uuid[ws.uuid ?? this.uuid];

              fs.writeSync(this.file, data);
              hash.update(data);
              if(ws2)
                ws2.sendCommand({
                  type: 'progress',
                  done: progress,
                  name,
                  filename: this.filename,
                  total: +headers['content-length'],
                });
            },

            onClose(name, file) {
              try {
                console.log(`onClose[1](${name}, ${file})`, this.uuid);
                let exif, cache, sha1;
                if(hash) {
                  hash.finalize();
                  sha1 = hash.toString();
                }
                if(this.file) {
                  fs.closeSync(this.file);
                  this.file = null;
                }
                if(sha1) {
                  let f = x => 'uploads/' + sha1 + x;
                  let ret = os.rename(this.temp, (this.temp = f(ext)));
                  let { errno } = error();
                  let json = f('.json');

                  if(fs.existsSync(json) && (cache = ReadJSON(json))) {
                    exif = cache.exif;
                  } else {
                    worker.postMessage({ command: 'PostUpload', args: [sha1, this.filename, this.temp, address] }).then(result => {
                      console.log('PostUpload', { ws, result });
                      if(connections[0]) connections[0].sendCommand({ type: 'upload', ...result });
                    });
                    return;

                    /* if(!/(png|svg|gif|tga)$/i.test(ext)) {
                      try {
                        exif = ReadExiftool(f(ext));
                      } catch(e) {
                        try {
                          exif = ReadExiftool(this.temp);
                        } catch(e) {}
                      }
                    }

                    let obj = {
                      filename: this.filename,
                      storage: f(ext),
                      uploaded: Date.now(),
                      address,
                      exif
                    };

                    if(/\.hei[fc]$/gi.test(ext)) {
                      HeifConvert(f(ext), f('.jpg'));
                      if(fs.existsSync(f('.jpg'))) obj.jpg = f('.jpg');
                    }

                    let width = '',
                      height = '256';

                    if(exif) {
                      const { ImageSize, ImageHeight, ImageWidth } = exif;
                      let aspect = ImageWidth / ImageHeight;
                      if(aspect >= 1) {
                        width = 256;
                        height = width / aspect;
                      } else {
                         height = 256;
                          width = height * aspect;
                      }
                    }

                    MagickResize(obj.jpg ?? f(ext), f('.thumb.jpg'), obj.exif?.Rotation ?? 0, width, height);

                    if(fs.existsSync(f('.thumb.jpg'))) obj.thumbnail = f('.thumb.jpg');

                    WriteJSON(json, obj);
 
                    console.log(`uuid`, ws.uuid ?? this.uuid);
                    cache = obj;*/
                  }

                  if(ret == 0 || errno == 17) {
                    unlink(this.temp);
                    this.temp = null;
                  }
                }

                const { filename } = this;
                let ws2 = by_uuid[ws.uuid ?? this.uuid];

                if(ws2) ws2.sendCommand({ type: 'upload', ...(cache ?? {}), filename, exif });

                //console.log(`onClose[2](${name}, ${file})`);
              } catch(e) {
                console.log(`onClose ERROR:`, e.message);
              }
            },
            onFinalize() {
              console.log(`onFinalize() form parser`, this.uuid);
              resp.write(`${this.temp}\r\n`);
              resp.write(`done: ${progress} bytes read\r\n`);
              resp.finish();
            },
          });
        }

        //throw new Error(`http ${method} request`);

        if(!req.headers || typeof req.headers != 'object') console.log('No headers', req);

        const { body, url } = resp ?? {};
        const { referer } = req.headers;

        if(url) {
          let file = url.path.slice(1);
          const dir = path.dirname(file);

          if(file.endsWith('.txt') || file.endsWith('.html') || file.endsWith('.css')) {
            resp.body = ReadFile(file);
          } else if(file.endsWith('.js')) {
            let file1 = file;
            if(/qjs-modules\/lib/.test(file) && !/(dom|util)\.js/.test(file)) {
              let file2 = file.replace(/.*qjs-modules\//g, '');
              if(fs.existsSync(file2)) {
                file = file2;
              }
            } else if(!fs.existsSync(file)) {
              for(let dir of ['quickjs/qjs-modules', 'quickjs/qjs-modules/lib', '.', 'lib']) {
                let file2 = dir + '/' + file;
                console.log('inexistent file', file, file2, fs.existsSync(file2), referer);
                if(fs.existsSync(file2)) {
                  file = file2;
                  break;
                }
              }
            }

            if(file1 != file) {
              //  console.log('\x1b[38;5;214monRequest\x1b[0m', file1, '->', file);
              resp.status = 302;
              resp.headers = { ['Location']: '/' + file };
              return resp;
            }
            //console.log('\x1b[38;5;33monRequest\x1b[0m', file1, file);

            //
            let body = ReadFile(file);

            const re = /^(\s*(im|ex)port[^\n]*from ['"])([^./'"]*)(['"]\s*;[\t ]*\n?)/gm;

            resp.body = (body ?? '').replaceAll(re, (match, p1, p0, p2, p3, offset) => {
              if(!/[\/\.]/.test(p2)) {
                let fname = `${p2}.js`;
                let rel = path.relative(fname, dir);
                //console.log('onRequest', { match, fname }, rel);

                // if(!fs.existsSync(  rel)) return ``;

                match = [p1, rel, p3].join('');

                //console.log('args', { match, p1, p2, p3, offset });
              }
              return match;
            });
          }
        }

        return resp;
      },
      onMessage(ws, data) {
        //if(typeof data != 'string') data = toString(data);

        console.log(`onMessage [${ws.uuid}]:`, data);
        return callbacks.onMessage(ws, data);
      },
      ...(url && url.host ? url : {}),
    });
  });

  define(globalThis, {
    get connections() {
      return [...connections];
    },
    get by_uuid() {
      return by_uuid;
    },
    uuid(data) {
      return by_uuid[data];
    },
  });

  globalThis.serv = new RPCServer(
    FactoryEndpoint(
      {
        Directory,
        Match,
      },
      params.verbose,
    ),
    params.verbose,
  );

  Object.assign(globalThis, {
    rpc: {
      MessageReceiver,
      MessageTransmitter,
      MessageTransceiver,
      codecs,
      RPCApi,
      RPCProxy,
      RPCObject,
      RPCFactory,
      Connection,
      RPC_PARSE_ERROR,
      RPC_INVALID_REQUEST,
      RPC_METHOD_NOT_FOUND,
      RPC_INVALID_PARAMS,
      RPC_INTERNAL_ERROR,
      RPC_SERVER_ERROR_BASE,
      FactoryEndpoint,
      RPCServer,
      RPCClient,
      FactoryClient,
      RPCSocket,
      GetProperties,
      GetKeys,
      SerializeValue,
      DeserializeSymbols,
      DeserializeValue,
      RPCConnect,
      RPCListen,
    },
    repl,
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
    dateToObject,
    Hash,
    FormParser,
    ExecTool,
    extendGenerator,
    extendArray,
    extendAsyncGenerator,
    Matcher,
    ExecTool,
    GetMime,
    Matcher,
    KeyOrValueMatcher,
    GetRootDirectories,
    GetDir,
    DirName,
    DateStr,
    ModeStr,
    ReadExiv2,
    ReadExiftool,
    HeifConvert,
    MagickResize,
    Directory,
  });

  delete globalThis.DEBUG;

  createWS(
    { protocol: 'ws', host: '0.0.0.0', port: 8999 },
    {
      onFd(fd, rd, wr) {
        //console.log('onFd', fd, rd, wr);
        os.setReadHandler(fd, rd);
        os.setWriteHandler(fd, wr);
      },
      onClose(ws, reason) {},
      onMessage(ws, data) {
        const msg = JSON.parse(data);

        console.log('onMessage', msg);

        switch (msg.type) {
          case 'uuid':
            break;

          default:
            const result = serv.processMessage(msg);
            result.type = msg.type;
            console.log('send', result);
            ws.send(JSON.stringify(result));
            break;
        }
      },
    },
    true,
  );

  function quit(why) {
    console.log(`quit('${why}')`);

    let cfg = { inspectOptions: console.options };
    WriteJSON(`.${base}-config`, cfg);
    repl.cleanup(why);
  }

  repl.run();
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error?.message ?? error}\n${error?.stack}`);

  std.exit(1);
} finally {
}
