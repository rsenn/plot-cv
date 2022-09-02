import * as std from 'std';
import * as os from 'os';
import * as deep from './lib/deep.js';
import * as xml from 'xml';
import * as path from 'path';
import { Console } from 'console';
import REPL from './quickjs/qjs-modules/lib/repl.js';
import inspect from './lib/objectInspect.js';
import * as Terminal from './terminal.js';
import * as fs from 'fs';
import { link, unlink, error } from 'misc';
import { toString, define, toUnixTime, getOpt, randStr, isObject, isNumeric, isArrayBuffer, glob, GLOB_BRACE } from 'util';
import { setLog, LLL_USER, LLL_NOTICE, LLL_WARN, client, server, FormParser, Hash } from 'net';
import { parseDate, dateToObject } from './date-helpers.js';
import { IfDebug, LogIfDebug, ReadFile, LoadHistory, ReadJSON, ReadXML, MapFile, WriteFile, WriteJSON, WriteXML, ReadBJSON, WriteBJSON, DirIterator, RecursiveDirIterator, ReadDirRecursive, Filter, FilterImages, SortFiles, StatFiles, ReadFd, FdReader, CopyToClipboard, ReadCallback, LogCall, Spawn, FetchURL } from './io-helpers.js';
import { parseDegMinSec, parseGPSLocation } from './string-helpers.js';

globalThis.fs = fs;
const MakeUUID = (rng = Math.random) => [8, 4, 4, 4, 12].map(n => randStr(n, '0123456789abcdef'), rng).join('-');

const defaultDirs = [
  './uploads/*.{sch,brd,lbr}',
  '/mnt/extext/Photos/*APPLE/*.{JPG,PNG,GIF,AAE,MOV,HEIC,MP4,WEBP}',
  ['/home/roman/Bilder', new RegExp('.(jpg|jpeg|png|heic|tif|tiff)$', 'i')]
];

function ReadExiv2(file) {
  console.log('ReadExiv2', file);
  let [rdf, stdout] = os.pipe();
  os.exec(['exiv2', '-e', 'X-', 'ex', file], { stdout });
  os.close(stdout);
  let xmpdat = fs.readAllSync(rdf);
  fs.closeSync(rdf);
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
      .map(([k, v]) => [k, isNaN(+v) ? (isNaN(Date.parse(v)) ? v : new Date(v)) : +v])
  );
  return flat;
}

function ReadExiftool(file) {
  console.log('ReadExiftool', file);
  let [rdf, stdout] = os.pipe();

  os.exec(['exiftool', '-S', '-ee', file], { stdout });

  os.close(stdout);

  let out = fs.readAllSync(rdf);
  fs.closeSync(rdf);

  let a = out.split(/\r?\n/g).filter(l => l != '');

  a = a.map(line => [line, line.indexOf(': ')]).map(([line, idx]) => [line.slice(0, idx), line.slice(idx + 2)]);
  let o = Object.fromEntries(a);

  //console.log('ReadExiftool',o);
  return o;
}

function HeifConvert(src, dst, quality = 100) {
  console.log('HeifConvert', src, dst);
  let [rd, stdout] = os.pipe();

  os.exec(['heif-convert', '-q', quality + '', src, dst], { stdout, stderr: stdout });
  os.close(stdout);

  let out = fs.readAllSync(rd);
  fs.closeSync(rd);

  console.log('HeifConvert', out);
}

function MagickResize(src, dst, rotate = 0, width, height) {
  console.log('MagickResize', { width, height, dst, rotate });
  let [rd, stdout] = os.pipe();

  os.exec(
    ['convert-im6.q16', src, '-resize', width + 'x' + height, ...(rotate ? ['-rotate', '-' + rotate] : []), dst],
    {
      stdout,
      stderr: stdout
    }
  );
  os.close(stdout);

  let out = fs.readAllSync(rd);
  fs.closeSync(rd);

  console.log('MagickResize', out);
}

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

  const {
    address = '0.0.0.0',
    port = 8999,
    'ssl-cert': sslCert = 'localhost.crt',
    'ssl-private-key': sslPrivateKey = 'localhost.key'
  } = params;
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
  let by_uuid = {};

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

  const createWS = (globalThis.createWS = (url, callbacks, listen) => {
    //console.log('createWS', { url, callbacks, listen });

    const out = s => logFile.puts(s + '\n');
    setLog((params.debug ? LLL_USER : 0) | (((params.debug ? LLL_NOTICE : LLL_WARN) << 1) - 1), (level, message) => {
      if(/__lws/.test(message)) return;
      if(/(Unhandled|PROXY-|VHOST_CERT_AGING|BIND|EVENT_WAIT|_BODY[^_])/.test(message)) return;

      if(params.debug || level <= LLL_WARN)
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
        ['.js', 'application/javascript'],
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
        // ['/upload', 'lws-deaddrop', null, 'lws-deaddrop'],
        /*function* upload(arg) {
          console.log('upload', arg);
          yield 'done!';
        },*/
        function proxy(req, res) {
          console.log('proxy', { req, res });
          const { url, method, headers } = req;
          console.log('proxy', { url, method, headers });
          const { status, ok, type } = res;

          console.log('proxy', { status, ok, url, type });
        },
        function* file(req, resp) {
          let { body, headers, json } = req;

          const data = json ? json : JSON.parse(body ?? '{}');

          let { action = 'load', charset = 'utf-8', binary = false, file, contents } = data;

          switch (action) {
            case 'load':
              yield fs.readFileSync(file, binary ? null : charset);
              break;
            case 'save':
              fs.writeFileSync(file, contents);
              yield 'done!\r\n';
              break;
          }
        },
        function* files(req, resp) {
          let { body, headers, json } = req;

          console.log('body', body);

          const data = {}; //json ? json : JSON.parse(body ?? '{}');
          resp.type = 'application/json';
          let {
            dirs = defaultDirs,
            filter = '[^.].*' ?? '.(brd|sch|G[A-Z][A-Z])$',
            verbose = false,
            objects = true,
            key = 'mtime',
            limit = null
          } = data ?? {};
          let results = [];
          for(let dir of dirs) {
            let st,
              names = [];
            if(Array.isArray(dir)) {
              let [, re] = dir;
              let absdir = path.realpath(dir[0]);
              names = [...RecursiveDirIterator(absdir, n => re.test(n))]; //.map(n => path.relative(n, absdir));
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
                name
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
                        size: st.size
                      }
                    : {}
                )
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
                }
              }[typeof entries[0][1][key]];
              entries = entries.sort(cmp);
            }
            names = entries.map(([name, obj]) => (objects ? obj : name));
            results.push({ dir, names });
          }

          // yield '\n]';
          yield JSON.stringify(...[results, ...(verbose ? [null, 2] : [])]);
        }
      ],
      ...url,

      ...callbacks,
      onConnect(ws, req) {
        const { peer, address, port } = ws;

        //  console.log('\x1b[38;5;33monConnect\x1b[0m', { address, port });

        ws.sendCommand = function(data) {
          if(!isArrayBuffer(data) /*&& isObject(data)*/) data = JSON.stringify(data);

          return this.send(data);
        };
        if(!ws.uuid) {
          let data = (ws.uuid = MakeUUID());

          ws.sendCommand({ type: 'uuid', data });
          by_uuid[data] = ws;
        }
        connections.add(ws);
        if(!req.url || req.url.path.endsWith('uploads')) {
        } else {
          return callbacks.onConnect(ws, req);
        }
      },
      onClose(ws, reason) {
        connections.delete(ws);

        return callbacks.onClose(ws, reason);
      },
      /*      onRead(data) {
         const req = this;
        console.log('onRead', { req, data }); 
      },*/
      /* onPost(data) {
       const req = this;
        try {
          req.json = JSON.parse(data);
        } catch(error) {
          console.log('onPost', { req, data, error });
        }
      },*/
      onHttp(ws, req, resp) {
        const { peer, address, port } = ws;
        const { method, headers } = req;

        if(req.url.path.endsWith('files')) {
          resp.type = 'application/json';
        } else if(req.method != 'GET') {
          let fp,
            hash,
            tmpnam,
            ext,
            progress = 0;
          console.log(req.method, headers);
          if(req.url.path.endsWith('upload')) resp.status = 200;
          resp.type = 'text/raw';

          fp = new FormParser(ws, ['files', 'uuid'], {
            chunkSize: 8192 /** 256*/,
            onOpen(name, filename) {
              if(this.file) {
                this.onclose.call(this, name);
              }

              this.name = name;
              this.filename = filename;
              ext = path.extname(filename).toLowerCase();

              this.file = fs.openSync((this.temp = 'uploads/' + (tmpnam = randStr(20) + '.tmp')), 'w+', 0o644);
              hash = new Hash(Hash.TYPE_SHA1);
              //console.log(`onOpen(${filename})`, this.temp);
            },
            onContent(name, data) {
              // console.log(`onContent(${this.filename})`,data.byteLength);
              progress += data.byteLength;

              let ws2 = by_uuid[ws.uuid ?? this.uuid];

              fs.writeSync(this.file, data);
              hash.update(data);
              if(ws2) ws2.sendCommand({ type: 'progress', done: progress, total: +headers['content-length'] });
            },

            onClose(name, file) {
              try {
                // console.log(`onClose[1](${name}, ${file})`, this.uuid);
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
                  let ret = link(this.temp, f(ext));
                  let { errno } = error();
                  let json = f('.json');
                  if(fs.existsSync(json) && (cache = ReadJSON(json))) {
                    exif = cache.exif;
                  } else {
                    if(!/(png|svg|gif|tga)$/i.test(ext)) {
                      try {
                        exif = ReadExiftool(f(ext));
                      } catch(e) {
                        try {
                          exif = ReadExiftool(this.temp);
                        } catch(e) {}
                      }
                    }
                    let obj = { filename: this.filename, storage: f(ext), uploaded: Date.now(), address, exif };
                    if(!/jpe?g$/.test(ext)) {
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
                        /* height = 256;
                        width = height * aspect;*/
                      }
                    }
                    MagickResize(obj.jpg ?? f(ext), f('.thumb.jpg'), obj.exif?.Rotation ?? 0, width, height);
                    if(fs.existsSync(f('.thumb.jpg'))) obj.thumbnail = f('.thumb.jpg');
                    WriteJSON(json, obj);
                    console.log(`by_uuid`, by_uuid);
                    console.log(`uuid`, ws.uuid ?? this.uuid);
                    cache = obj;
                  }
                  if(ret == 0 || errno == 17) {
                    unlink(this.temp);
                    this.temp = null;
                  }
                }
                const { filename } = this;
                let ws2 = by_uuid[ws.uuid ?? this.uuid];
                if(ws2) ws2.sendCommand({ type: 'upload', ...(cache ?? {}), filename, exif });
                //  console.log(`onClose[2](${name}, ${file})`);
              } catch(e) {
                console.log(`onClose ERROR:`, e.message);
              }
            },
            onFinalize() {
              console.log(`onFinalize() form parser`, this.uuid);
              resp.body = `done: ${progress} bytes read\r\n`;
            }
          });
        }

        const { body, url } = resp;
        const { referer } = req.headers;

        let file = url.path.slice(1);
        const dir = path.dirname(file); //file.replace(/\/[^\/]*$/g, '');

        if(file.endsWith('.txt') || file.endsWith('.html') || file.endsWith('.css')) {
          resp.body = fs.readFileSync(file, 'utf-8');
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
            //  console.log('\x1b[38;5;214monHttp\x1b[0m', file1, '->', file);
            resp.status = 302;
            resp.headers = { ['Location']: '/' + file };
            return resp;
          }
          //console.log('\x1b[38;5;33monHttp\x1b[0m', file1, file);

          //
          let body = fs.readFileSync(file, 'utf-8');

          const re = /^(\s*(im|ex)port[^\n]*from ['"])([^./'"]*)(['"]\s*;[\t ]*\n?)/gm;

          resp.body = body.replaceAll(re, (match, p1, p0, p2, p3, offset) => {
            if(!/[\/\.]/.test(p2)) {
              let fname = `${p2}.js`;
              let rel = path.relative(fname, dir);
              console.log('onHttp', { match, fname }, rel);

              // if(!fs.existsSync(  rel)) return ``;

              match = [p1, rel, p3].join('');

              console.log('args', { match, p1, p2, p3, offset });
            }
            return match;
          });
        }
        //console.log('\x1b[38;5;33monHttp\x1b[0m', { resp });

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
    },
    get by_uuid() {
      return by_uuid;
    },
    uuid(data) {
      return by_uuid[data];
    }
  });

  Object.assign(globalThis, {
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
    FormParser
  });

  delete globalThis.DEBUG;

  createWS(
    { protocol: 'ws', host: '0.0.0.0', port: 8999 },
    {
      onFd(fd, rd, wr) {
        os.setReadHandler(fd, rd);
        os.setWriteHandler(fd, wr);
      },
      onClose(ws, reason) {},
      onMessage(ws, data) {}
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
