#!/usr/bin/env qjsm
/*
 * eagle-server.js - backend of the EAGLE renderer (index.html + main.js).
 *
 * Runs unchanged on QuickJS (qjsm with qjs-lws) and on Bun: everything
 * sits on the Bun.serve() shape (fetch handler, server.upgrade(),
 * server.publish()).
 *
 *   qjsm eagle-server.js [-p PORT] [-a ADDRESS] [-d DIR]...
 *   bun  eagle-server.js [-p PORT] [-a ADDRESS] [-d DIR]...
 *
 * Environment: PORT, EAGLE_DIRS (colon separated project directories).
 * The nginx on transistorisiert.ch mounts it at /eagle/ (port 3000), so
 * every URL the client uses is relative and works under any prefix.
 *
 * Endpoints (what main.js and commands.js call):
 *   GET  /                          redirect to index.html
 *   GET  /index.html                with <TS> replaced by the unix time
 *   GET  /config   POST /config     client settings document
 *   GET  /files?filter=RE           { files: [ { name, dir, mtime, ... } ] }
 *   POST /files    { filter, names, limit }   [ { name, dir, mtime, ... } ]
 *   GET  /file?action=load&file=N   a project file (.brd .sch .lbr ...)
 *   POST /save                      store the body as tmp/<filename>
 *   POST|GET /gerber                EAGLE board -> Gerber / Excellon
 *   POST|GET /gcode                 Gerber -> G-code with pcb2gcode
 *   POST /contours                  broadcast to the websocket clients
 *   WS   /ws                        PING/PONG hub, receives CONTOURS
 *   GET  /list-serial               serial devices
 *   GET  /static/*  /tmp/*  /data/*  /vfs/*  and any file below the root
 *
 * Not carried over from server.js: the /github proxy and the /serial
 * websocket bridge (no caller left in the client).
 *
 * eagle and pcb2gcode run through the shell and block the server while
 * they work (qjsm has no async child_process); with one operator at a
 * time that is fine.
 */
import * as fs from 'fs';
import * as path from 'path';
import process from 'process';
import { execSync } from 'child_process';

const serve = globalThis.Bun?.serve;

if(!serve) throw new Error('Bun.serve() is not available (qjsm needs qjs-lws in its module path)');

const root = path.resolve(import.meta.dirname ?? path.dirname(import.meta.url.replace(/^file:\/\//, '')));
const args = process.argv.slice(2);
const opt = (short, def) => {
  const i = args.indexOf(short);

  return i == -1 ? def : args[i + 1];
};
const port = +(opt('-p', process.env.PORT ?? 3000));
const hostname = opt('-a', '127.0.0.1');
const cliDirs = args.flatMap((a, i) => (a == '-d' ? [args[i + 1]] : []));
const envDirs = (process.env.EAGLE_DIRS ?? 'data:../an-tronics/eagle:../insider/eagle:../lc-meter/eagle:../pictest/eagle').split(':');
const mountDirs = (cliDirs.length ? cliDirs : envDirs).map(d => path.resolve(root, d));
const tmpDir = path.join(root, 'tmp');
const configFile = path.join(root, '.eagle-config.json');
const projectPattern = '.*\\.(brd|sch|lbr|GBL|GTL|GKO|ngc)$';
const maxBody = 64 * 1024 * 1024;

const mime = {
  html: 'text/html; charset=utf-8',
  htm: 'text/html; charset=utf-8',
  js: 'text/javascript; charset=utf-8',
  mjs: 'text/javascript; charset=utf-8',
  css: 'text/css; charset=utf-8',
  json: 'application/json',
  svg: 'image/svg+xml',
  png: 'image/png',
  jpg: 'image/jpeg',
  jpeg: 'image/jpeg',
  gif: 'image/gif',
  ico: 'image/x-icon',
  wasm: 'application/wasm',
  woff: 'font/woff',
  woff2: 'font/woff2',
  ttf: 'font/ttf',
  eot: 'application/vnd.ms-fontobject',
  xml: 'application/xml',
  brd: 'application/xml',
  sch: 'application/xml',
  lbr: 'application/xml',
  txt: 'text/plain; charset=utf-8',
  ngc: 'text/plain; charset=utf-8',
  gbl: 'text/plain; charset=utf-8',
  gtl: 'text/plain; charset=utf-8',
  gko: 'text/plain; charset=utf-8',
  md: 'text/plain; charset=utf-8',
};

const log = (...a) => process.stderr.write(a.join(' ') + '\n');
const mimeOf = file => mime[path.extname(file).slice(1).toLowerCase()] ?? 'application/octet-stream';
const unixTime = ms => Math.floor(ms / 1000);

/* FNV-1a, enough to tell two config texts apart */
function Hash(str) {
  let h = 0x811c9dc5;

  for(let i = 0; i < str.length; i++) h = Math.imul(h ^ str.charCodeAt(i), 0x01000193) >>> 0;

  return h.toString(16).padStart(8, '0');
}

const Json = (obj, status = 200) => new Response(JSON.stringify(obj), { status, headers: { 'content-type': 'application/json' } });
const Text = (text, status = 200, type = 'text/plain; charset=utf-8') => new Response(text, { status, headers: { 'content-type': type } });

/* a decoded request path or file name as a safe relative path, or null */
function Safe(rel) {
  let p;

  try {
    p = decodeURIComponent(rel);
  } catch(e) {
    return null;
  }

  if(p.includes('\0')) return null;

  p = path.normalize('/' + p).slice(1);

  return p.split('/').some(s => s == '..') ? null : p;
}

const IsFile = p => {
  try {
    return fs.statSync(p).isFile();
  } catch(e) {
    return false;
  }
};

/* ---- project files -------------------------------------------------- */

/* { relative name: directory } of all files below the mount dirs matching the pattern */
function DirMap(pattern = projectPattern) {
  const re = new RegExp(pattern);
  const map = {};

  function Walk(base, rel, depth) {
    let names;

    try {
      names = fs.readdirSync(path.join(base, rel));
    } catch(e) {
      return;
    }

    for(const name of names) {
      const r = rel ? rel + '/' + name : name;
      let st;

      try {
        st = fs.statSync(path.join(base, r));
      } catch(e) {
        continue;
      }

      if(st.isDirectory()) depth < 4 && Walk(base, r, depth + 1);
      else if(re.test(name) && !(r in map)) map[r] = base;
    }
  }

  for(const dir of mountDirs) Walk(dir, '', 0);

  return map;
}

function Describe(dir, name) {
  try {
    const st = fs.statSync(path.join(dir, name));

    return { name, dir: path.relative(root, dir) || '.', mtime: unixTime(st.mtimeMs), time: unixTime(st.ctimeMs ?? st.mtimeMs), mode: '0' + (st.mode & 0o777).toString(8), size: st.size };
  } catch(e) {
    return null;
  }
}

function FilesList({ filter = projectPattern, names, limit = '' } = {}) {
  let list;

  if(Array.isArray(names) && names.length) {
    /* the client asks for derived files (svg renderings): they live in tmp/ */
    const wanted = new Set(names.map(n => String(n).replace(/.*\//, '')));

    list = [...wanted].map(n => Describe(tmpDir, n));
  } else {
    const map = DirMap(filter);

    list = Object.keys(map).sort().map(n => Describe(map[n], n));
  }

  list = list.filter(Boolean);

  if(limit !== '') {
    const parts = String(limit).split(/[^0-9]+/).filter(Boolean).map(Number);
    const [start, count] = parts.length == 1 ? [0, parts[0]] : parts;

    list = list.slice(start, start + count);
  }

  return list;
}

function FindProjectFile(name) {
  const rel = Safe(name);

  if(!rel) return null;

  for(const dir of mountDirs) if(IsFile(path.join(dir, rel))) return path.join(dir, rel);

  return null;
}

/* ---- running tools -------------------------------------------------- */

const Q = s => "'" + String(s).replace(/'/g, `'\\''`) + "'";

/* run a command in the server root, merge stderr into stdout; returns { code, output } */
function Run(argv) {
  const cmd = argv.map(Q).join(' ');
  const logFile = path.join(tmpDir, `.run-${process.pid}.log`);
  let code = 0;

  fs.mkdirSync(tmpDir, { recursive: true });

  try {
    const r = execSync(`cd ${Q(root)} && ${cmd} > ${Q(logFile)} 2>&1 < /dev/null`);

    code = r?.status ?? 0;
  } catch(e) {
    code = e.status ?? e.code ?? 1;
  }

  const output = (fs.existsSync(logFile) ? fs.readFileSync(logFile, 'utf8') : '').replace(/\s*\r*\n/g, '\n');

  fs.rmSync(logFile, { force: true });

  return { code, output, cmd };
}

const ReadText = file => fs.readFileSync(file, 'utf8');
const Truthy = v => v === true || v === 1 || /^(1|true|yes|on)$/i.test(String(v));

/* a board given as 'dir/name.brd' or 'name.brd' -> absolute path below a mount dir */
function BoardPath(board) {
  if(typeof board != 'string') return null;

  const direct = FindProjectFile(board);

  if(direct) return direct;

  for(const dir of mountDirs) {
    const rel = path.relative(dir, path.resolve(root, board));

    if(!rel.startsWith('..') && IsFile(path.join(dir, rel))) return path.join(dir, rel);
  }

  return null;
}

function ConvertToGerber(opts) {
  const board = BoardPath(opts.board);

  if(!board) return { error: `no such board: ${opts.board}` };

  const side = opts.side;
  const drill = Truthy(opts.drill) || side == 'drill';
  const front = opts.front !== undefined ? Truthy(opts.front) : side == 'front';
  const layers = Array.isArray(opts.layers) && opts.layers.every(l => /^[\w.+-]+$/.test(l)) ? opts.layers : side == 'outline' ? ['Measures'] : drill ? ['Drills', 'Holes'] : [front ? 'Top' : 'Bottom', 'Pads', 'Vias'];
  const format = /^[A-Z0-9_]+$/.test(opts.format ?? '') ? opts.format : drill ? 'EXCELLON' : 'GERBER_RS274X';
  const ext = drill || format.startsWith('EXCELLON') ? 'TXT' : side == 'outline' ? 'GKO' : front ? 'GTL' : 'GBL';
  const file = `./tmp/${path.basename(board, path.extname(board))}.${ext}`;
  const { code, output } = Run(['eagle', '-X', '-d', format, '-o', file, board, ...layers]);
  const result = { code, output, file };

  if(code !== 0) return { ...result, error: output || `eagle exited with ${code}` };

  if(Truthy(opts.fetch) && IsFile(path.join(root, file))) result.data = ReadText(path.join(root, file));

  return result;
}

function GerberToGcode(opts) {
  const { file, fetch, data, raw, ...rest } = opts;
  const rel = Safe(String(file ?? '').replace(/^\.?\/?(tmp\/)?/, ''));

  if(!rel) return { error: `bad gerber file: ${file}` };

  const gerber = path.join(tmpDir, rel);
  const base = path.basename(gerber).replace(/\.[^.]*$/, '');
  const options = {
    basename: base,
    zsafe: '1mm',
    zchange: '2mm',
    zwork: '-1mm',
    zdrill: '-2mm',
    zcut: '-2mm',
    'cutter-diameter': '1mm',
    'drill-feed': 1000,
    'drill-speed': 10000,
    'mill-feed': 600,
    'mill-speed': 16000,
    'cut-feed': 200,
    'cut-speed': 10000,
    'cut-infeed': '1mm',
    ...rest,
    'output-dir': './tmp/',
  };

  if(options.front === undefined && options.back === undefined && options.drill === undefined) options.back = gerber;

  const sides = [];

  for(const side of ['front', 'back', 'drill', 'outline'])
    if(side in options) {
      if(typeof options[side] != 'string') options[side] = gerber;
      sides.push(side);
    }

  if(options.voronoi && !options.vectorial) options.vectorial = 1;

  const params = Object.entries(options)
    .filter(([k, v]) => /^[a-z][a-z0-9-]*$/i.test(k) && (typeof v == 'number' || (typeof v == 'string' && /^[\w .+\/:=,~-]*$/.test(v)) || v === true))
    .map(([k, v]) => `--${k}${v === true || v === '' ? '' : '=' + v}`);
  const { code, output, cmd } = Run(['pcb2gcode', ...params]);
  const makePath = (ext, side, name = base) => path.join('tmp', `${name}_${side}.${ext}`);

  /* pcb2gcode names the rendering processed.svg: keep one per side */
  for(const side of sides) {
    const from = path.join(root, makePath('svg', side, 'processed'));

    if(fs.existsSync(from)) fs.renameSync(from, path.join(root, makePath('svg', side)));
  }

  const files = Object.fromEntries(sides.map(side => [side, makePath('ngc', side)]).filter(([, f]) => fs.existsSync(path.join(root, f))));
  const result = { code, output: output.length > 400 ? output.slice(0, 400) + '…' : output, cmd, files };

  if(Truthy(fetch)) for(const [side, f] of Object.entries(files)) result[side] = ReadText(path.join(root, f));

  if(!Object.keys(files).length) result.error = output || `pcb2gcode exited with ${code}`;

  return result;
}

/* ---- request handlers ----------------------------------------------- */

/* the query of a request URL; parsed by hand because qjs-modules' URL.search throws on unencoded ( ) | $, which the client sends */
const Query = url => {
  const q = {};
  const raw = String(url.href ?? url);
  const i = raw.indexOf('?');

  if(i == -1) return q;

  for(const part of raw.slice(i + 1).replace(/#.*$/, '').split('&')) {
    if(!part) continue;

    const eq = part.indexOf('=');
    const dec = s => {
      try {
        return decodeURIComponent(s.replace(/\+/g, ' '));
      } catch(e) {
        return s;
      }
    };

    q[dec(eq == -1 ? part : part.slice(0, eq))] = eq == -1 ? '' : dec(part.slice(eq + 1));
  }

  return q;
};

const Params = async req => {
  const url = new URL(req.url);
  const query = Query(req.url);

  if(req.method == 'GET' || req.method == 'HEAD') return query;

  const text = await req.text();

  try {
    return { ...query, ...(text ? JSON.parse(text) : {}) };
  } catch(e) {
    return query;
  }
};

async function GerberEndpoint(req) {
  const opts = await Params(req);
  const result = ConvertToGerber(opts);

  log('gerber', opts.board, result.error ?? `ok ${result.file}`);

  if((req.method == 'GET' || Truthy(opts.raw)) && result.file && !result.error) {
    const file = path.join(root, result.file);

    return new Response(fs.readFileSync(file), { headers: { 'content-type': 'application/octet-stream', 'content-disposition': `attachment; filename="${path.basename(file)}"` } });
  }

  return Json(result);
}

async function GcodeEndpoint(req) {
  const opts = await Params(req);
  const result = GerberToGcode(opts);

  log('gcode', opts.file, result.error ?? Object.values(result.files).join(' '));

  return Json(result);
}

async function SaveEndpoint(req) {
  const disp = req.headers.get('content-disposition') ?? '';
  const name = Safe((/filename="([^"]*)"/.exec(disp)?.[1] || 'output.svg').replace(/^\.?\/?(tmp\/)?/, ''));
  const body = new Uint8Array(await req.arrayBuffer());

  if(!name || name.startsWith('.')) return Json({ error: 'bad file name' }, 400);
  if(body.length > maxBody) return Json({ error: 'too large' }, 413);

  const filename = path.join('tmp', name);

  fs.mkdirSync(path.dirname(path.join(root, filename)), { recursive: true });
  fs.writeFileSync(path.join(root, filename), body);

  log('saved', filename, body.length);

  return Json({ size: body.length, filename });
}

function ConfigGet() {
  let str = '{}';
  let time = 0;

  if(IsFile(configFile)) {
    str = ReadText(configFile);
    time = fs.statSync(configFile).mtimeMs;
  }

  let config = {};

  try {
    config = JSON.parse(str);
  } catch(e) {}

  return Json({ config, time, hash: Hash(str) });
}

async function ConfigPost(req) {
  const text = await req.text();

  try {
    JSON.parse(text);
  } catch(e) {
    return Json({ error: 'not JSON' }, 400);
  }

  fs.writeFileSync(configFile, text);

  return Json({ size: text.length, time: fs.statSync(configFile).mtimeMs, hash: Hash(text) });
}

/* ---- static files ---------------------------------------------------- */

const aliases = (() => {
  try {
    const map = JSON.parse(ReadText(path.join(root, 'package.json')))._moduleAliases ?? {};
    const out = {};

    /* module aliases whose target is a bare module name (fs, deep, child_process, ...): the browser
     * cannot resolve those, so imports of the name are rewritten to the shim file (the key) */
    for(const [key, value] of Object.entries(map)) if(!/[./]/.test(value) && IsFile(path.join(root, key))) (out[value] ??= []).push(key);

    return out;
  } catch(e) {
    return {};
  }
})();

function RewriteImports(code, file) {
  const names = Object.keys(aliases);

  if(!names.length) return code;

  const re = new RegExp(`(\\bfrom\\s*)(['"])(${names.join('|')})\\2`, 'g');

  return code.replace(re, (all, from, q, name) => {
    let rel = path.relative(path.dirname(file), path.join(root, aliases[name][0]));

    if(!rel.startsWith('.')) rel = './' + rel;

    return `${from}${q}${rel}${q}`;
  });
}

function ServeFile(file, req) {
  if(!IsFile(file)) return null;

  const type = mimeOf(file);
  const headers = { 'content-type': type, 'cache-control': 'no-cache' };
  let body;

  if(/\.m?js$/.test(file)) body = RewriteImports(ReadText(file), file);
  else if(path.basename(file) == 'index.html') body = ReadText(file).replace(/<TS>/g, String(unixTime(Date.now())));
  else body = fs.readFileSync(file);

  return new Response(req.method == 'HEAD' ? null : body, { headers });
}

function Static(pathname, req) {
  const rel = Safe(pathname.replace(/^\/+/, ''));

  if(rel === null) return null;

  /* generated files: static/tmp/x, tmp/x */
  const m = /^(?:static\/)?(tmp|data|vfs)\/(.*)$/.exec(rel);

  if(m) {
    const [, kind, name] = m;

    if(kind == 'tmp') return ServeFile(path.join(tmpDir, name), req);

    const found = FindProjectFile(name) ?? FindProjectFile(name.replace(/^.*\//, ''));

    if(found) return ServeFile(found, req);
  }

  /* no dotfiles (config, logs, git) and no server sources */
  if(rel.split('/').some(s => s.startsWith('.')) || /-server\.js$/.test(rel)) return null;

  const file = path.join(root, rel || 'index.html');

  return ServeFile(file, req) ?? ServeFallback(rel, req);
}

/* paths that no longer exist in this checkout: the quickjs/qjs-* submodule directories are empty here and
 * qjs-net became qjs-lws; main.js still imports the old locations */
const fallbacks = [
  [/^quickjs\/qjs-modules\/lib\/events\.js$/, () => 'lib/events.js'],
  [/^quickjs\/qjs-net\/js\/rpc\.js$/, () => 'lib/rpc.js'],
  [/^quickjs\/qjs-modules\/(.*)$/, m => path.resolve(root, '../quickjs/qjs-modules', m[1])],
];

/* a repo-relative target is answered with a relative redirect, so that the module's own relative imports
 * resolve against its real location (and the redirect still works below a reverse proxy prefix);
 * an absolute target is served in place */
function ServeFallback(rel, req) {
  for(const [re, to] of fallbacks) {
    const m = re.exec(rel);

    if(!m) continue;

    const target = to(m);

    if(path.isAbsolute(target)) return ServeFile(target, req);

    return new Response(null, { status: 302, headers: { location: '../'.repeat(rel.split('/').length - 1) + target } });
  }

  return null;
}

/* ---- websocket hub --------------------------------------------------- */

let nextId = 1;

function WsMessage(ws, data) {
  for(const line of String(data).split('\n')) {
    const m = /^(?::(\S+) )?(\S+)(?: (.*))?$/.exec(line);

    if(m && m[2] == 'PING') ws.send(`PONG ${m[1] ?? ''}${m[1] ? ' ' : ''}${m[3] ?? ''}`.trim());
  }
}

/* ---- server ---------------------------------------------------------- */

fs.mkdirSync(tmpDir, { recursive: true });

const server = serve({
  port,
  hostname,
  maxRequestBodySize: maxBody,

  async fetch(req, srv) {
    const url = new URL(req.url);
    const { pathname } = url;
    const { method } = req;

    if(process.env.EAGLE_DEBUG) log(method, req.url);

    try {
      if(/^\/(ws|\.websocket|ws\/\.websocket)$/.test(pathname)) {
        if(srv.upgrade(req, { data: { id: nextId++ } })) return;

        return Text('websocket upgrade failed', 400);
      }

      if(method == 'OPTIONS') return new Response(null, { status: 204, headers: { allow: 'GET,POST,HEAD,OPTIONS' } });

      if(pathname == '/' && (method == 'GET' || method == 'HEAD')) return new Response(null, { status: 302, headers: { location: 'index.html' } });

      if(pathname == '/config') return method == 'POST' ? await ConfigPost(req) : ConfigGet();

      if(pathname == '/files' || pathname == '/list' || pathname == '/list.html') {
        if(method == 'POST') return Json(FilesList(await Params(req)));

        const q = Query(req.url);

        return Json({ files: FilesList({ filter: q.filter || projectPattern, limit: q.limit ?? '' }) });
      }

      if(pathname == '/file') {
        const { action, file } = Query(req.url);
        const found = action == 'load' && file ? FindProjectFile(file) : null;

        if(action != 'load') return Text('No such action!', 404);

        return (found && ServeFile(found, req)) || Text('No such file', 404);
      }

      if(pathname == '/save' && method == 'POST') return await SaveEndpoint(req);
      if(/^\/gerber\/?$/.test(pathname)) return await GerberEndpoint(req);
      if(/^\/gcode\/?$/.test(pathname)) return await GcodeEndpoint(req);

      if(pathname == '/contours' && method == 'POST') {
        const body = await Params(req);

        srv.publish('all', `:* CONTOURS * :${JSON.stringify(body)}`);

        return Text('OK');
      }

      if(pathname == '/list-serial') {
        const devices = fs
          .readdirSync('/dev')
          .filter(n => /^(ttyUSB|ttyACM|tnt)\d+$/.test(n))
          .map(n => ({ path: '/dev/' + n }));

        return Json(devices);
      }

      if(pathname == '/favicon.ico') {
        const icon = ServeFile(path.join(root, 'static/eagle.ico'), req) ?? ServeFile(path.join(root, 'lib/eagle/icon/eagleicon.ico'), req);

        if(icon) return icon;
      }

      if(method == 'GET' || method == 'HEAD') return Static(pathname, req) ?? Text('Not found', 404);

      return Text('Method not allowed', 405);
    } catch(error) {
      log('error', method, pathname, error?.message ?? error);

      return Json({ error: String(error?.message ?? error) }, 500);
    }
  },

  websocket: {
    open(ws) {
      ws.subscribe('all');
      ws.send(`HELLO ${ws.data?.id ?? 0}`);
    },
    message(ws, data) {
      WsMessage(ws, data);
    },
    close() {},
  },
});

log(`eagle-server listening on http://${hostname}:${port}/ (root ${root})`);
log(`project dirs: ${mountDirs.join(', ')}`);

export default server;
