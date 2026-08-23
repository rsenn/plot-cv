/**
 * project-server.js — serves source files (C & JS, syntax-highlighted) and
 * markdown (rendered to HTML) out of a fixed set of sibling project
 * directories, over plain HTTP via the raw qjs-lws (lws.so) API. Anything
 * else is served as-is, with its mime type taken from the extension or,
 * failing that, sniffed via libmagic.
 *
 * There are no real filesystem mounts: every request is dispatched through
 * a single LWSMPRO_CALLBACK mount on '/', which reads the requested file
 * itself and returns generated HTML (or the raw file).
 *
 * Run:
 *   qjs -m project-server.js [port]
 */

import { createServer, LWSMPRO_CALLBACK, LWS_WRITE_HTTP_FINAL } from 'lws';
import { glob, GLOB_MARK } from 'misc';
import { Magic } from 'magic';
import * as path from 'path';
import * as std from 'std';
import * as os from 'os';
import { render as renderMarkdown } from './quickjs/qjs-lws/tools/site/markdown.js';
import { highlight } from './lexer-highlight.js';

const PORT = +(scriptArgs[1] || 8085);

/* -------------------------------------------------------------- projects */

const PATTERNS = ['../shish', '../c-utils', '../an-tronics', '../pictest', '../lc-meter', '../USB-*', '../insider', 'qjs-*', 'quickjs', 'quickjs/qjs-*'];

/* Files/directories that are noise (VCS metadata, build output, vendored
   packages, logs, generated JSON, ...) and would otherwise dominate the file
   listing. Matched against the basename with fnmatch(3) (glob-style: '*',
   '?', '[...]'). */
const EXCLUDE_PATTERNS = ['.git', 'node_modules', 'build', '.cache', 'dist', 'CMakeFiles', '.qtc_clangd', 'autom4te.cache', 'tmp', '*.log', '*.json', '*.tmp*', '*.user*', '*.out', '*.xz', '*.es', 'try-*.c' ];

const excluded = name => EXCLUDE_PATTERNS.some(p => path.fnmatch(p, name) === 0);

const EXT_LANG = {
  '.c': 'c', '.h': 'c', '.cc': 'c', '.cpp': 'c', '.hpp': 'c', '.cxx': 'c',
  '.js': 'js', '.mjs': 'js', '.cjs': 'js', '.jsx': 'js', '.ts': 'js', '.tsx': 'js',
};

/* Matched by filename (not extension) with fnmatch(3): CMake source has no
   dedicated extension for its main file, and this catches shell/XML files
   the EXT_LANG table above doesn't (e.g. a shebang script with no .sh). */
const NAME_LANG = [
  [['CMakeLists.txt', '*.cmake', '*.cmake.*'], 'cmake'],
  [['*.sh'], 'sh'],
  [['*.xml'], 'xml'],
  [['Makefile', 'GNUmakefile', '*.mk'], 'make'],
  [['*.ini'], 'ini'],
  [['*.y', '*.l'], 'bnf'],
];

function langByName(basename) {
  for(const [patterns, lang] of NAME_LANG)
    if(patterns.some(p => path.fnmatch(p, basename) === 0)) return lang;
  return undefined;
}

/* Falls back to the mime type (extension- or libmagic-detected, see mimeFor
   below) for files langByName/EXT_LANG don't otherwise recognize. */
const MIME_LANG = { 'text/x-shellscript': 'sh', 'text/xml': 'xml', 'text/html': 'xml' };

/* Common extensions get their mime type without asking libmagic. Anything
   else falls back to content sniffing via MAGIC below. */
const EXT_MIME = {
  '.txt': 'text/plain', '.css': 'text/css', '.html': 'text/html', '.htm': 'text/html',
  '.xml': 'text/xml', '.csv': 'text/csv', '.svg': 'image/svg+xml',
  '.png': 'image/png', '.jpg': 'image/jpeg', '.jpeg': 'image/jpeg', '.gif': 'image/gif',
  '.webp': 'image/webp', '.ico': 'image/x-icon', '.bmp': 'image/bmp',
  '.pdf': 'application/pdf', '.zip': 'application/zip', '.gz': 'application/gzip',
  '.wasm': 'application/wasm', '.woff': 'font/woff', '.woff2': 'font/woff2', '.ttf': 'font/ttf',
  '.mp3': 'audio/mpeg', '.mp4': 'video/mp4', '.wav': 'audio/wav',
};

/* libmagic (file(1)) content-type detection, used when the extension alone
   doesn't say what a file is. Loaded from the system magic database. */
const MAGIC = new Magic(Magic.MIME_TYPE, '/usr/share/misc/magic.mgc');

function mimeFor(abs, ext) {
  if(EXT_MIME[ext]) return EXT_MIME[ext];
  try {
    const type = MAGIC.file(abs);
    if(type) return type;
  } catch(e) { /* fall through */ }
  return 'application/octet-stream';
}

/** Binary-safe whole-file read into an ArrayBuffer. */
function readFile(abs) {
  const [st, statErr] = os.stat(abs);
  if(statErr) return null;
  const fd = os.open(abs, os.O_RDONLY);
  if(fd < 0) return null;
  const buf = new ArrayBuffer(st.size);
  os.read(fd, buf, 0, st.size);
  os.close(fd);
  return buf;
}

/** name (basename) -> absolute project directory */
const PROJECTS = new Map();
for(const pattern of PATTERNS)
  for(const hit of glob(pattern, GLOB_MARK)) {
    if(!hit.endsWith('/')) continue; // GLOB_MARK: only directories end in '/'
    const dir = path.resolve(hit);
    PROJECTS.set(path.basename(dir), dir);
  }

/* ------------------------------------------------------------------ html */

const esc = s => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
const escAttr = s => esc(s).replace(/"/g, '&quot;');

const STYLE = `
:root { --bg:#fbfbfa; --fg:#1b1c1a; --fg-muted:#5e625c; --border:#dedcd6; --bg-code:#f6f6f4;
  --t-kw:#8a3ba8; --t-str:#0a7d43; --t-num:#b5581a; --t-cm:#8c918a; --t-fn:#1c5fbe;
  --t-type:#0b7c72; --t-atom:#b5581a; --t-op:#5e625c; --t-var:#b5581a; --t-pre:#8a3ba8; }
@media (prefers-color-scheme: dark) {
  :root { --bg:#14161a; --fg:#e6e8e3; --fg-muted:#9aa0a6; --border:#2a2e35; --bg-code:#1a1d22;
    --t-kw:#c792ea; --t-str:#9ad48a; --t-num:#f0a05a; --t-cm:#6d747b; --t-fn:#82aaff;
    --t-type:#3fd0bd; --t-atom:#f0a05a; --t-op:#9aa0a6; --t-var:#f0a05a; --t-pre:#c792ea; }
}
body { background:var(--bg); color:var(--fg); font:15px/1.5 system-ui, sans-serif; margin:0; padding:1.5rem 2rem; }
a { color:var(--t-fn); }
h1 { font-size:1.3rem; }
ul.tree { list-style:none; padding-left:1.1rem; }
ul.tree li { margin:.15rem 0; }
summary { cursor:pointer; color:var(--fg-muted); }
.count { color:var(--fg-muted); font-size:.85em; }
pre { background:var(--bg-code); border:1px solid var(--border); border-radius:6px; padding:1rem; overflow-x:auto; }
code { font-family: ui-monospace, Menlo, Consolas, monospace; font-size:.9em; }
.t-kw{color:var(--t-kw);} .t-str{color:var(--t-str);} .t-num{color:var(--t-num);} .t-cm{color:var(--t-cm);font-style:italic;}
.t-fn{color:var(--t-fn);} .t-type{color:var(--t-type);} .t-atom{color:var(--t-atom);} .t-op{color:var(--t-op);}
.t-var{color:var(--t-var);} .t-pre{color:var(--t-pre);} .t-prop,.t-id{color:inherit;}
`;

function page(title, body) {
  return `<!doctype html><html><head><meta charset="utf-8"><title>${escAttr(title)}</title>` +
    `<style>${STYLE}</style></head><body>${body}</body></html>`;
}

/** 'n' levels of '../', to reach the site root from a page 'n' directories deep.
 *  Every generated link is relative (never starts with '/') so the server works
 *  unchanged when reverse-proxied under any path prefix (e.g. /projects/). */
const up = n => n === 0 ? '.' : '../'.repeat(n);

function projectIndex() {
  const items = [...PROJECTS.keys()].sort()
    .map(name => `<li><a href="${escAttr(name)}/">${esc(name)}</a></li>`).join('');
  return page('projects', `<h1>Projects</h1><ul class="tree">${items}</ul>`);
}

const MAX_DEPTH = 20;
const MAX_FILES = 20000;

/* Directories matched here (path relative to the project root, glob-style
   via fnmatch(3)) start expanded; everything else starts collapsed. Still
   foldable either way, just a different default. */
const OPEN_DIRS = ['src/', 'include/', 'lib/', 'lib/*/', 'doc/', 'doc/*/'];

const isOpenDir = rel => OPEN_DIRS.some(p => path.fnmatch(p, rel + '/', path.FNM_PATHNAME) === 0);

/** Recursively scan dir into { dirs: Map<name, node>, files: [name...], count }.
 *  Skips dotfiles/dotdirs, EXCLUDE_PATTERNS, and symlinked directories (which
 *  could otherwise cycle back on themselves); count is the recursive total
 *  of files below this node. */
function scan(dir, depth = 0, budget = { n: 0 }) {
  const node = { dirs: new Map(), files: [], count: 0 };
  if(depth > MAX_DEPTH || budget.n >= MAX_FILES) return node;
  const [entries, err] = os.readdir(dir);
  if(err) return node;
  for(const name of entries.sort()) {
    if(budget.n >= MAX_FILES) break;
    if(name.startsWith('.') || excluded(name)) continue;
    const abs = path.join(dir, name);
    if(path.isSymlink(abs)) continue;
    if(path.isDirectory(abs)) {
      const child = scan(abs, depth + 1, budget);
      node.dirs.set(name, child);
      node.count += child.count;
    } else {
      node.files.push(name);
      node.count++;
      budget.n++;
    }
  }
  return node;
}

/** Renders one node's children as <li> items; 'base' is this node's path
 *  relative to the project root (for building file hrefs). */
function renderNode(node, base) {
  let html = '';
  for(const [name, child] of node.dirs) {
    const childBase = base ? base + '/' + name : name;
    const open = isOpenDir(childBase) ? ' open' : '';
    html += `<li><details${open}><summary>${esc(name)}/ <span class="count">(${child.count} files)</span></summary>` +
      `<ul class="tree">${renderNode(child, childBase)}</ul></details></li>`;
  }
  for(const name of node.files) {
    const rel = base ? base + '/' + name : name;
    html += `<li><a href="${rel.split('/').map(encodeURIComponent).join('/')}">${esc(name)}</a></li>`;
  }
  return html;
}

function fileTree(projectName, dir) {
  const root = scan(dir);
  const items = renderNode(root, '');
  return page(projectName, `<h1><a href="${up(1)}">projects</a> / ${esc(projectName)}</h1><ul class="tree">${items}</ul>`);
}

/** Breadcrumb for a file page at '/<project>/<rel>': links back to the site
 *  root and the project root, both relative to this page's own directory. */
function breadcrumb(projectName, rel) {
  const depth = rel.split('/').length;
  return `<a href="${up(depth)}">projects</a> / <a href="${up(depth - 1)}">${esc(projectName)}</a> / ${esc(rel)}`;
}

function sourcePage(projectName, rel, lang, src) {
  const body = `<h1>${breadcrumb(projectName, rel)}</h1>` +
    `<pre><code>${highlight(src, lang)}</code></pre>`;
  return page(rel, body);
}

function markdownPage(projectName, rel, src) {
  const { html } = renderMarkdown(src, { highlight });
  const body = `<h1>${breadcrumb(projectName, rel)}</h1><article>${html}</article>`;
  return page(rel, body);
}

/* -------------------------------------------------------------------- http */

function send(wsi, status, body) {
  wsi.respond(status, { 'content-type': 'text/html; charset=utf-8' });
  wsi.write(body, LWS_WRITE_HTTP_FINAL);
}

function sendFile(wsi, mime, buf) {
  wsi.respond(200, { 'content-type': mime }, buf.byteLength);
  wsi.write(buf, LWS_WRITE_HTTP_FINAL);
}

function decode(part) {
  try { return decodeURIComponent(part); } catch(e) { return part; }
}

function onHttp(wsi) {
  const segments = wsi.uri.split('/').filter(Boolean).map(decode);

  if(segments.length === 0) return send(wsi, 200, projectIndex());

  const projectName = segments[0];
  const dir = PROJECTS.get(projectName);
  if(!dir) return send(wsi, 404, page('not found', `<h1>404</h1><p>no such project: ${esc(projectName)}</p>`));

  const rel = segments.slice(1).join('/');
  if(!rel) return send(wsi, 200, fileTree(projectName, dir));

  const abs = path.resolve(dir, rel);
  if(abs !== dir && !abs.startsWith(dir + '/'))
    return send(wsi, 403, page('forbidden', '<h1>403</h1>'));

  if(!path.isFile(abs))
    return send(wsi, 404, page('not found', `<h1>404</h1><p>no such file: ${esc(rel)}</p>`));

  const ext = path.extname(abs).toLowerCase();

  if(ext === '.md') return send(wsi, 200, markdownPage(projectName, rel, std.loadFile(abs)));

  let lang = EXT_LANG[ext] || langByName(path.basename(abs));
  let mime;
  if(!lang) {
    mime = mimeFor(abs, ext);
    lang = MIME_LANG[mime];
  }
  if(lang) return send(wsi, 200, sourcePage(projectName, rel, lang, std.loadFile(abs)));

  const buf = readFile(abs);
  if(!buf) return send(wsi, 404, page('not found', `<h1>404</h1><p>no such file: ${esc(rel)}</p>`));
  return sendFile(wsi, mime || mimeFor(abs, ext), buf);
}

createServer({
  port: PORT,
  vhostName: 'localhost',
  mounts: [{ mountpoint: '/', protocol: 'http', originProtocol: LWSMPRO_CALLBACK }],
  protocols: [{ name: 'http', onHttp }],
});

console.log(`project-server: ${PROJECTS.size} projects, listening on http://localhost:${PORT}/`);
