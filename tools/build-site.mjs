#!/usr/bin/env qjsm
/*
 * Build the static transistorisiert.ch site.
 *
 * Usage: qjsm tools/build-site.mjs [OUTDIR]   (default: build/site)
 *
 * Stages every demo page listed in index.json (made by
 * `qjsm directory.js collect`) with everything it references (scripts, ES
 * module imports, stylesheets, icons, url() assets), plus the index itself:
 * landing.html is published as index.html (front page), directory.html as
 * directory.html next to directory.js, index.json and previews/.  Missing references are reported on stderr
 * and skipped.
 */
import * as fs from 'fs';
import * as path from 'path';
import process from 'process';

const root = path.resolve(import.meta.dirname ?? path.dirname(process.argv[1]), '..');
const outDir = path.resolve(process.argv[2] ?? path.join(root, 'build/site'));

/* bare specifiers the browser cannot resolve on its own */
const importMap = {
  imports: {
    rbush: './static/vendor/rbush.js',
    quickselect: './static/vendor/quickselect.js',
    earcut: './static/vendor/earcut.js',
  },
};

const seen = new Set();
const missing = new Set();
const queue = [];

function Enqueue(file) {
  file = path.normalize(file);
  if(!file.startsWith(root + '/') || seen.has(file)) return;
  seen.add(file);

  if(fs.existsSync(file) && fs.statSync(file).isFile()) queue.push(file);
  else missing.add(path.relative(root, file));
}

function Resolve(from, ref) {
  ref = ref.replace(/[?#].*/, '');
  if(!ref || /^([a-z][a-z0-9+.-]*:|\/\/|data:)/i.test(ref)) return null;

  if(ref.startsWith('\0')) {
    const mapped = importMap.imports[ref.slice(1)];

    return mapped ? path.join(root, mapped) : null;
  }

  return ref.startsWith('/') ? path.join(root, ref) : path.resolve(path.dirname(from), decodeURIComponent(ref));
}

/* try the usual module-resolution suffixes for an extensionless import */
function WithSuffix(file) {
  if(fs.existsSync(file) && fs.statSync(file).isFile()) return file;

  for(const suffix of ['.js', '/index.js']) if(fs.existsSync(file + suffix)) return file + suffix;

  return file;
}

/* a module specifier without ./ ../ or / is a bare one: only the import map can resolve it */
const Bare = spec => (/^(\.\.?\/|\/|[a-z][a-z0-9+.-]*:)/i.test(spec) ? spec : '\0' + spec);

function Refs(file) {
  const ext = path.extname(file);
  const refs = [];
  let text;

  if(!/^\.(html?|m?js|css|svg|json|webmanifest)$/.test(ext)) return refs;

  text = fs.readFileSync(file, 'utf8');

  if(/^\.html?$/.test(ext)) {
    for(const m of text.matchAll(/<(?:script|link|img|source|image|use)\b[^>]*?\b(?:src|href)\s*=\s*["']([^"']+)["']/gi)) refs.push(m[1]);
    for(const m of text.matchAll(/\bimport\s*(?:[^'"()]*?from\s*)?["']([^"']+)["']/g)) refs.push(Bare(m[1]));
  }

  if(/^\.m?js$/.test(ext)) {
    for(const m of text.matchAll(/(?:^|[;\s}])(?:import|export)\s*(?:[^'"()=;]*?\bfrom\s*)?["']([^"']+)["']/gm)) refs.push(Bare(m[1]));
    for(const m of text.matchAll(/\bimport\(\s*["']([^"']+)["']\s*\)/g)) refs.push(Bare(m[1]));
  }

  if(ext == '.css') {
    for(const m of text.matchAll(/@import\s+(?:url\()?["']?([^"')\s]+)/g)) refs.push(m[1]);
    for(const m of text.matchAll(/url\(\s*["']?([^"')]+)/g)) refs.push(m[1]);
  }

  return refs;
}

const data = JSON.parse(fs.readFileSync(path.join(root, 'index.json'), 'utf8'));
const pages = data.entries.filter(e => e.kind == 'demo' && !e.hidden).map(e => [e.href]);

for(const file of ['directory.js', 'index.json']) Enqueue(path.join(root, file));
if(fs.existsSync(path.join(root, 'previews'))) for(const f of fs.readdirSync(path.join(root, 'previews'))) Enqueue(path.join(root, 'previews', f));

for(const [f] of pages) Enqueue(path.join(root, f));

for(let file; (file = queue.shift()); ) {
  const ext = path.extname(file);

  for(const ref of Refs(file)) {
    const target = Resolve(file, ref);

    if(target) Enqueue(WithSuffix(target));
  }
}

fs.rmSync(outDir, { recursive: true, force: true });

/* the import map must precede every module script: put it right after <head> */
function WithImportMap(html) {
  return /<head[^>]*>/i.test(html) ? html.replace(/<head[^>]*>/i, m => m + '\n    ' + mapTag) : mapTag + html;
}

const mapTag = `<script type="importmap">${JSON.stringify(importMap)}</script>\n    `;

for(const file of seen) {
  const rel = path.relative(root, file);

  if(missing.has(rel)) continue;

  fs.mkdirSync(path.dirname(path.join(outDir, rel)), { recursive: true });

  if(/^[^/]+\.html?$/.test(rel)) fs.writeFileSync(path.join(outDir, rel), WithImportMap(fs.readFileSync(file, 'utf8')));
  else fs.copyFileSync(file, path.join(outDir, rel));
}

/* the landing page is the front page; the project index lives at /directory.html */
fs.copyFileSync(path.join(root, 'landing.html'), path.join(outDir, 'index.html'));
fs.copyFileSync(path.join(root, 'directory.html'), path.join(outDir, 'directory.html'));

process.stderr.write(`${pages.length} pages, ${seen.size - missing.size} files -> ${outDir}\n`);
if(missing.size) process.stderr.write(`missing (${missing.size}):\n  ${[...missing].sort().join('\n  ')}\n`);
