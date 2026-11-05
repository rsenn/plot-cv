/*
 * directory.js - index of the projects and demos published on
 * transistorisiert.ch.  One module, two faces:
 *
 *   browser   renders index.json as a searchable grid of preview cards
 *             (loaded by directory.html)
 *
 *   qjsm/bun  collects the data:
 *       qjsm directory.js collect [--nginx FILE] [--out FILE] [--no-probe]
 *           index.json from the root *.html pages and from every active
 *           (not commented out) location {} of the nginx config that
 *           serves something: a proxy_pass, alias or root.  A location is
 *           reachable when its target answers (proxy), exists (alias,
 *           root) or has an index file; unreachable ones stay in the file
 *           with reachable:false and are hidden by the browser.
 *       qjsm directory.js previews [--site DIR] [--force]
 *           previews/<id>.jpg by screenshotting the pages (from DIR,
 *           default build/site) and the reachable html locations with
 *           headless google-chrome.  Needs ImageMagick `convert`.
 *
 *   Hand-written names, descriptions and flags live in directory.json:
 *       { "<page file or location path>": { name, description, hidden } }
 */

const isBrowser = typeof document != 'undefined';

/* ------------------------------------------------------------------ */
/* browser                                                            */
/* ------------------------------------------------------------------ */

const KINDS = { demo: 'Demos', site: 'Sites & apps', service: 'Services' };

function El(tag, attrs = {}, ...children) {
  const e = document.createElement(tag);

  for(const [k, v] of Object.entries(attrs)) if(v !== undefined && v !== null) e.setAttribute(k, v);

  e.append(...children.flat().filter(c => c !== undefined && c !== null));
  return e;
}

function Hue(s) {
  let h = 0;

  for(const c of s) h = (h * 31 + c.charCodeAt(0)) % 360;
  return h;
}

function Card(entry) {
  const hue = Hue(entry.id);
  const thumb = El('div', { class: 'thumb', style: `--h:${hue}` }, El('span', {}, (entry.name || entry.id).replace(/^\W+/, '').slice(0, 1).toUpperCase()));

  if(entry.preview) {
    const img = El('img', { src: entry.preview, alt: '', loading: 'lazy', decoding: 'async' });

    img.addEventListener('error', () => img.remove());
    thumb.append(img);
  }

  return El(
    'a',
    { class: 'card ' + entry.kind, href: entry.href, 'data-q': `${entry.name} ${entry.description ?? ''} ${entry.id}`.toLowerCase() },
    thumb,
    El('div', { class: 'info' }, El('h2', {}, entry.name), El('p', {}, entry.description ?? ''), El('code', {}, entry.id)),
  );
}

async function Render() {
  const root = document.getElementById('entries');
  const data = await fetch('index.json').then(r => r.json());
  const showAll = new URLSearchParams(location.search).has('all');
  const entries = data.entries.filter(e => !e.hidden && (e.reachable !== false || showAll));
  const sections = {};
  let kind = 'all';

  for(const k of Object.keys(KINDS)) {
    const list = El('div', { class: 'grid' }, entries.filter(e => e.kind == k).sort((a, b) => !b.preview - !a.preview).map(Card));

    if(!list.children.length) continue;

    sections[k] = El('section', { 'data-kind': k }, El('h3', {}, KINDS[k], El('small', {}, ` ${list.children.length}`)), list);
    root.append(sections[k]);
  }

  const apply = () => {
    const q = document.getElementById('q').value.trim().toLowerCase();

    for(const [k, s] of Object.entries(sections)) {
      let n = 0;

      for(const c of s.querySelectorAll('.card')) {
        const on = (!q || c.dataset.q.includes(q)) && (kind == 'all' || kind == k);

        c.hidden = !on;
        n += on;
      }

      s.hidden = !n;
    }
  };

  const chips = document.getElementById('chips');

  for(const [k, label] of [['all', 'All'], ...Object.entries(KINDS).filter(([k]) => sections[k])]) {
    const b = El('button', { type: 'button', class: k == kind ? 'on' : '' }, label);

    b.addEventListener('click', () => {
      kind = k;
      chips.querySelectorAll('button').forEach(x => x.classList.toggle('on', x === b));
      apply();
    });
    chips.append(b);
  }

  document.getElementById('q').addEventListener('input', apply);
  document.getElementById('stamp').textContent = `${entries.length} entries · ${data.generated.slice(0, 10)}`;
}

if(isBrowser) document.addEventListener('DOMContentLoaded', () => Render().catch(e => (document.getElementById('entries').textContent = 'index.json: ' + e.message)));

/* ------------------------------------------------------------------ */
/* collector (qjsm / bun / node)                                      */
/* ------------------------------------------------------------------ */

const SKIP_PAGES = /^(404|directory|index)\.html?$/i;
const NON_SITE = /^\/(ws|uploads|moreload|debug|static|\.well-known)$/;

const Decode = s => s.replace(/&quot;/g, '"').replace(/&amp;/g, '&').replace(/&lt;/g, '<').replace(/&gt;/g, '>').replace(/&#0*39;/g, "'").replace(/\s+/g, ' ').trim();
const Strip = s => Decode(s.replace(/<[^>]*>/g, ' '));

function Meta(html) {
  const title = /<title[^>]*>([^<]*)<\/title>/i.exec(html)?.[1];
  const desc =
    /<meta\b[^>]*\bname\s*=\s*["']description["'][^>]*\bcontent\s*=\s*["']([^"']*)/i.exec(html)?.[1] ??
    /<meta\b[^>]*\bcontent\s*=\s*["']([^"']*)["'][^>]*\bname\s*=\s*["']description["']/i.exec(html)?.[1];
  const h1 = /<h1[^>]*>([\s\S]*?)<\/h1>/i.exec(html)?.[1];
  const head = h1 ? Strip(h1) : '';

  /* a heading that only repeats the title is no description; neither are leftovers of script-only pages */
  return { title: title ? Decode(title) : '', description: desc ? Decode(desc) : head && head.toLowerCase() != Decode(title ?? '').toLowerCase() && head.length > 3 ? head.slice(0, 160) : '' };
}

/* nginx.conf -> [{ head, args, children }], comments dropped */
export function ParseNginx(text) {
  text = text
    .split('\n')
    .map(l => l.replace(/(^|\s)#.*$/, ''))
    .join('\n');

  function Block(i) {
    const out = [];
    let cur = '';

    for(; i < text.length; i++) {
      const c = text[i];

      if(c == ';' || c == '{') {
        const words = cur.trim().match(/"[^"]*"|'[^']*'|\S+/g) ?? [];
        const node = { name: words[0], args: words.slice(1).map(w => w.replace(/^["']|["']$/g, '')) };

        if(c == '{') [node.children, i] = Block(i + 1);

        if(node.name) out.push(node);
        cur = '';
      } else if(c == '}') return [out, i];
      else cur += c;
    }

    return [out, i];
  }

  return Block(0)[0];
}

/* active, servable locations of every server block, grouped by path */
export function NginxLocations(conf) {
  const groups = new Map();

  for(const server of conf.filter(n => n.name == 'server')) {
    const get = n => server.children.find(c => c.name == n)?.args;
    const serverRoot = get('root')?.[0];

    for(const loc of server.children.filter(c => c.name == 'location')) {
      const [mod, p] = loc.args.length > 1 ? loc.args : ['', loc.args[0]];

      if(/^~/.test(mod) || !p || /\.[a-z0-9]+$/i.test(p)) continue;

      const d = n => loc.children.find(c => c.name == n)?.args;
      const key = p.replace(/\/+$/, '') || '/';
      const g = groups.get(key) ?? { path: key, serverRoot, redirects: [] };

      if(d('proxy_pass')) Object.assign(g, { type: 'proxy', target: d('proxy_pass')[0] });
      else if(d('alias')) Object.assign(g, { type: 'alias', target: d('alias')[0], index: d('index') ?? ['index.html'] });
      else if(d('root')) Object.assign(g, { type: 'root', target: d('root')[0], index: d('index') ?? ['index.html'] });
      else if(d('autoindex')?.[0] == 'on') Object.assign(g, { type: 'autoindex', target: serverRoot });
      else if(d('return')) g.redirects.push(d('return').join(' '));

      groups.set(key, g);
    }
  }

  return [...groups.values()].filter(g => g.type);
}

async function Collect(args) {
  const fs = await import('fs');
  const path = await import('path');
  const { execSync } = await import('child_process');
  const process = (await import('process')).default;
  const opt = (name, def) => (args.includes(name) ? args[args.indexOf(name) + 1] : def);
  const here = import.meta.dirname ?? path.dirname(import.meta.url.replace(/^file:\/\//, ''));
  const nginxFile = opt('--nginx', '/etc/nginx/sites-available/ssl');
  const outFile = opt('--out', path.join(here, 'index.json'));
  const probe = !args.includes('--no-probe');
  const tmp = path.join(here, 'build', `.probe-${process.pid}`);
  const overrides = fs.existsSync(path.join(here, 'directory.json')) ? JSON.parse(fs.readFileSync(path.join(here, 'directory.json'), 'utf8')) : {};
  const entries = [];

  fs.mkdirSync(path.dirname(tmp), { recursive: true });

  /* run a shell command, return its stdout; output goes through a file because qjsm's execSync does not capture */
  const sh = cmd => {
    try {
      execSync(`${cmd} > "${tmp}" 2>/dev/null`);
    } catch(e) {}

    return fs.existsSync(tmp) ? fs.readFileSync(tmp, 'utf8') : '';
  };

  for(const file of fs.readdirSync(here).filter(f => /\.html?$/i.test(f) && !SKIP_PAGES.test(f)).sort()) {
    const { title, description } = Meta(fs.readFileSync(path.join(here, file), 'utf8'));

    entries.push({ id: file, kind: 'demo', name: title || file.replace(/\.html?$/, ''), description, href: file, reachable: true });
  }

  const index = [];
  const fetchHtml = url => {
    const info = sh(`curl -sLk -m 4 -H 'Accept: text/html' -o "${tmp}.body" -w '%{http_code} %{content_type}' '${url}'`).trim();
    const [code, type = ''] = info.split(' ');
    const body = fs.existsSync(tmp + '.body') ? fs.readFileSync(tmp + '.body', 'utf8') : '';

    fs.rmSync(tmp + '.body', { force: true });
    return { code: +code, html: /html/i.test(type), body };
  };

  for(const loc of NginxLocations(ParseNginx(fs.readFileSync(nginxFile, 'utf8')))) {
    const entry = { id: loc.path, kind: 'site', name: loc.path.replace(/^\//, ''), description: '', href: loc.path + '/', reachable: false, nginx: loc.type };
    let meta = { title: '', description: '' };

    if(loc.type == 'proxy') {
      const url = loc.target.replace(/\/?$/, '/');

      entry.upstream = loc.target;

      if(probe) {
        const r = fetchHtml(url);

        entry.reachable = r.code > 0 && r.code < 500;
        if(r.html) meta = Meta(r.body);
        else if(entry.reachable) entry.kind = 'service';
      }
    } else {
      const dir = loc.type == 'alias' ? loc.target : path.join(loc.target ?? '/nonexistent', loc.path);
      const idx = (loc.index ?? ['index.html']).map(i => path.join(dir, i)).find(f => fs.existsSync(f));

      entry.dir = dir;
      entry.reachable = fs.existsSync(dir) && (loc.type == 'autoindex' || !!idx);

      if(idx) meta = Meta(fs.readFileSync(idx, 'utf8'));
      else if(loc.type == 'autoindex' && entry.reachable) meta.description = 'File listing';
    }

    if(NON_SITE.test(loc.path)) entry.kind = 'service';

    entry.name = meta.title || entry.name;
    entry.description = meta.description;
    index.push(entry);
  }

  entries.push(...index);

  for(const e of entries) Object.assign(e, overrides[e.id] ?? {});

  /* preview files that exist are linked; `previews` creates them */
  for(const e of entries) {
    const rel = `previews/${e.id.replace(/^\//, '').replace(/[^\w.-]+/g, '_')}.jpg`;

    if(fs.existsSync(path.join(here, rel))) e.preview = rel;
  }

  fs.rmSync(tmp, { force: true });
  fs.writeFileSync(outFile, JSON.stringify({ generated: new Date().toISOString(), host: 'transistorisiert.ch', entries }, null, 2) + '\n');
  process.stderr.write(`${entries.length} entries (${entries.filter(e => e.reachable).length} reachable) -> ${outFile}\n`);
}

async function Previews(args) {
  const fs = await import('fs');
  const path = await import('path');
  const { execSync } = await import('child_process');
  const process = (await import('process')).default;
  const opt = (name, def) => (args.includes(name) ? args[args.indexOf(name) + 1] : def);
  const here = import.meta.dirname ?? path.dirname(import.meta.url.replace(/^file:\/\//, ''));
  const siteDir = path.resolve(opt('--site', path.join(here, 'build/site')));
  const force = args.includes('--force');
  const port = 18000 + (process.pid % 1000);
  const data = JSON.parse(fs.readFileSync(path.join(here, 'index.json'), 'utf8'));
  const chrome = process.env.CHROME ?? 'google-chrome';
  const flags = `--headless=new --disable-gpu --hide-scrollbars --ignore-certificate-errors --window-size=1280,800 --virtual-time-budget=12000 --host-resolver-rules='MAP transistorisiert.ch 127.0.0.1,MAP www.transistorisiert.ch 127.0.0.1'`;
  const pidFile = path.join(here, 'build', `.http-${process.pid}`);
  let made = 0;

  fs.mkdirSync(path.join(here, 'previews'), { recursive: true });
  fs.mkdirSync(path.dirname(pidFile), { recursive: true });

  /* a separate process serves the staged site: execSync below blocks this one's event loop */
  execSync(`sh -c 'cd "${siteDir}" && nohup python3 -m http.server ${port} --bind 127.0.0.1 >/dev/null 2>&1 & echo $! > "${pidFile}"'`);
  execSync('sleep 1');

  try {
    for(const e of data.entries) {
      if(e.hidden || !e.reachable || e.kind == 'service') continue;

      const rel = `previews/${e.id.replace(/^\//, '').replace(/[^\w.-]+/g, '_')}.jpg`;
      const out = path.join(here, rel);
      const url = e.kind == 'demo' ? `http://127.0.0.1:${port}/${e.href}` : `https://transistorisiert.ch${e.href}`;

      if(!force && fs.existsSync(out)) continue;
      if(e.kind == 'demo' && !fs.existsSync(path.join(siteDir, e.href))) continue;

      try {
        execSync(`${chrome} ${flags} --screenshot="${out}.png" '${url}' >/dev/null 2>&1`, { timeout: 40000 });
        execSync(`convert "${out}.png" -resize 640x400^ -gravity north -extent 640x400 -quality 80 "${out}"`);

        /* a (near) uniform capture says nothing: drop it, the card shows its placeholder */
        execSync(`identify -format '%[fx:standard_deviation]' "${out}" > "${pidFile}.dev"`);

        const dev = +fs.readFileSync(`${pidFile}.dev`, 'utf8');

        if(dev < 0.02) fs.rmSync(out, { force: true });
        else made++;
      } catch(err) {
        process.stderr.write(`no preview for ${e.id}: ${err.message.split('\n')[0]}\n`);
      }

      fs.rmSync(out + '.png', { force: true });
    }
  } finally {
    execSync(`kill $(cat "${pidFile}") 2>/dev/null; rm -f "${pidFile}" "${pidFile}.dev"; true`);
  }

  process.stderr.write(`${made} previews written\n`);
}

if(!isBrowser && typeof process != 'undefined') {
  const [cmd, ...args] = process.argv.slice(2);
  const self = process.argv[1] ?? '';

  if(/(^|\/)directory\.js$/.test(self)) {
    if(cmd == 'collect') await Collect(args);
    else if(cmd == 'previews') await Previews(args);
    else {
      process.stderr.write('usage: directory.js collect [--nginx FILE] [--out FILE] [--no-probe] | previews [--site DIR] [--force]\n');
      process.exit(2);
    }
  }
}
