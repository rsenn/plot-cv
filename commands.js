import dom from './lib/dom.js';
import geom from './lib/geom.js';
import { BBox, Rect, Point, Polyline, Line, PointList, isPoint } from './lib/geom.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import { parseGcode } from './lib/gcode.js';
import React, { Component } from './lib/dom/preactComponent.js';
import components from './components.js';
import Voronoi from './lib/geom/voronoi.js';
import { makeEagleNode } from './lib/eagle.js';
import { trkl } from './lib/trkl.js';
import Alea from './lib/alea.js';
import KolorWheel from './lib/KolorWheel.js';
import { SVG, Element } from './lib/dom.js';

const prng = new Alea(1598127218);

export async function NormalizeResponse(resp) {
  resp = await resp;

  if(Util.isObject(resp)) {
    let { cached, status, ok, redirected, url, headers } = resp;
    let disp = headers.get('Content-Disposition');
    let type = headers.get('Content-Type');
    if(ok) {
      if(!disp && /json/.test(type) && typeof resp.json == 'function') resp = { data: await resp.json() };
      else if(typeof resp.text == 'function') resp = { data: await resp.text() };
      if(disp && !resp.file) resp.file = disp.replace(/.*['"]([^"]+)['"].*/, '$1');
      if(disp && type) resp.type = type;
      if(resp.file) if (!/tmp\//.test(resp.file)) resp.file = 'tmp/' + resp.file;
    } else {
      console.info('resp:', resp);
      resp = { ...resp, error: resp.statusText };
    }
    if(cached) resp.cached = true;
    if(redirected) resp.redirected = true;

    let fn = Util.mapFunction(headers);
    //let { keys, get } = fn;
    resp.headers = Util.define(Object.fromEntries(fn.entries()), { keys: fn, get: fn });
  }
  return resp;
}
export async function ResponseData(resp) {
  resp = await NormalizeResponse(resp);
  if(resp.data) return resp.data;
}

export const FetchCached = Util.cachedFetch({
  debug: true,
  print({ cached, ok, status, redirected, statusText, type, url }, fn, ...args) {
    console.debug(`FetchCached(${args
        .map((a, i) =>
          typeof a == 'string'
            ? '"' + a + '"'
            : i == 1
            ? Util.toSource({ ...this.opts, ...a }, { colors: false, multiline: false })
            : a
        )
        .join(', ')}) =`,
      { cached, ok, status, redirected, statusText, type, url } /*.then(  NormalizeResponse)*/
    );
  }
});

export async function FetchURL(url, allOpts = {}) {
  let { nocache = false, ...opts } = allOpts;
  let result;
  let ret;
  if(opts.method && opts.method.toUpperCase() == 'POST') nocache = true;
  let fetch = nocache ? window.fetch : FetchCached;
  if(/tmp\//.test(url)) {
    url = url.replace(/.*tmp\//g, '/tmp/');
  } else if(/^\//.test(url)) {
  } else if(/:\/\//.test(url)) {
  } else {
    url = '/static/' + url;
  }
  try {
    if(!ret) ret = result = await fetch(url, opts);
  } catch(error) {
    Util.putError(error);
    throw error;
  }
  return ret;
}

export async function ListProjects(opts = {}) {
  const { url, descriptions = true, names, filter } = opts;
  console.log('ListProjects', { url, descriptions, names, filter });
  let response;
  if(!url) {
    response = await fetch('/files.html', {
      method: 'post',
      mode: 'cors',
      cache: 'no-cache',
      credentials: 'same-origin',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ descriptions, names, filter })
    })
      .then(NormalizeResponse)
      .catch(error => ({ error }));

    /*   if(typeof response.text == 'function') response = await response.text();
    //console.log('response:', Util.abbreviate(response));
    if(response) response = JSON.parse(response);*/
  } else {
    response = await GithubListContents(url, null, null, '\\.(brd|sch|lbr)$', opts);

    console.log('GithubListContents response:', response);
    if(Util.isArray(response)) {
      let fileList = response.map((file, i) => {
        let project = { ...file, name: response.at(i) };
        return project;
      });
      response = { files: fileList };
    }
  }
  return response;
}

export const FindLayer = (name, project = window.project) => {
  let layers = window.layers;
  return layers.find(l => l.name === name);
};

export const GetLayer = (layer, project = window.project) => FindLayer(layer.name, project) || AddLayer(layer, project);

export const AddLayer = (layer, project = window.project) => {
  const { color, name, create, ...props } = layer;
  let layers = window.layers;
  let i = Math.max(...layers.map(l => l.i)) + 1;

  let dom = create
    ? create(project, { ...props, 'data-layer': `${i} ${name}` })
    : SVG.create('g', { i, stroke: color, ...props }, project.svgElement);
  let visible = trkl(true);

  visible.subscribe(value => {
    console.warn(`layer ${name} visible=${value}`);
    value ? dom.style.removeProperty('display') : dom.style.setProperty('display', 'none');
  });
  layer = { i, name, color, visible, dom };
  window.layers = [...layers, layer];
  return layer;
};

export async function BoardToGerber(proj, opts = { fetch: true }) {
  let data;
  let request = { ...opts, board: proj.name, raw: false },
    response;
  response = await FetchURL(`/gerber/${opts.side ? '?side=' + opts.side : ''}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(request)
  })
    .then(NormalizeResponse)
    .catch(error => ({ error }));

  if(opts.fetch && response.file && !response.data)
    response.data = await FetchURL(`static/${response.file.replace(/^\.\//, '')}`).then(NormalizeResponse);

  console.debug('BoardToGerber response =', Util.filterOutKeys(response, /(data)/));
  return response;
}

export async function GerberToGcode(file, allOpts = {}) {
  const { side, ...opts } = allOpts;
  console.debug('GerberToGcode', file, allOpts);
  let request = {
    file,
    fetch: true,
    software: 'LinuxCNC',
    /*raw: true, */ 'isolation-width': '1mm',
    ...opts
  };
  let response;
  if(typeof side == 'string') request[side] = 1;
  response = await FetchURL(`/gcode${side ? '?side=' + side : ''}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(request)
  })
    .then(NormalizeResponse)
    .catch(error => ({ error }));

  if(opts.fetch && response.file && !response.data)
    response.data = await FetchURL(`static/${response.file.replace(/^\.\//, '')}`).then(NormalizeResponse);

  response.opts = opts;
  console.debug('GerberToGcode response =', Util.filterOutKeys(response, /(data)/));
  return response;
}

export const GcodeToPolylines = (data, opts = {}) => {
  const { fill = false, color, side } = opts;
  let gc = [...Util.filter(parseGcode(data), g => /G0[01]/.test(g.command + '') && 'x' in g.args && 'y' in g.args)];
  console.debug('GcodeToPolylines', Util.abbreviate(data), { opts, gc });
  let polylines = [];
  let polyline = null;
  let bb = new BBox();
  const NewPolyline = () => {
    if(polyline) polylines.push(polyline);
    polyline = new Polyline([]);
  };
  const f = new Point(-25.4, 25.4);
  for(let g of gc) {
    if(g.command == 'G00') NewPolyline();
    if(isPoint(g.args)) {
      let p = new Point(g.args.x, g.args.y).prod(f).round(0.01, 2);
      bb.updateXY(p.x, p.y);
      polyline.push(p);
    }
  }
  NewPolyline();
  let palette = GeneratePalette(polylines.length);
  let ret = { polylines, bbox: bb, palette };
  //console.log('polylines(1):', polylines);
  let remove = new Set();
  let props = color
    ? (polyline, i) => ({
        fill: fill ? palette[i].prod(1, 1, 1, 0.5) : 'none'
      })
    : (polyline, i) => ({
        stroke: color || palette[i],
        fill: fill ? palette[i].prod(1, 1, 1, 0.5) : 'none'
      });
  let grp = GetLayer({
      fill: 'none',
      name: `Voronoi ${side}`,
      class: `gcode ${side} side`,
      color,
      'stroke-width': 0.15,
      transform: ` translate(-0.3175,0) ` +
        (side == 'front' ? 'scale(-1,-1)' : 'scale(1,-1)') +
        ` translate(${0},${-bb.y2})  translate(0,0)`
    },
    project
  ).dom;
  let paths = [];

  if(fill) {
    polylines = polylines.map(pl => pl.toMatrix().flat());
    //console.log('polylines(2):', polylines);
    polylines = polylines.map(pl => geom.simplify(pl, 0.02, true));
    //console.log('polylines(3):', polylines);
    polylines = polylines.map(pl => Util.chunkArray(pl, 2).map(pt => new Point(...pt)));
    //console.log('polylines(4):', polylines);
    polylines = polylines.map(pl => new Polyline([]).push(...pl));
    let inside = new Map(polylines.map((polyline2, i) => [
        polyline2,
        polylines.filter((polyline, j) => polyline !== polyline2 && i !== j && Polyline.inside(polyline, polyline2))
      ])
    );
    let insideOf = polylines.map((polyline, i) => [
      i,
      polylines
        .map((polyline2, j) => [inside.get(polyline2).length, j, polyline2])
        .filter(([n, j, polyline2]) => i !== j && inside.get(polyline2).indexOf(polyline) != -1)
        .sort(([a], [b]) => a - b)
    ]);
    //console.log('GcodeToPolylines insideOf:', insideOf);
    let holes = polylines.map((polyline, i) => new Set());
    insideOf.filter(([i, list]) => list.length == 1).map(([i, list]) => holes[list[0][1]].add(i));
    //console.log('GcodeToPolylines holes:', holes);
    let remove = new Set();
    for(let [i, inner] of holes.entries()) {
      let ids = [i, ...inner];
      //console.log('polygon', { i, ids, inner });
      const polyline = polylines[i];
      inner = [...inner].map(ip => polylines[ip].counterClockwise);
      if(inner.length == 0) continue;
      //console.log('polygon', { polyline, inner });
      let list = [polyline, ...inner];
      paths.push([i, list.map(pl => pl.toPath()).join('\n')]);
      ids.forEach(id => remove.add(id));
    }
  }
  let ids = polylines.map((pl, i) => i).filter(i => !remove.has(i));
  let polys = [
    ...ids.map(i =>
      polylines[i].toSVG((...args) => args, { ...props(polylines[i], i), id: `polyline-${i}` }, grp, 0.01)
    ),
    ...paths
      .map(([i, d]) => ({ ...props(polyline, i), id: `polygon-${polylines.indexOf(polyline)}`, d }))
      .map((p, i) => ['path', p, grp])
  ];
  // console.log('GcodeToPolylines polys:', polys.length, { bb, color });
  let svgAttr = Element.attr(project.svgElement);
  //console.log('GcodeToPolylines svgAttr:', svgAttr);
  let elements = polys.map(args => SVG.create(...args));
  return { ...ret, group: grp, elements };
};

export function GeneratePalette(numColors) {
  let ret = [];
  let base = new HSLA(Util.randInt(0, 360, prng), 100, 50).toRGBA();
  let offsets = Util.range(1, numColors).reduce((acc, i) => [...acc, ((acc[acc.length - 1] || 0) + Util.randInt(20, 80)) % 360],
    []
  );
  offsets = offsets.sort((a, b) => a - b);
  //offsets = Util.shuffle(offsets, prng);
  //Util.log('offsets:', offsets);

  new KolorWheel(base.hex()).rel(offsets, 0, 0).each(function () {
    const hex = this.getHex();
    const rgba = new RGBA(hex);
    const hsla = rgba.toHSLA();
    //Util.log(hex, rgba.toString(), hsla.toString());
    ret.push(hsla);
  });
  return ret;
}

export async function* GithubListRepositories(user) {
  let url = `/github/${user}?tab=repositories`;
  let fetched = [];
  do {
    fetched.push(url);
    let response = await fetch(url, { mode: 'no-cors' });
    let text = await response.clone().text();
    let matches = [...text.matchAll(/([^<"]*after=[^">]*)/g)].map(m => m[1]);
    matches = matches.map(m => decodeURIComponent(Util.decodeHTMLEntities(m)));
    matches = matches.filter(m => m[0] != '{' && fetched.indexOf(m) == -1);
    //console.log('matches:', matches);
    url = matches[matches.length - 1] + '';
    //   url = decodeURIComponent(Util.decodeHTMLEntities(url));
    url = url.replace(/.*github\.com/, '/github');
    url = url.replace(new RegExp('(,|"|%22).*', 'g'), '');
    console.log('url:', url);
    yield* [...text.matchAll(/pository"\s*>\s*([^<]*)/g)].map(m => m[1]);
  } while(url != 'undefined');
}

export async function GithubRepositories(user) {
  let url = `/github/${user}?tab=repositories`;
  let ret = [];
  let fetched = [];
  let i = 0;
  do {
    // console.debug(`url #${i}:`, url);
    let html = await fetch(url).then(ResponseData);
    fetched.push(url);
    let indexes = [...html.matchAll(/<li[^>]*itemtype[^>]*>/g)].map(i => i.index);
    let hrefs = Util.unique([...html.matchAll(/href="([^"]*)"/g)].map(m => m[1]))
      .map(Util.decodeHTMLEntities)
      .map(decodeURIComponent)
      .filter(h => !/(return_to=|before=)/.test(h))
      .filter(h => /(tab=repositories|after=)/.test(h))
      .map(url => url.replace('https://github.com', ''))
      .map(url => '/github' + url)
      .filter(url => fetched.indexOf(url) == -1 && !/page=1/.test(url));
    url = hrefs[hrefs.length - 1];
    //console.debug('hrefs:', hrefs);
    let tags = indexes.map(index => [index, html.indexOf('</li>', index)]).map(([s, e]) => html.substring(s, e));
    let data = tags.map(Util.stripHTML);
    ret = ret.concat(data.map(a =>
        a.slice(0, 2).map(p => (['0', 'Forked from', 'Updated'].indexOf(p) != -1 || Util.isNumeric(p) ? '' : p))
      )
    );
    i++;
  } while(url);
  return new Map(ret.map(([name, description]) => [`https://github.com/${user}/${name}`, description]));
  return new Map(ret.map(([name, description]) => [name, { url: `https://github.com/${user}/${name}`, description }]));
  return new Map(ret.map(([name, ...rest]) => [name, [`https://github.com/${user}/${name}`, ...rest]]));
}

export const GithubListContents = async (owner, repo, dir, filter, opts = {}) => {
  const { username, password } = opts;
  let host, path;
  if(new RegExp('://').test(owner) || (repo == null && dir == null)) {
    const url = owner;
    let parts = url
      .replace(/.*:\/\//g, '')
      .replace('/tree/master', '')
      .split('/');
    while(!/github.com/.test(parts[0])) parts = parts.slice(1);
    [host, owner, repo, ...path] = parts;
    dir = path.join('/');
  }
  const url = `https://api.github.com/repos/${owner}/${repo}/contents/${dir}`;
  console.log('GithubListContents', { host, owner, repo, dir, filter, url });
  const headers = {
    Authorization: 'Basic ' + window.btoa(`${username}:${password}`)
  };
  let response = await FetchURL(url, { headers });
  let result = JSON.parse(await response.text());
  if(!Util.isArray(result)) {
    result.status = response.status;
    return result;
  }
  if(filter) {
    const re = new RegExp(filter, 'g');
    result = result.filter(({ name, type }) => type == 'dir' || re.test(name));
  }
  console.log('result:', result);
  const firstFile = result.find(r => !!r.download_url);
  const base_url = firstFile ? firstFile.download_url.replace(/\/[^\/]*$/, '') : '';
  const files = result.map(({ download_url = '', html_url, name, type, size, path, sha }) => ({
    url: (download_url || html_url || '').replace(base_url + '/', ''),
    name,
    type,
    size,
    path,
    sha
  }));
  const at = i => {
    let url = files[i].url;
    if(!/:\/\//.test(url)) url = base_url + '/' + url;
    return url;
  };
  return Object.assign(files.map((file, i) => {
      file.toString = () => at(i);
      if(file.type == 'dir') file.list = async (f = filter) => await GithubListContents(at(i), null, null, f, {});
      else {
        let getter = async function() {
          let data = await fetch(at(i), {});
          this.buf = await data.text();
          return this.buf;
        };
        let text = function() {
          return typeof this.buf == 'string' && this.buf.length > 0 ? this.buf : this.get();
        };
        file.get = getter;
        file.getText = text;
        Object.defineProperty(file, 'text', { get: text, enumerable: true, configurable: true });
      }
      return file;
    }),
    {
      base_url,
      at,
      async get(i) {
        const url = at(i);
        console.log('url:', url);
        return await FetchURL(url, {});
      },
      get files() {
        return files.filter(item => item.type != 'dir');
      },
      get dirs() {
        return files.filter(item => item.type == 'dir');
      }
    }
  );
};

export async function ListGithubRepoServer(owner, repo, dir, filter) {
  let response;
  let request = { owner, repo, dir, filter };
  try {
    response = await fetch('/github', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(request)
    });
  } catch(err) {}
  let ret = Util.tryCatch(() => JSON.parse(response),
    response => response,
    error => ({ error, response })
  );

  ret.at = function(i) {
    return this.base_url + '/' + this.files[i];
  };
  ret.get = async function(i) {
    let data = await FetchCached(this.at(i));
    return data;
  };
  return ret;
}

export async function ClearCache(match = /.*/) {
  let pred = Util.predicate(match);
  let cache = await caches.open('fetch');
  for(let request of await cache.keys()) {
    if(pred(request.url)) {
      console.warn(`Cleared cache entry ${request.url}`);
      cache.delete(request);
    }
  }
}

export async function ShowCache(match = /.*/) {
  let pred = Util.predicate(match);
  let cache = await caches.open('fetch');
  let baseUrl = Util.makeURL({ location: '/' });

  for(let request of await cache.keys()) {
    if(pred(request.url)) {
      const file = request.url.replace(baseUrl, '');
      console.info(`Cache entry ${file}`);
    }
  }
}

export async function GetCache(match = /.*/) {
  let pred = Util.predicate(match);
  let cache = await caches.open('fetch');
  let baseUrl = Util.makeURL({ location: '/' });
  let entries = [];

  for(let request of await cache.keys()) {
    let response = await cache.match(request);
    let time = Date.parse(response.headers.get('date')) / 1000;
    let headers = new Map(Util.map(response.headers.keys(), k => [k, response.headers.get(k)]));
    delete response.headers;
    let methods = Util.bindMethods(Util.getMethods(response), response);

    response = Object.assign(Util.getMembers(response), { headers, time, ...methods });
    entries.push([request.url.replace(baseUrl, ''), response]);
  }
  return entries;
}

export default {
  FetchURL,
  ListProjects,
  FindLayer,
  GetLayer,
  AddLayer,
  GeneratePalette,
  GithubListRepositories,
  GithubListContents,
  ListGithubRepoServer,
  ClearCache,
  ShowCache,
  GetCache,
  GithubRepositories
};
