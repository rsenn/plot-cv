import Alea from './lib/alea.js';
import { SVG } from './lib/dom/svg.js';
import { Element } from './lib/dom/element.js';
import dom from './lib/dom-old.js';
import { FetchCached, NormalizeResponse, ResponseData } from './lib/fetch.js';
import { parseGcode } from './lib/gcode.js';
import { BBox, isPoint, Point, Polyline, default as geom } from './lib/geom.js';
import { GithubListContents, GithubListRepositories, GithubRepositories, ListGithubRepoServer } from './lib/github.js';
import KolorWheel from './lib/KolorWheel.js';
import { isObject, lazyProperty } from './lib/misc.js';
import { trkl } from './lib/trkl.js';

const prng = new Alea(1598127218);

export async function AsyncFetch(url, params = {}) {
  return await fetch('files', {
    method: 'post',
    mode: 'cors',
    cache: 'no-cache',
    credentials: 'same-origin',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(params)
  })
    .then(NormalizeResponse)
    .catch(error => ({ error }));
}

export async function ListProjects(opts = {}) {
  const { url, descriptions = true, names, filter = '.*.(brd|sch|lbr|GBL|GKO|GTL)$' } = opts;
  console.log('ListProjects', { url, descriptions, names, filter });
  let response;
  if(!url) {
    response = await AsyncFetch('files', { descriptions, names, filter })
      .then(NormalizeResponse)
      .catch(error => ({ error }));

    if(isObject(response)) response = response.data;

    if(typeof response == 'string') {
      console.log('ListProjects', { response }, response.length);

      /*let resp=response.substring(330100);
      console.log('ListProjects', { resp },resp.length);  */

      response = JSON.parse(response);
    }
  } else {
    response = await GithubListContents(url, null, null, '\\.(brd|sch|lbr)$', opts);

    //console.log('GithubListContents response:', response);
    if(Array.isArray(response)) {
      let fileList = response.map((file, i) => {
        let project = { ...file, name: response.at(i) };
        return project;
      });
      response = { files: fileList };
    }
  }

  console.log('ListProjects', { response });
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

  let dom = create ? create(project, { ...props, 'data-layer': `${i} ${name}` }) : SVG.create('g', { i, stroke: color, ...props }, project.svgElement);
  let visible = trkl(true);

  visible.subscribe(value => {
    //console.warn(`layer ${name} visible=${value}`);
    value ? dom.style.removeProperty('display') : dom.style.setProperty('display', 'none');
  });
  layer = { i, name, color, visible, dom };
  window.layers = [...layers, layer];
  return layer;
};

export async function BoardToGerber(proj, opts = { fetch: true }) {
  let b,
    gerber = proj.gerber || {};
  let params = { ...opts, board: proj.path ? proj.path() : proj.name, raw: false },
    response,
    result;
  response = await FetchURL(`gerber/${opts.side ? '?side=' + opts.side : ''}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(params)
  })
    .then(NormalizeResponse)
    .catch(error => ({ error }));
  //console.debug('FetchURL response =', response);
  if(!gerber[opts.side]) gerber[opts.side] = {};
  result = gerber[opts.side];

  result.file = response.data.file;
  result.output = response.data.output;
  result.data = response.data.data;

  if((opts.fetch || !gerber[opts.side].data) && response.file) {
    response = await FetchCached('static/' + r.file.replace(/^\.\//, '')).then(ResponseData);
    //console.debug('FetchURL response =', response);
    if(response) result.data = response;
  }

  //console.debug('BoardToGerber result =', filterOutKeys(result, ['headers', 'code']));
  return result;
}

export async function GerberToGcode(project, allOpts = {}) {
  const { side, ...opts } = allOpts;
  //console.debug('GerberToGcode', { side, allOpts });
  //  console.debug('project.gerber', project.gerber);
  const gerber = project.gerber[side];
  const file = allOpts.file || gerber.file;
  let request = {
    file,
    fetch: true,
    software: 'LinuxCNC',
    /*raw: true, */ 'isolation-width': '1mm',
    ...opts
  };
  let response,
    result = (project.gcode[side] = {});
  if(typeof side == 'string') request[side] = 1;
  response = await FetchURL(`gcode${side ? '?side=' + side : ''}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(request)
  })
    .then(NormalizeResponse)
    .catch(error => ({ error }));
  if(response.data) {
    let { data } = response;
    if(data[side]) {
      //      console.debug('GerberToGcode data =', data);
      result.data = data[side];
      result.file = data.files[side];
      result.cmd = data.cmd;
      result.output = data.output;
    }
  }
  if((opts.fetch || typeof result.data != 'string') && result.file) {
    response = await FetchCached('static/' + response.file.replace(/^\.\//, '')).then(ResponseData);
    //console.debug('GerberToGcode result =', result);
    if(response.data) result.data = response.data;
  }

  //  console.debug('GerberToGcode result =', result);
  if(!result.data)
    lazyProperty(result, 'data', async () => {
      let response = await FetchCached('static/' + response.file).then(ResponseData);
      return response;
    });
  return result;
}

export const GcodeToPolylines = (data, opts = {}) => {
  const { fill = false, color, side } = opts;
  //console.debug('GcodeToPolylines', { data, opts });

  let gc = [...filter(parseGcode(data), g => /G0[01]/.test(g.command + '') && 'x' in g.args && 'y' in g.args)];
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
  let grp = GetLayer(
    {
      fill: 'none',
      name: `Voronoi ${side}`,
      class: `gcode ${side} side`,
      color,
      'stroke-width': 0.15,
      transform: ` translate(-0.3175,0) ` + (side == 'front' ? 'scale(-1,-1)' : 'scale(1,-1)') + ` translate(${0},${-bb.y2})  translate(0,0)`
    },
    project
  ).dom;
  let paths = [];

  if(fill) {
    polylines = polylines.map(pl => pl.toMatrix().flat());
    //console.log('polylines(2):', polylines);
    polylines = polylines.map(pl => geom.simplify(pl, 0.02, true));
    //console.log('polylines(3):', polylines);
    polylines = polylines.map(pl => chunkArray(pl, 2).map(pt => new Point(...pt)));
    //console.log('polylines(4):', polylines);
    polylines = polylines.map(pl => new Polyline([]).push(...pl));
    let inside = new Map(polylines.map((polyline2, i) => [polyline2, polylines.filter((polyline, j) => polyline !== polyline2 && i !== j && Polyline.inside(polyline, polyline2))]));
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
    ...ids.map(i => polylines[i].toSVG((...args) => args, { ...props(polylines[i], i), id: `polyline-${i}` }, grp, 0.01)),
    ...paths
      .map(([i, d]) => ({
        ...props(polyline, i),
        id: `polygon-${polylines.indexOf(polyline)}`,
        d
      }))
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
  let base = new HSLA(randInt(0, 360, prng), 100, 50).toRGBA();
  let offsets = range(1, numColors).reduce((acc, i) => [...acc, ((acc[acc.length - 1] || 0) + randInt(20, 80)) % 360], []);
  offsets = offsets.sort((a, b) => a - b);
  //offsets = shuffle(offsets, prng);
  //log('offsets:', offsets);

  new KolorWheel(base.hex()).rel(offsets, 0, 0).each(function () {
    const hex = this.getHex();
    const rgba = new RGBA(hex);
    const hsla = rgba.toHSLA();
    //log(hex, rgba.toString(), hsla.toString());
    ret.push(hsla);
  });
  return ret;
}

export async function ClearCache(match = /.*/) {
  let pred = predicate(match);
  let cache = await caches.open('fetch');
  for(let request of await cache.keys()) {
    if(pred(request.url)) {
      console.warn(`Cleared cache entry ${request.url}`);
      cache.delete(request);
    }
  }
}

export async function ListCache(match = /.*/) {
  let pred = predicate(match);
  let cache = await caches.open('fetch');
  let baseUrl = makeURL({ location: '/' });
  let result = [];

  for await(let request of await cache.keys()) {
    if(pred(request.url)) {
      //yield request.url.replace(baseUrl, '');
      result.push(request.url.replace(baseUrl, ''));
    }
  }
  return result;
}

export async function ShowCache(match = /.*/) {
  for(let url of await ListCache(match)) console.info(`Cache entry ${url}`);
}

export async function GetCache(match = /.*/, key = 'fetch') {
  let pred = predicate(match);
  let cache = await caches.open(key);
  let baseUrl = makeURL({ location: '/' });
  let entries = [];

  for(let request of await cache.keys()) {
    let response = await cache.match(request);
    let time = Date.parse(response.headers.get('date')) / 1000;
    let headers = new Map(map(response.headers.keys(), k => [k, response.headers.get(k)]));
    delete response.headers;
    let methods = bindMethods(getMethods(response), response);

    response = Object.assign(getMembers(response), {
      headers,
      time,
      ...methods
    });
    const file = request.url.replace(baseUrl, '');

    if(pred(file)) entries.push([file, response]);
  }
  return entries;
}

export async function FetchURL(url, allOpts = {}) {
  let { nocache = false, ...opts } = allOpts;
  let result;
  let ret;
  if(opts.method && opts.method.toUpperCase() == 'POST') nocache = true;
  let { fetch } = globalThis;
  if(/tmp\//.test(url)) {
    url = url.replace(/.*tmp\//g, '/tmp/');
  } else if(/^\//.test(url)) {
  } else if(/:\/\//.test(url)) {
  } else if(!/[\?&=]/.test(url)) {
    url = '/static/' + url;
  }
  try {
    if(!ret) ret = result = await fetch(url, opts);
  } catch(error) {
    console.log('FetchURL ERROR:', error.message + '\n' + error.stack);
    throw error;
  }
  return ret;
}

export default {
  ListProjects,
  FindLayer,
  GetLayer,
  AddLayer,
  GeneratePalette,
  GithubListRepositories,
  GithubListContents,
  ListGithubRepoServer,
  ClearCache,
  ListCache,
  ShowCache,
  GetCache,
  GithubRepositories
};