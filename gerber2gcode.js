import { spawn } from 'child_process';
import fs from 'fs';
import * as path from 'path';
import { getOpt } from 'util';
import { ReadFile, WriteFile } from './io-helpers.js';
import { PointList } from './lib/geom/pointList.js';
import GerberParser from './lib/gerber/parser.js';
import { parse as parsePath } from './lib/svg/path-parser.js';
import { Parser, Serializer } from './quickjs/qjs-modules/lib/dom.js';
import { Console } from 'console';
#!/usr/bin/env qjsm
let extToSide = { GTL: 'front', GBL: 'back', GKO: 'outline', TXT: 'drill' };
let extToOptions = { GTL: { front: true }, GBL: { back: true }, GKO: { side: 'outline' }, TXT: { drill: true } };

function ReadSVG(file) {
  let parser = new Parser();
  let doc = parser.parseFromFile(file);

  return doc;
}

function ReadGerber(file) {
  let p = file.endsWith('TXT') ? new GerberParser(5, undefined, 'drill') : new GerberParser(undefined, undefined, 'gerber');
  return p.parseSync(ReadFile('tmp/Mind-Synchronizing-Generator-PinHdrPot-Cinch.GBL', 'utf-8'));
}

function* Style2Entries(element) {
  let i,
    n = element.style.length;
  for(i = 0; i < n; i++) {
    let key = element.style.item(i);

    yield [key, element.style.getPropertyValue(key)];
  }
}

function Style2Obj(element) {
  return Object.fromEntries(Style2Entries(element));
}

function AccumulatePaths(paths) {
  let pathMap = {};

  for(let p of paths) {
    let o = path.parse(p);
    let { ext } = o;
    delete o.ext;
    delete o.base;

    let key = path.format(o);

    if(!(key in pathMap)) pathMap[key] = {};

    ext = ext.slice(1).toUpperCase();

    let side = extToSide[ext];

    if(side === undefined) throw new Error(`Unknown extension '${ext}' on argument '${p}'`);

    pathMap[key][side] = p;
  }
  return pathMap;
}

export function GerberToGcode(gerberFile, allOpts = {}) {
  const basename = (gerberFile ?? '').replace(/.*\//g, '').replace(/\.[^.]*$/, '');
  let { fetch, data, raw, outdir, ...opts } = allOpts;
  opts = {
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
    'output-dir': outdir ?? './tmp/',
    ...opts
  };
  if(opts['output-dir']) {
    if(!path.exists(opts['output-dir'])) os.mkdir(outdir, 0o775);
  }

  if(typeof basename == 'string' && basename.length > 0) opts.basename = basename;
  if(opts.front == undefined && opts.back == undefined && opts.drill == undefined) opts.back = gerberFile;
  let sides = [];

  for(let side of ['front', 'back', 'drill', 'outline'])
    if(side in opts) {
      if(typeof opts[side] != 'string') opts[side] = gerberFile;
      sides.push(side);
    }

  if(opts.voronoi && !opts.vectorial) opts.vectorial = 1;

  console.debug(`gerberToGcode`, opts);
  function makePath(ext, side, base = basename) {
    return path.join(opts['output-dir'], `${base}_${side}.${ext}`);
  }

  const params = [...Object.entries(opts)]
    .filter(([k, v]) => typeof v == 'string' || typeof v == 'number' || (typeof v == 'boolean' && v === true))
    .map(([k, v]) => `--${k}${typeof v != 'boolean' && v != '' ? '=' + v : ''}`);
  //console.log('Request /gcode', { gerberFile, fetch, raw });
  //console.warn(`gerberToGcode`, Util.abbreviate(gerberFile), { gcodeFile, opts });

  return ['pcb2gcode'].concat(params);
}

export function ExecTool(cmd, ...args) {
  let child = spawn(cmd, args, { stdio: [0, 'pipe', 2] });
  let [stdin, stdout, stderr] = child.stdio;
  let r;
  let b = new ArrayBuffer(1024);
  r = child.wait();
  // console.log('ExecTool', { args, chil ELECTRA® Shape-Based PCB Autorouter v6.56 |d });

  r = os.read(stdout, b, 0, 1024);
  let data = b.slice(0, r);
  let str = toString(data);
  console.log('str', str);
  return str;
  return parseInt(str);
}

function main(...args) {
  globalThis.console = new Console({
    colors: true,
    depth: Infinity,
    maxArrayLength: Infinity,
    compact: 1
  });

  let {
    outdir = './tmp/',
    vectorial,
    voronoi,
    ...params
  } = getOpt(
    {
      outdir: [true, null, 'd'],
      vectorial: [false, null, 'v'],
      voronoi: [false, null, 'V'],
      '@': 'files'
    },
    args
  );
  let pathsObj = AccumulatePaths(params['@']);

  for(let base in pathsObj) {
    let files = [],
      options = pathsObj[base];
    let gerbFile = options.drill ?? options.back ?? options.front;
    console.log(`options`, options);

    let gerber = ReadGerber(gerbFile);
    console.log(`gerber (${gerbFile})`, console.config({ compact: 2 }), gerber);
    let cmd = GerberToGcode(base, { ...options, vectorial, voronoi, outdir });
    let outputIndex = cmd.indexOf('-o') + 1;
    let outputFile = outputIndex > 0 ? cmd[outputIndex] : null;
    if(outputFile) files.push(outputFile);

    console.log(`Command: ${cmd.join(' ')}`);

    let result = ExecTool(...cmd);
    console.log(`Result:`, result);

    if(fs.existsSync(outdir + '/processed_back.svg')) {
      let svg = ReadSVG(outdir + '/processed_back.svg');
      let paths = svg.querySelectorAll('path, polyline');

      console.log('svg', { svg });

      for(let elem of paths) {
        let styleObj = Style2Obj(elem);
        //console.log('elem', elem, { tag: elem.tagName, styleObj });

        if(!('fill-opacity' in styleObj) || +styleObj['fill-opacity'] != 0.2) elem.style.display = 'none';

        switch (elem.tagName) {
          case 'polyline': {
            let data = elem.getAttribute('points');
            let pl = new PointList(data);
            //  console.log('pl', pl);
            break;
          }
          case 'path': {
            let data = elem.getAttribute('d');
            let svgP = parsePath(data);
            //  console.log('svgP', svgP);

            break;
          }
        }
      }
      let ser = new Serializer();

      WriteFile('out.svg', ser.serializeToString(svg));
    }
    //   console.log(`Generated files:\n${files.join('\n')}`);
  }
}

main(...scriptArgs.slice(1));