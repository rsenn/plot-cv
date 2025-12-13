import * as path from 'path';
import { ExecTool } from './os-helpers.js';
import { Directory } from 'directory';

export { ExecTool } from './os-helpers.js';

export function GerberFiles(dir) {
  let d = new Directory(dir, Directory.NAME, Directory.TYPE_REG);
  let ret = {};
  for(let entry of d) {
    let ext = path.extname(entry).slice(1);
    console.log('GerberFiles', { entry, ext });
    let name = { GTL: 'front', GBL: 'back', TXT: 'drill', GTK: 'outline' }[ext];
    if(name) ret[name] = path.join(dir, entry);
  }
  return ret;
}

export function EagleToGerber(boardFile, opts = {}) {
  console.log('convertToGerber', { boardFile, opts });

  let {
    layers = opts.side == 'outline'
      ? ['Measures']
      : opts.drill
      ? ['Drills', 'Holes']
      : [opts.front ? 'Top' : 'Bottom', 'Pads', 'Vias'],
    format = opts.drill ? 'EXCELLON' : 'GERBER_RS274X',
    data,
    fetch = false,
    front,
    back,
    outdir = std.getenv('TMPDIR') ?? '/tmp'
  } = opts;

  const base = path.basename(boardFile, '.brd');

  const formatToExt = (layers, format) => {
    if(opts.drill || format.startsWith('EXCELLON') || layers.indexOf('Drills') != -1 || layers.indexOf('Holes') != -1)
      return 'TXT';
    if(layers.indexOf('Bottom') != -1 || format.startsWith('GERBER'))
      return opts.side == 'outline' ? 'GKO' : front ? 'GTL' : 'GBL';

    return 'rs274x';
  };

  const gerberFile = `${outdir}/${base}.${formatToExt(layers, format)}`;
  const cmd = `eagle -X -d ${format} -o "${gerberFile}" "${boardFile}" ${layers.join(' ')}`;
  const args = ['-X', '-d', format, '-o', gerberFile, boardFile, ...layers];
  const bin = '/opt/eagle-7.2.0/bin/eagle';

  return [bin].concat(args);
}

export function GerberToGcode(gerberFile, allOpts = {}) {
  const basename = gerberFile.replace(/.*\//g, '').replace(/\.[^.]*$/, '');
  let { fetch, data, raw, ...opts } = allOpts;
  opts = {
    basename,
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
    ...opts
  };

  if(opts.front == undefined && opts.back == undefined && opts.drill == undefined) opts.back = gerberFile;
  let sides = [];

  for(let side of ['front', 'back', 'drill', 'outline'])
    if(side in opts) {
      if(typeof opts[side] != 'string') opts[side] = gerberFile;
      sides.push(side);
    }

  if(opts.voronoi && !opts.vectorial) opts.vectorial = 1;

  //  console.debug(`gerberToGcode`, opts);
  function makePath(ext, side, base = basename) {
    return path.join(opts['output-dir'], `${base}_${side}.${ext}`);
  }

  const params = [...Object.entries(opts)]
    .filter(([k, v]) => typeof v == 'string' || typeof v == 'number' || (typeof v == 'boolean' && v === true))
    .map(([k, v]) => `--${k}${typeof v != 'boolean' && v != '' ? '=' + v : ''}`);

  return ['pcb2gcode'].concat(params);
}