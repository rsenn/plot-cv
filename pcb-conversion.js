import { abbreviate } from './lib/misc.js';
import * as path from 'path';
import * as util from 'util';
import { exec, spawn } from 'child_process';
import * as fs from 'fs';
import child_process from 'child_process';
import { ExecTool } from './os-helpers.js';
export { ExecTool } from './os-helpers.js';

export function EagleToGerber(boardFile, opts = {}) {
  console.log('convertToGerber', { boardFile, opts });
  let {
    layers = opts.side == 'outline' ? ['Measures'] : opts.drill ? ['Drills', 'Holes'] : [opts.front ? 'Top' : 'Bottom', 'Pads', 'Vias'],
    format = opts.drill ? 'EXCELLON' : 'GERBER_RS274X',
    data,
    fetch = false,
    front,
    back,
    outdir = std.getenv('TMPDIR') ?? '/tmp'
  } = opts;
  const base = path.basename(boardFile, '.brd');
  const formatToExt = (layers, format) => {
    if(opts.drill || format.startsWith('EXCELLON') || layers.indexOf('Drills') != -1 || layers.indexOf('Holes') != -1) return 'TXT';
    if(layers.indexOf('Bottom') != -1 || format.startsWith('GERBER')) return opts.side == 'outline' ? 'GKO' : front ? 'GTL' : 'GBL';

    return 'rs274x';
  };
  const gerberFile = `${outdir}/${base}.${formatToExt(layers, format)}`;
  const cmd = `eagle -X -d ${format} -o "${gerberFile}" "${boardFile}" ${layers.join(' ')}`;
  const args = ['-X', '-d', format, '-o', gerberFile, boardFile, ...layers];
  const bin = '/opt/eagle-7.2.0/bin/eagle';

  return [bin].concat(args);

  /*  console.log(`executing '${cmd}'`);
  //  const child = exec(`${cmd} 2>&1 0</dev/null`, {});
  const child = spawn(bin, args, {
    //  stdio: [ 'inherit','inherit',1]
  });
  console.log(`child:`, child);
  // do whatever you want with `child` here - it's a ChildProcess instance just
  // with promise-friendly `.then()` & `.catch()` functions added to it!
  let output = '';

  let code = child.wait();

  console.log(`code: ${code}`);
  //  console.log(`output: ${output}`);
  if(code !== 0) throw new Error(output);
  if(output) output = output.replace(/\s*\r*\n/g, '\n');
  let result = { code, output };
  if(opts.fetch) result.data = fs.readFileSync(GetVFSPath(gerberFile)); // await (await fs.readFile(GetVFSPath(gerberFile))).toString();
  result.file = gerberFile;
  console.log('convertToGerber result =', result);
  return result;*/
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

    'output-dir': './tmp/',
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
  /*
  let wait;
  try {
    const cmd = `pcb2gcode ${params.join(' ')} 2>&1`;
    console.warn(`executing '${cmd}'`);
    const child = exec(cmd, {});
    // do whatever you want with `child` here - it's a ChildProcess instance just
    // with promise-friendly `.then()` & `.catch()` functions added to it!
    let output = '';
    child.stdout.on('data', data => (output += data));
    child.stderr.on('data', data => (output += data));
    wait = await child.catch(error => ({ code: -1, error }));

    const { stdout, stderr, code, signal } = wait;
    if(output) output = Util.abbreviate(output.replace(/\s*\r*\n/g, '\n'), 200);
    console.log('Response /gcode', { stdout, output, sides });

    //   if(code !== 0) throw new Error(output);

    const gcodeFile = makePath('ngc', sides[0]);
    const svgFile = makePath('svg', sides[0], 'processed');

    for(let [file, to] of sides.map(side => [makePath('svg', side, 'processed'), makePath('svg', side)]))
      if(fs.existsSync(file)) fs.renameSync(file, to);

    let files = sides.map(side => [side, makePath('ngc', side)]).filter(([side, file]) => fs.existsSync(file));
    console.log('Response /gcode', { files });

    let result = { code, output, cmd };
    if(fetch) {
      for(let [side, file] of files) result[side] = await (await fsPromises.readFile(GetVFSPath(file))).toString();
    }
    if(raw) {
      const { file } = result;
      return SendRaw(res, file, result.data);
    }
    result.files = Object.fromEntries(files);
    console.log(
      'Response /gcode',
      util.filterKeys(result, key => !/(Xoutput|data)/.test(key))
    );
    return result;
  } catch(error) {
    console.log(`ERROR: ${error.message}`);
  }*/
}
