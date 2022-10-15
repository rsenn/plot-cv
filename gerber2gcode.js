#!/usr/bin/env qjsm
import { Console } from 'console';
import * as path from 'path';
import { exec, spawn } from 'child_process';
import { getOpt } from 'util';

let extToOptions = { GTL: { front: true }, GBL: { back: true }, GKO: { side: 'outline' }, TXT: { drill: true } };

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
    depth: Infinity
  });

  let { outdir, ...params } = getOpt(
    {
      outdir: [true, null, 'd'],
      '@': 'files'
    },
    args
  );
  let files = params['@'];

  if(args.length == 0) args = ['/dev/stdin'];

  for(let arg of files) {
    let files = [];
    let ext = path.extname(arg).slice(1);
    console.log(`Ext: ${ext}`);

    let options = extToOptions[ext];
    let cmd = GerberToGcode(arg, { ...options, outdir });
    let outputFile = cmd[cmd.indexOf('-o') + 1];
    files.push(outputFile);

    //  console.log(`Generating '${outputFile}'`);
    console.log(`Command: ${cmd.join(' ')}`);

    let result = ExecTool(...cmd);
    console.log(`Result:`, result);

    console.log(`Generated files:\n${files.join('\n')}`);
  }
}

main(...scriptArgs.slice(1));
