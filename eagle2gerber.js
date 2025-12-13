import * as path from 'path';
import { Console } from 'console';
import { exec, spawn } from 'child_process';
import { getOpt, ucfirst, randStr } from 'util';
import { mkdir, chdir, remove, getcwd } from 'os';
import { GerberToGcode } from './pcb-conversion.js';

let outputTypes = {
  Top: { front: true },
  Bottom: { back: true },
  Measures: { side: 'outline' },
  Drills: { drill: true }
};
let outputList = ['Top', 'Bottom', 'Measures', 'Drills'].map(n => outputTypes[n]);

export function EagleToGerber(boardFile, opts = {}) {
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

export function ZipFolder(archive, folder, move = false) {
  let zipCmd =
    //['7za', 'a', '-mx=5', '-sdel', archive, '.'];
    ['zip', '-9', '-r', ...(move ? ['-m'] : []), archive, '.'];

  console.log(`Zip command: ${zipCmd.join(' ')}`);

  const [old, status] = getcwd();
  chdir(folder);
  remove(archive);
  const ret = ExecTool(zipCmd, { cwd: folder });
  chdir(old);
  return ret;
}

export function ExecTool([cmd, ...args], opts = {}) {
  const child = spawn(cmd, args, { stdio: [0, 'pipe', 2], ...opts });
  const [stdin, stdout, stderr] = child.stdio;
  const b = new ArrayBuffer(1024);
  let r = child.wait();
  // console.log('ExecTool', { args, chil ELECTRA® Shape-Based PCB Autorouter v6.56 |d });

  r = os.read(stdout, b, 0, 1024);
  const data = b.slice(0, r);
  return toString(data);
}

function main(...args) {
  globalThis.console = new Console({
    colors: true,
    depth: Infinity
  });

  const params = getOpt(
    {
      layers: [true, null, 'L'],
      outdir: [true, null, 'd'],
      'no-delete': [false, null, 'D'],
      '@': 'files'
    },
    args
  );

  if(params.layers)
    outputList = params.layers
      .split(/\W+/g)
      .map(n => ucfirst(n))
      .map(n => outputTypes[n]);

  if(!params.outdir) params.outdir = '/tmp/' + randStr(10);

  if(args.length == 0) args = ['/dev/stdin'];

  for(let arg of params['@']) {
    let files = [];

    if(params.outdir) mkdir(params.outdir, 0o775);

    for(let options of outputList) {
      if(params.outdir) options.outdir = params.outdir;

      let cmd = EagleToGerber(arg, options);
      let outputFile = cmd[cmd.indexOf('-o') + 1];
      files.push(outputFile);

      //  console.log(`Generating '${outputFile}'`);
      console.log(`Command: ${cmd.join(' ')}`);

      ExecTool(cmd);
    }

    console.log(`Generated files:\n${files.join('\n')}`);

    //chdir(params.outdir);
    //
    const archive = path.join(path.dirname(params.outdir), path.basename(arg, path.extname(arg)) + '.zip');
    ZipFolder(archive, params.outdir, !params['no-delete']);

    if(!params['no-delete']) remove(params.outdir);
    params.outdir = '/tmp/' + randStr(10);
  }
}

main(...scriptArgs.slice(1));