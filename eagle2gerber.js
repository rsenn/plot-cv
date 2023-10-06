import { spawn } from 'child_process';
import * as path from 'path';
import { exec, spawn } from 'child_process';
import { getOpt, ucfirst } from 'util';
import { mkdir, chdir } from 'os';

let outputTypes = { Top: { front: true }, Bottom: { back: true }, Measures: { side: 'outline' }, Drills: { drill: true } };
let outputList = ['Top', 'Bottom', 'Measures', 'Drills'].map(n => outputTypes[n]);

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
}

export function ExecTool(cmd, ...args) {
  const child = spawn(cmd, args, { stdio: [0, 'pipe', 2] });
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
      '@': 'files'
    },
    args
  );

  if(params.layers)
    outputList = params.layers
      .split(/\W+/g)
      .map(n => ucfirst(n))
      .map(n => outputTypes[n]);

  if(!params.outdir) params.outdir = '/tmp';

  if(params.outdir) mkdir(params.outdir, 0o775);

  if(args.length == 0) args = ['/dev/stdin'];

  for(let arg of params['@']) {
    let files = [];

    for(let options of outputList) {
      if(params.outdir) options.outdir = params.outdir;

      let cmd = EagleToGerber(arg, options);
      let outputFile = cmd[cmd.indexOf('-o') + 1];
      files.push(outputFile);

      //  console.log(`Generating '${outputFile}'`);
      console.log(`Command: ${cmd.join(' ')}`);

      ExecTool(...cmd);
    }

    console.log(`Generated files:\n${files.join('\n')}`);

    chdir(params.outdir);

    ExecTool('zip', '-9', '-r', '-m', `../${arg}.zip`, '.');
  }
}

main(...scriptArgs.slice(1));
