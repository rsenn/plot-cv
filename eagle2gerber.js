import { spawn } from 'child_process';
import * as path from 'path';
import { getOpt } from 'util';
import { Console } from 'console';
#!/usr/bin/env qjsm
let optionsArray = [{ front: true }, { back: true }, { side: 'outline' }, { drill: true }];

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
    for(let options of optionsArray) {
      let cmd = EagleToGerber(arg, { ...options, outdir });
      let outputFile = cmd[cmd.indexOf('-o') + 1];
      files.push(outputFile);

      //  console.log(`Generating '${outputFile}'`);
      console.log(`Command: ${cmd.join(' ')}`);

      let result = ExecTool(...cmd);
      console.log(`Result:`, result);
    }
    console.log(`Generated files:\n${files.join('\n')}`);
  }
}

main(...scriptArgs.slice(1));