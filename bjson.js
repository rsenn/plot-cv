import { getOpt } from './lib/misc.js';
import path from './lib/path.js';
import filesystem from 'fs';
import { Console } from 'console';

const WriteBJSON = (filename, obj) =>
  import('bjson.so').then(({ write }) => {
    let data = write(obj);
    console.log('WriteBJSON', filename, data);
    return filesystem.writeFileSync(filename, data);
  });

const WriteJSON = async (filename, obj) => filesystem.write(filename, JSON.stringify(obj, null, 2) + '\n');

let ReadBJSON = filename => import('bjson.so').then(({ read }) => read(filesystem.readFileSync(filename, null)));

const ReadJSON = filename => JSON.parse(filesystem.readFileSync(filename, 'utf-8'));

async function main(...args) {
  globalThis.console = new Console(process.stderr, { depth: Infinity });
  let space = 2;
  let params = getOpt(
    {
      output: [true, null, 'o'],
      indent: [true, v => (space = !isNaN(+v) ? +v : v), 'i'],
      'output-dir': [true, null, 'd'],
      '@': 'input'
    },
    args
  );

  let output = params.output ? filesystem.openSync(params.output, 'w+') : filesystem.stdout;

  if(params.output && params['@'].length > 1) throw new Error(`Output file specified as '${params.output}', but got ${params['@'].length} input files`);

  console.log('params', params);
  if(params.output && !('indent' in params)) space = '';

  for(let arg of params['@']) {
    let base = path.basename(arg, /\.[^./]*$/);
    let binary = true;
    let obj = await ReadBJSON(arg).catch(e => null);
    if(!obj) {
      console.log('ReadJSON', arg);
      obj = await ReadJSON(arg);
      binary = false;
    }

    if(binary) {
      let json = JSON.stringify(obj, null, space);
      filesystem.writeSync(output, json + '\n');
    } else {
      WriteBJSON(base + '.bjson', obj);
    }
  }
}

function WriteFile(name, data, verbose = true) {
  if(typeof data == 'string' && !data.endsWith('\n')) data += '\n';
  let ret = filesystem.writeFile(name, data);

  if(verbose) console.log(`Wrote ${name}: ${ret} bytes`);
}

main(...scriptArgs.slice(1));
