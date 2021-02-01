import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './lib/consoleSetup.js';
import PortableSpawn from './lib/spawn.js';
import { AcquireReader } from './lib/stream/utils.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import deep from './lib/deep.js';
import Tree from './lib/tree.js';
import { Type, Compile, AstDump, NodeType, NodeName, GetLoc, GetType } from './clang-ast.js';

const WriteBJSON = async (filename, obj) =>
  await import('bjson').then(({ write }) => {
    let data = write(obj);
    return WriteFile(filename, data);
  });

const ReadBJSON = async filename =>
  await import('bjson').then(({ read }) => {
    let data = filesystem.readFile(filename, null);
    return read(data, 0, data.byteLength);
  });

const ReadJSON = async filename => {
  let data = filesystem.readFile(filename);
  return JSON.parse(data);
};

async function main(...args) {
  console.log('main(', ...args, ')');
  await ConsoleSetup({ breakLength: 120, depth: 10 });
  await PortableFileSystem(fs => (filesystem = fs));
  await PortableSpawn(fn => (spawn = fn));

  let params = Util.getOpt({
      output: [true, null, 'o'],
      indent: [true, null, 'i'],
      'output-dir': [true, null, 'd'],
      '@': 'input'
    },
    args
  );
  console.log('main', params);

  let output = params.output ? filesystem.open(params.output, 'w+') : filesystem.stdout;

if(params.output && params['@'].length > 1)
  throw new Error(`Output file specified as '${params.output}', but got ${params['@'].length} input files`);
  
  for(let arg of params['@']) {
    let base = path.basename(arg, /\.[^./]*$/);
    let binary = true;
    let obj = await ReadBJSON(arg).catch(console.log);
    if(!obj) {
      obj = await ReadJSON(arg);
      binary = false;
    }

    if(binary) {
      let json = JSON.stringify(obj, null, params.indent ?? 2);
      filesystem.write(output, json);
    } else {
      WriteBJSON(base+'.bjson', obj);
    }
  }
}

Util.callMain(main, true);

function WriteFile(name, data, verbose = true) {
  if(typeof data == 'string' && !data.endsWith('\n')) data += '\n';
  let ret = filesystem.writeFile(name, data);

  if(verbose) console.log(`Wrote ${name}: ${ret} bytes`);
}
