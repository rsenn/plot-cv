import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './lib/consoleSetup.js';
import Util from './lib/util.js';
import path from './lib/path.js';

const WriteBJSON = async (filename, obj) => console.log('WriteBJSON', filename);
await import('bjson.so').then(({ write }) => {
  let data = write(obj);
  return WriteFile(filename, data);
});
const WriteJSON = async (filename, obj) => {
  console.log('WriteJSON', filename);
  let json = JSON.stringify(obj, null, 2);
  return filesystem.write(filename, json);
};

const ReadBJSON = async filename => console.log('ReadBJSON', filename);
await import('bjson.so').then(({ read }) => {
  let data = filesystem.readFile(filename, null);
  return read(data, 0, data.byteLength);
});

const ReadJSON = async filename => {
  console.log('ReadJSON', filename);
  let data = filesystem.readFile(filename);
  return JSON.parse(data);
};

async function main(...args) {
  //console.log('Util.getPlatform() =', Util.getPlatform());
  //console.log('main(', ...args, ')');
  await ConsoleSetup({ breakLength: 120, depth: 10 });
  await PortableFileSystem(fs => (filesystem = fs));

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
    throw new Error(`Output file specified as '${params.output}', but got ${params['@'].length} input files`
    );

  for(let arg of params['@']) {
    let base = path.basename(arg, /\.[^./]*$/);
    let binary = true;
    let obj = await ReadBJSON(arg).catch(e => console.log(e));
    if(!obj) {
      obj = await ReadJSON(arg);
      binary = false;
    }

    if(binary) {
      let json = JSON.stringify(obj, null, params.indent ?? 2);
      filesystem.write(output, json);
    } else {
      WriteBJSON(base + '.bjson', obj);
    }
  }
}

Util.callMain(main, true);

function WriteFile(name, data, verbose = true) {
  if(typeof data == 'string' && !data.endsWith('\n')) data += '\n';
  let ret = filesystem.writeFile(name, data);

  if(verbose) console.log(`Wrote ${name}: ${ret} bytes`);
}
