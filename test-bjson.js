import PortableFileSystem from './lib/filesystem.js';
import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import deep from './lib/deep.js';
import path from './lib/path.js';
import tXml from './lib/tXml.js';
import { toXML } from './lib/xml.js';
import Tree from './lib/tree.js';
import { Path } from './lib/json.js';
import Alea from './lib/alea.js';
import * as diff from './lib/json/diff.js';
import inspect from './lib/objectInspect.js';

let filesystem;
let prng = new Alea().seed(Date.now());

async function readBJSON(filename) {
  let data = filesystem.readFile(filename, null);
  let obj = await import('bjson')
    .then(({ read }) => read(data, 0, data.byteLength))
    .catch(err => console.log(err));
  return obj;
}
function readXML(filename) {
  //console.log('readXML', filename);
  let data = filesystem.readFile(filename);
  let xml = tXml(data);
  //console.log('xml:', xml);
  return xml;
}
function dumpFile(name, data) {
  console.log('dumpFile', { name });
  if(typeof data == 'string' && !data.endsWith('\n')) data += '\n';

  if(name == '-' || typeof name != 'string') {
    //  let stdout = filesystem.fdopen(1, 'r');
    let buffer = data instanceof ArrayBuffer ? data : filesystem.bufferFrom(data);
    filesystem.write(1, buffer, 0, buffer.byteLength);
    return;
  }

  if(Util.isArray(data)) data = data.join('\n');
  q;
  // if(typeof data != 'string') data = '' + data;
  filesystem.writeFile(name, data);
  console.log(`Wrote ${name}: ${data.length} bytes`);
}

const push_back = (arr, ...items) => [...(arr || []), ...items];
const push_front = (arr, ...items) => [...items, ...(arr || [])];
const tail = arr => arr[arr.length - 1];

async function main(...args) {
  await ConsoleSetup({ depth: 20, colors: true, breakLength: 80 });
  filesystem = await PortableFileSystem();

  let params = Util.getOpt({
      output: [true, null, 'o'],
      input: [true, null, 'i'],
      xml: [true, null, 'x'],
      json: [true, null, 'j'],
      tag: [true, null, 't'],
      include: [true, null, 'I'],
      exclude: [true, null, 'X'],
      'no-remove-empty': [false, null, 'E'],
      '@': 'input,output,xml'
    },
    Util.getArgs().slice(1)
  );
  console.log('main', args, params);
  if(params['@'].length == 0 && !params.input) {
    console.log(`Usage: ${Util.getArgs()[0]} <...files>`);
    return 1;
  }

  let { input: filename = '-' } = params;

  let basename = path.basename(filename, /\.[^.]+$/g);
  let outfile = params.output || '-'; /* ||   basename + '.out.xml'*/
  let xmlfile = params.xml || basename + '.out.xml';
  let jsonfile = params.json || basename + '.out.json';
  let tagkey = params.tag || 'type';
  let { include, exclude } = params;

  let cmds = args;
  let newObj = {};
  let xmlData;

  try {
    let js = await readBJSON(filename);
    let json = JSON.stringify(js, null, '  ');

    //   await import('bjson').then(({ read, write }) => json = write(xml)).catch(err => console.error(err));

    dumpFile(outfile, json);

    // dumpFile(xmlfile, toXML(xmlData));
  } catch(err) {
    let st = Util.stack(err.stack);
    // console.log(err.message, '\n', st.toString()); //st.map(f =>  Util.toString(f)));
    throw err;
  }
}
Util.callMain(main, true);
