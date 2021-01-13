import PortableFileSystem from './lib/filesystem.js';
import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import deep from './lib/deep.js';
import path from './lib/path.js';
import tXml from './lib/tXml.js';
import Tree from './lib/tree.js';
import { toXML, Path } from './lib/json.js';
import Alea from './lib/alea.js';
import * as diff from './lib/json/diff.js';
import inspect from './lib/objectInspect.js';

let filesystem;
let prng = new Alea().seed(Date.now());

function readXML(filename) {
  //console.log('readXML', filename);
  let data = filesystem.readFile(filename);
  let xml = tXml(data);
  //console.log('xml:', xml);
  return xml;
}
function dumpFile(name, data) {
  console.log('dumpFile', { name });
  if(!data.endsWith('\n')) data += '\n';

  if(name == '-' || typeof name != 'string') {
    let stdout = filesystem.fdopen(1, 'r');
    let buffer = filesystem.bufferFrom(data);
    filesystem.write(1, buffer, 0, buffer.byteLength);
    return;
  }

  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;
  filesystem.writeFile(name, data + '\n');
  console.log(`Wrote ${name}: ${data.length} bytes`);
}

const push_back = (arr, ...items) => [...(arr || []), ...items];
const push_front = (arr, ...items) => [...items, ...(arr || [])];
const tail = arr => arr[arr.length - 1];

async function main(...args) {
  await ConsoleSetup({ depth: 10 });
  filesystem = await PortableFileSystem();

  let params = Util.getOpt({
      output: [true, null, 'o'],
      input: [true, null, 'i'],
      xml: [true, null, 'x'],
      json: [true, null, 'j'],
      tag: [true, null, 't'],
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

  let cmds = args;
  let newObj = {};
  let xmlData;

  try {
    let xml = readXML(filename);
    let tree = new Tree(xml);
    for(let [path, node] of tree) {
      let parentPath = path.slice(0, -1);
      let key = tail(path);
      let parentNode = tree.at(parentPath);
      if(key == 'children') {
        let { children } = parentNode;
        delete parentNode.children;
        Object.assign(parentNode, children.length ? { ...parentNode, children } : parentNode);
      }
      /*if(parentNode && typeof node == 'string')  {
        if(!isNaN(+node))
          node = +node;
        parentNode[key] = node;
      }*/

      if(key == 'tagName') {
        if(key) delete parentNode[key];
        Object.assign(parentNode, { ...parentNode, [tagkey]: node, ...parentNode });
      }
      if(key == 'attributes') {
        let parent = tree.at(path.slice(0, -1)) || tree.parentNode(node);
        let { attributes } = parent;
        delete parent.attributes;

        for(let key in attributes) if(!isNaN(+attributes[key])) attributes[key] = +attributes[key];

        tree.replace(parent, { ...parent, ...attributes });
      }

      if(Array.isArray(node) && node.length == 0) tree.remove(node);
      if(Util.isObject(node) && Util.isEmpty(node)) tree.remove(node);
    }
    let js = inspect(xml, {
      depth: Number.MAX_SAFE_INTEGER,
      multiline: true,
      breakLength: 80,
      indent: 2,
      colors: false
    });
    dumpFile(outfile, js);
    let json = JSON.stringify(xml, null, '  ');
    dumpFile(jsonfile, json);

    let flat = tree.flat();
    console.log('flat:', flat);

    let rebuilt = tree.build(flat);
    console.log('rebuilt:', rebuilt);

    xmlData = xml[0];
    newObj = deep.clone(xml[0]);

    /*= deep.flatten(xml[0],
      new Map(),
      (v, p) =>
        (typeof v != 'object' && p.indexOf('attributes') == -1) ||
        (p.length && p.indexOf('attributes') == p.length - 1),
      (p, v) => [new Path(p), v]
    );*/

    dumpFile(xmlfile, toXML(newObj));
  } catch(err) {
    let st = Util.stack(err.stack);
    // console.log(err.message, '\n', st.toString()); //st.map(f =>  Util.toString(f)));
    throw err;
  }
}
Util.callMain(main, true);
