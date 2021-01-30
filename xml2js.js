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
    let xml = readXML(filename);
    let tree = new Tree(xml);
    for(let [node, path] of tree) {
      let parentPath = path.slice(0, -1);
      let key = tail(path);
      let parentNode = tree.at(parentPath);
      if(key == 'children') {
        let { children } = parentNode;
        delete parentNode.children;
        Object.assign(parentNode, children.length ? { ...parentNode, children } : parentNode);
      }

      /*  if(key == 'tagName') {
        if(key) delete parentNode[key];
        Object.assign(parentNode, { ...parentNode, [tagkey]: node, ...parentNode });
      }*/
      if(key == 'attributes') {
        let parent = tree.at(path.slice(0, -1)) || tree.parentNode(node);
        let { attributes } = parent;
        delete parent.attributes;

        for(let key in attributes)
          if(!isNaN(+attributes[key])) attributes[key] = +attributes[key];

        tree.replace(parent, { ...parent, ...attributes });
      }

      if(Array.isArray(node) && node.length == 0) tree.removeAt(path);
      if(Util.isObject(node) && Util.isEmpty(node)) tree.removeAt(path);
    }
    let js;
    let json = JSON.stringify(xml, null, '  ');

    await import('bjson')
      .then(({ read, write }) => (json = write(xml)))
      .catch(err => console.error(err));

    dumpFile(jsonfile, json);

    let flat = tree.flat();
    let rebuilt = [];

    if(include) {
      let pred = Util.predicate(`(<${include}[ \\t>/].*|.*\\s${include}=)`, (arg, pred) => {
        const { tagName, children, ...attributes } = arg;
        let node = { tagName, attributes };
        let str = toXML(node);

        if(pred(str)) {
          // console.log('pred:', { str, pred: pred.valueOf() });
          return true;
        }
      });

      let output = [];
      newObj = [];
      for(let [path, node] of tree.filter((n, p) => n.tagName !== undefined)) {
        let { tagName, children, ...attributes } = node;
        let str = toXML({ tagName, children, attributes });

        if(pred(node)) {
          output.push(str);
          newObj.push(node);
        }
      }
      xmlData = output.map(str => tXml(str));
      js = newObj
        .map(obj =>
          inspect(obj, {
            depth: Number.MAX_SAFE_INTEGER,
            multiline: false,
            breakLength: 80,
            indent: 2,
            colors: false
          })
        )
        .join(',\n');
    } else {
      xmlData = xml[0];
      newObj = deep.clone(xmlData);
      js = inspect(newObj, {
        depth: Number.MAX_SAFE_INTEGER,
        multiline: true,
        breakLength: 80,
        indent: 2,
        colors: false
      });
    }
    //console.log('newObj:', newObj);

    dumpFile(outfile, js);
    dumpFile(xmlfile, toXML(xmlData));
  } catch(err) {
    let st = Util.stack(err.stack);
    // console.log(err.message, '\n', st.toString()); //st.map(f =>  Util.toString(f)));
    throw err;
  }
}
Util.callMain(main, true);
