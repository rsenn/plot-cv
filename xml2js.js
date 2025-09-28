#!/usr/bin/env qjsm
import filesystem from 'fs';
import Alea from './lib/alea.js';
import deep from './lib/deep.js';
import * as path from './lib/path.js';
import Tree from './lib/tree.js';
import tXml from './lib/tXml.js';
import { toXML } from './lib/xml.js';

let prng = new Alea().seed(Date.now());

function readXML(filename) {
  //console.log('readXML', filename);
  let data = filesystem.readFileSync(filename);
  let xml = tXml(data);
  //console.log('xml:', xml);
  return xml;
}

function WriteFile(name, data) {
  console.log('WriteFile', { name });
  if(typeof data == 'string' && !data.endsWith('\n')) data += '\n';

  if(name == '-' || typeof name != 'string') {
    //  let stdout = filesystem.fdopen(1, 'r');
    let buffer = data instanceof ArrayBuffer ? data : filesystem.bufferFrom(data);
    filesystem.write(1, buffer, 0, buffer.byteLength);
    return;
  }

  if(Array.isArray(data)) data = data.join('\n');
  // if(typeof data != 'string') data = '' + data;
  filesystem.writeFile(name, data);
  console.log(`Wrote ${name}: ${data.length} bytes`);
}

const push_back = (arr, ...items) => [...(arr || []), ...items];

const push_front = (arr, ...items) => [...items, ...(arr || [])];
const tail = arr => arr[arr.length - 1];

async function main(...args) {
  let params = Util.getOpt(
    {
      output: [true, null, 'o'],
      input: [true, null, 'i'],
      xml: [true, null, 'x'],
      json: [true, null, 'j'],
      tag: [true, null, 't'],
      include: [true, null, 'I'],
      exclude: [true, null, 'X'],
      'no-remove-empty': [false, null, 'E'],
      '@': 'input,output,xml',
    },
    scriptArgs.slice(1),
  );
  console.log('main', args, params);
  if(params['@'].length == 0 && !params.input) {
    console.log(`Usage: ${scriptArgs[0]} <...files>`);
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

        for(let key in attributes) if(!isNaN(+attributes[key])) attributes[key] = +attributes[key];

        tree.replace(parent, { ...parent, ...attributes });
      }

      if(Array.isArray(node) && node.length == 0) tree.removeAt(path);
      if(Util.isObject(node) && Util.isEmpty(node)) tree.removeAt(path);
    }
    let js;
    let json = JSON.stringify(xml, null, '  ');

    await import('bjson.so').then(({ read, write }) => (json = write(xml))).catch(err => console.error(err));

    WriteFile(jsonfile, json);

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
          [Symbol.for('nodejs.util.inspect.custom')](obj, {
            depth: Number.MAX_SAFE_INTEGER,
            multiline: false,
            breakLength: 80,
            indent: 2,
            colors: false,
          }),
        )
        .join(',\n');
    } else {
      xmlData = xml[0];
      newObj = deep.clone(xmlData);
      js = [Symbol.for('nodejs.util.inspect.custom')](newObj, {
        depth: Number.MAX_SAFE_INTEGER,
        multiline: true,
        breakLength: 80,
        indent: 2,
        colors: false,
      });
    }
    //console.log('newObj:', newObj);

    WriteFile(outfile, js);
    WriteFile(xmlfile, toXML(xmlData));
  } catch(err) {
    let st = Util.stack(err.stack);
    // console.log(err.message, '\n', st.toString()); //st.map(f =>  inspect(f)));
    throw err;
  }
}

main(...scriptArgs.slice(1));
