import filesystem from 'fs';
import Alea from './lib/alea.js';
import deep from './lib/deep.js';
import { SVG } from './lib/dom/svg.js';
import { Matrix, Point, Size, TransformationList } from './lib/geom.js';
import { Path } from './lib/json.js';
import * as path from './lib/path.js';
import Tree from './lib/tree.js';
import tXml from './lib/tXml.js';
import { toXML } from './lib/xml.js';
let prng = new Alea().seed(Date.now());

function readXML(filename) {
  let data = filesystem.readFileSync(filename);
  let xml = tXml(data);

  return xml;
}

function WriteFile(name, data) {
  console.log('WriteFile', { name });
  if(!data.endsWith('\n')) data += '\n';

  if(name == '-' || typeof name != 'string') {
    let buffer = filesystem.bufferFrom(data);
    filesystem.write(1, buffer, 0, buffer.byteLength);
    return;
  }

  if(Array.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;
  filesystem.writeFile(name, data + '\n');
  console.log(`Wrote ${name}: ${data.length} bytes`);
}

const push_back = (arr, ...items) => [...(arr || []), ...items];

const push_front = (arr, ...items) => [...items, ...(arr || [])];

const tail = (arr, n = 0) => arr[arr.length - 1 - n];

async function main(...args) {

  let params = Util.getOpt(
    {
      output: [true, null, 'o'],
      input: [true, null, 'i'],
      xml: [true, null, 'x'],
      json: [true, null, 'j'],
      tag: [true, null, 't'],
      transform: [true, null, 't'],
      'no-remove-empty': [false, null, 'E'],
      '@': 'input'
    },
    scriptArgs.slice(1)
  );
  console.log('main', args, params);
  if(params['@'].length == 0 && !params.input) {
    console.log(`Usage: ${scriptArgs[0]} <...files>`);
    return 1;
  }

  let { input: filename = '-' } = params;

  let basename = path.basename(filename, /\.[^.]+$/g);
  let outfile = params.output || '-';
  let xmlfile = params.xml || basename + '.out.xml';
  let jsonfile = params.json || basename + '.out.json';
  let tagkey = params.tag || 'type';
  let { include, exclude } = params;

  let cmds = args;
  let newObj = {};
  let xmlData;
  let transforms = new WeakMap();
  let positioned = new Set();

  try {
    let xml = readXML(filename);
    let tree = new Tree(xml);
    for(let [node, path] of tree) {
      if(tail(path) == 'transform') {
        let parentPath = path.slice(0, -2);
        let parentNode = tree.at(parentPath);
        let t = new TransformationList(node);

        transforms.set(parentNode, t);

        delete parentNode.attributes.transform;
      }
    }

    for(let [node, acc] of tree.each(
      null,
      (acc, node) => {
        let parentNode = tree.parentNode(node);
        let t = transforms.get(node);
        if(t) return acc.concat(t);
        return acc;
      },
      new TransformationList(params.transform)
    )) {
      if(!node.attributes || !hasCoord(node.attributes)) continue;

      positioned.add(node);

      let matrices = acc.toMatrices();
      let matrix = matrices.reduce((acc, m) => acc.multiply(m), Matrix.identity());
      let path = new Path(tree.pathOf(node));
      let parentNode = tree.parentNode(node);

      if(acc.length < 1) continue;
      console.log('transformation', acc);
      console.log('matrix', matrix.decompose());

      switch (node.tagName) {
        case 'circle': {
          let { cx, cy, r } = node.attributes;

          let { x, y } = new Point(+cx, +cy).transform(matrix).round(0.00001, 6);
          let radius = new Size(+r, +r).transform(matrix).round(0.00001, 6).width;

          console.log('transform circle', { x, y, radius });

          node.attributes['cx'] = x;
          node.attributes['cy'] = y;
          node.attributes['r'] = radius;
          break;
        }
        case 'text': {
          let { x, y } = node.attributes;

          let p = new Point(+x, +y).transform(matrix).round(0.00001, 6);

          console.log(`transform '${node.tagName}'`, p);

          node.attributes['x'] = p.x;
          node.attributes['y'] = p.y;

          break;
        }
        case 'path': {
          let { d } = node.attributes;

          let path = SVG.parsePath(d).toAbsolute();

          path.commands = path.commands.map(command => {
            console.log('command:', command);
            switch (command.name) {
              case 'M': {
                const [x, y] = command.args;
                let p = new Point(+x, +y).transform(matrix);
                command.args = [p.x, p.y];
                break;
              }
              case 'H': {
                const h = command.args;
                command.args = h.map(arg => new Point(+arg, 0).transform(matrix).x);
                break;
              }
              case 'V': {
                const v = command.args;
                command.args = v.map(arg => new Point(+arg, 0).transform(matrix).y);
                break;
              }
              case 'A': {
                const [rx, ry, angle, largeArc, arcSweep, x, y] = command.args;
                let s = new Size(+rx, +ry).transform(matrix);
                let p = new Point(+x, +y).transform(matrix);
                command.args = [s.width, s.height, angle, largeArc, arcSweep, p.x, p.y];
                break;
              }
              case 'Z': {
                break;
              }
              default: {
                let message = `No such path command '${command.name}'`;
                console.error(message);
                throw new Error(message);
              }
            }
            command.args = command.args.map(arg => (typeof arg == 'number' ? Util.roundTo(arg, 0.00001, 6) : arg));

            return command;
          });

          node.attributes['d'] = path /*.toRelative()*/
            .toString();
          // console.log('path:', path);
          break;
        }
        default: {
          let message = `No such element '${node.tagName}'`;
          throw new Error(message);
          break;
        }
      }
    }

    /*  for(let node of positioned) {
      console.log("node:", node);
    }*/

    xmlData = xml[0];
    newObj = deep.clone(xmlData);
    WriteFile(outfile, toXML(xmlData));

    function hasCoord(node) {
      let keys = Object.keys(node);
      return keys.some(key => ['x', 'y', 'cx', 'cy', 'd', 'x1', 'y1', 'x2', 'y2'].indexOf(key) != -1);
    }
    function coordKeys(node) {
      let keys = Object.keys(node);
      return keys.filter(key => /(x|y|cx|cy|d|x1|y1|x2|y2)$/.test(key));
    }
  } catch(err) {
    let st = Util.stack(err.stack);

    throw err;
  }
}

main(...scriptArgs.slice(1));