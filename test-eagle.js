import { EagleElement } from './lib/eagle/element.js';
import { EagleDocument } from './lib/eagle/document.js';
import { EagleProject } from './lib/eagle/project.js';
import { EaglePath } from './lib/eagle/locator.js';
import { Line, Point, BBox } from './lib/geom.js';
import Util from './lib/util.js';
import fs, { promises as fsPromises } from 'fs';
import deep from './lib/deep.js';
import DeepDiff from 'deep-diff';
import { Console } from 'console';
import { inspect, toXML, dump } from './lib/eagle/common.js';
import { JsonPointer, JsonReference } from './lib/json-pointer.js';
import { RGBA } from './lib/dom.js';
import { Graph, Edge, Node } from './lib/fd-graph.js';
import ptr from './lib/json-ptr.js';
import util from 'util';

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 3, colors: true }
});

function xmlize(obj, depth = 2) {
  return obj.toXML ? obj.toXML().replace(/>\s*</g, '>\n    <') : EagleDocument.toXML(obj, depth).split(/\n/g)[0];
}

function testLocator() {
  let testobj = [0, 1, 2, { name: 'roman', children: ['x', 'y', { id: 1, items: ['a', 'b', 'c'] }] }];
  let l = new EaglePath([3, 'children', 2, 'items', -2]);
  let a = [l.slice(), l.slice()];
  a[1][0] = 'x';
  let b = [l.prevSibling, l.nextSibling, l.parent];
}

function testProxyTree() {
  let tree = Util.proxyTree((path, key, value) => {
    return true;
  });
  tree.a.b.c.d('test');
  tree.a.b.c.d.e = 0;
  tree.a.b.c.d.e[0] = 1;
}

function testProxyClone() {
  let obj = {
    blah: [1, 2, 3, 4],
    test: { text: 'eruoiewurew', name: 'haha' },
    num: 41
  };
  let clone = Util.proxyClone(obj);
  obj.addProp = '1234';
  clone.newProp = 'test';
}

function testJsonPointer() {
  var data = {
    legumes: [
      { name: 'pinto beans', unit: 'lbs', instock: 4 },
      { name: 'lima beans', unit: 'lbs', instock: 21 },
      { name: 'black eyed peas', unit: 'lbs', instock: 13 },
      { name: 'plit peas', unit: 'lbs', instock: 8 }
    ]
  };
  var pointer = ptr.append(ptr.nil, 'legumes', 0);
  var pointer2 = ptr.append(pointer, 'name');
  ptr.assign(pointer2)(data, 'test name');
}

const filesystem = {
  readFile(filename) {
    let data = fs.readFileSync(filename).toString();
    return data;
  },
  writeFile(filename, data, overwrite = true) {
    return fs.writeFileSync(filename, data, { flag: overwrite ? 'w' : 'wx' });
  },
  exists(filename) {
    return fs.existsSync(filename);
  }
};

var graph, project;

async function testGraph(proj) {
  project = proj;
  const { board } = proj;
  graph = new Graph();

  for(let element of board.getAll('element')) {
    const { x, y } = element;
    const { attributes } = element.raw;
    let lib = board.get(e => e.tagName == 'library' && e.attributes['name'] == attributes['library']);
    const pkg = lib.get(e => e.tagName == 'package' && e.attributes['name'] == attributes['package']);
    //console.log('lib:', lib, ' package:', pkg);
    const bb = element.getBounds();
    let rect = bb.rect;
    let pos = rect.center;
    pos = pos.round(0.127);
    const n = graph.addNode(element.name);
    Object.assign(n, { ...rect, ...pos, label: element.name });
    n.width = rect.width;
    n.height = rect.height;
    console.log(`element ${n.label} GraphNode`, n);
    let pads = [...pkg.getAll('pad')];
    let padNames = pads.map(p => p.name);
    for(let pad of pads) {
      const { diameter, drill, name, rot, shape, stop, x, y } = pad.raw.attributes;
      console.log(`Package '${pkg.name}' Pad '${name}'  ${pad.xpath()}`);
    }
    console.log(`Package '${pkg.name}' Pads: ${padNames}`);
  }

  for(let [name, signal] of board.signals) {
    console.log(`Signal ${name}:`, signal);
    for(let contactref of signal.getAll('contactref')) {
      const elementName = contactref.attributes['element'];
      const element = board.get({ tagName: 'element', name: elementName });
      const { name, library, package: pkg, value, x, y } = element.raw.attributes;
      console.log(`Element '${name}' ${library} ${pkg} ${value} ${Util.toString({ x, y })}`);
      {
        const { name, children } = element.package;
        //     console.log(`Package '${name}'`, children);
      }
      console.log(`${name} ${pkg} GraphEdge`);
    }
  }
}

async function testEagle(filename) {
  let proj = new EagleProject(filename, filesystem);
  let { board, schematic } = proj;
  const getPackage = (d, e) => {
    let part;
    if(e.tagName == 'instance') {
      part = e.part;
      let deviceset = part.deviceset;
      const device = deviceset.get('device', part.attributes.device);
      return device.package;
    }

    return e.package;
  };
  const packages = {
    board: [...board.getAll('package')],
    schematic: [...schematic.getAll('package')]
  };
  let e = board.elements.T1;
  let parts = schematic.parts;
  let firstPart = parts[0];
  proj.updateLibrary('c');
  //console.log('board.libraries.list:', board.libraries.list);
  //console.log('[...board.libraries]:', [...board.libraries]);
  //console.log('board.libraries:', board.libraries);
  for(let [libName, lib] of board.libraries) {
    //console.log('lib:', lib);
    for(let [pkgName, pkg] of lib.packages)
      for(let pad of pkg.children) {
        if(pad.tagName !== 'pad') continue;
        pad.setAttribute('drill', '0.7');
        pad.setAttribute('diameter', '1.778');
        pad.removeAttribute('stop');
        pad.removeAttribute('rot');
        pad.removeAttribute('shape');
      }
  }

  let cmds = [];
  for(let elem of board.elements.list) {
    cmds.push(`MOVE ${elem.name} ${elem.pos};`);
    if(elem.rot) cmds.push(`ROTATE ${elem.rot} ${elem.name};`);
  }

  //console.log(cmds.join(' '));
  for(let description of board.getAll('description')) {
  }

  proj.saveTo('.', true);

  console.log('board:', board);

  for(let instance of schematic.getAll(e => e.tagName == 'instance')) {
    const { part } = instance;
    //  console.log("instance:",{instance, part});
  }

  return proj;
}

(async () => {
  let args = process.argv.slice(2);
  if(args.length == 0) args.unshift('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3');
  for(let arg of args) {
    try {
      let project = await testEagle(arg);
      //console.log(project);
      // await testGraph(project);
    } catch(err) {
      const stack = err.stack;
      console.log('err:' + err.message);
      throw err;
    }
  }

  console.log('palette:');
})();
