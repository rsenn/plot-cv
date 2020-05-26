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
import { Graph, Edge, Node } from './lib/fd-graph.js';
import ptr from './lib/json-ptr.js';
import util from 'util';

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 2, colors: true }
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
    //console.log(`Read ${filename} ${data.length} bytes`);
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
  graph = new Graph();

  for(let element of proj.board.getAll('element')) {
    const { x, y } = element;
    const pkg = element.package;
    const bb = element.getBounds();
    let rect = bb.rect;
    let pos = rect.center;

    pos = pos.round(0.127);

    const n = graph.addNode(element.name);

    Object.assign(n, { ...rect, ...pos });
    n.width = rect.width;
    n.height = rect.height;

    console.log(`element ${n.label} GraphNode`, n);

    let pads = [...pkg.getAll('pad')];
    let padNames = pads.map(p => p.name);

    for(let pad of pads) {
      const { diameter, drill, name, rot, shape, stop, x, y } = pad;

      console.log(`Package '${pkg.name}' Pad '${pad.name}'  ${pad.xpath()}`);
    }
    console.log(`Package '${pkg.name}' Pads: ${padNames}`);
  }

  for(let signal of proj.board.signals.list) {
    console.log(`${signal} GraphNode`);

    for(let contactref of signal.getAll('contactref')) {
      const { element } = contactref;
      const pad = contactref.attributes.pad;

      console.log(`Signal '${signal.name}' ${element.name}.${pad}`);
      console.log(`${element.name} ${element.attributes.pad} GraphEdge`);
    }
  }

  //  console.log("testGraph", { project, graph });
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
    board: [...proj.board.getAll('package')],
    schematic: [...proj.schematic.getAll('package')]
  };
  let e = proj.board.elements.T1;

  //console.log("proj.board.packages:", proj.board.cache);

  //console.log("e:", e);
  //console.log("e.package:", e.package);

  //console.log("schematic.cache.libraries:", schematic.cache.libraries);
  //console.log("schematic.cache.parts:", Util.className(schematic.cache.parts));
  //console.log("schematic.cache.instances:", schematic.cache.instances);

  let parts = schematic.parts;
  //console.log("schematic.parts.ref:", parts.ref);
  //console.log("schematic.parts.raw:", parts.raw);
  //console.log("schematic.parts:", Util.className(schematic.parts));
  //console.log("schematic.parts.keys():", schematic.parts.keys());
  //console.log("schematic.parts.entries():", schematic.parts.entries());
  //console.log("schematic.layers:", schematic.layers);

  //console.log("schematic.parts.list:", schematic.parts.list);
  //console.log("schematic.parts.list[0]:", schematic.parts.list[0]);

  let firstPart = parts[0];

  proj.updateLibrary('c');

  //console.log(`proj.library.c:`, proj.library.c);

  for(let lib of board.libraries.list)
    for(let pkg of lib.packages.list)
      for(let pad of pkg.children) {
        if(pad.tagName !== 'pad') continue;

        pad.setAttribute('drill', '0.7');
        pad.setAttribute('diameter', '1.778');
        pad.removeAttribute('stop');
        pad.removeAttribute('rot');
        pad.removeAttribute('shape');
      }
  let cmds = [];

  for(let elem of board.elements.list) {
    cmds.push(`MOVE ${elem.name} ${elem.pos};`);
    if(elem.rot) cmds.push(`ROTATE ${elem.rot} ${elem.name};`);
  }
  console.log(cmds.join(' '));
  /*
  const signals = board.find("signals");
  //console.log("signals.path:" + signals.path);

  for(let wire of signals.getAll(v => v.tagName == "wire" && ["1", "16"].includes(v.attributes.layer))) {
    //console.log("wire:", wire);

    let line = Line.bind(wire.attributes, null, k => v => (v === undefined ? +wire.attributes[k] : (wire.attributes[k] = "" + v)));
    let pointA = Point.bind(wire.attributes, ["x1", "y1"], k => v => (v === undefined ? +wire.attributes[k] : (wire.attributes[k] = "" + v)));
    let copy = line.clone();

    line.round(1.27);

    if(!line.equals(copy)) {
      //console.log("line:", line);
      //console.log("copy:", copy);
      //console.log("diff:", line.diff(copy));
      console.log("save:", new Line(wire.attributes));

      //console.log("\n");
    }
  }

  for(let element of board.getAll("element")) {
    let position = Point.bind(element.attributes, null, k => v => (v === undefined ? +element.attributes[k] : (element.attributes[k] = "" + v)));

    let copy = position.clone();
    position.round(1.27);
    if(!position.equals(copy)) {
      console.log("position:", position);

      console.log("save:", Point.bind(element.attributes));
    }
  }

  for(let change of board.changes) {
    const { path, ...item } = change;

    console.log(`board change:`, util.inspect(item, { depth: 5 }));
  } 
*/

  //[change.path.reverse()[0],change.rhs]));
  // console.log("schematic changes:",schematic.changes);

  for(let description of board.getAll('description')) {
    // console.log("description:",description.raw);
  }

  //console.log("board.raw:", board.raw);
  proj.saveTo('.', true);
  //  proj.board = board;proj.schematic = schematic;
  return proj;
}

(async () => {
  let args = process.argv.slice(2);
  if(args.length == 0) args.unshift('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3');

  for(let arg of args) {
    try {
      //testLocator();

      let project = await testEagle(arg);
      console.log(project);

      await testGraph(project);
      //console.log("r:", r);
    } catch(err) {
      const stack = err.stack;
      //console.log("err:", err);
      //console.log("stack:", stack);
      throw err;
    }
  }
})();
