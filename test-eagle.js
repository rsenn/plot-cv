import { EagleDocument, EagleProject } from './lib/eagle.js';
import { LineList, Rect } from './lib/geom.js';
import { toXML } from './lib/json.js';
import Util from './lib/util.js';
import fs, { promises as fsPromises } from 'fs';
import deep from './lib/deep.js';
import DeepDiff from 'deep-diff';
import { Console } from 'console';
import { Graph } from './lib/fd-graph.js';
import ptr from './lib/json-ptr.js';
import LogJS from './lib/log.js';

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 2, colors: true }
});

function xmlize(obj, depth = 2) {
  return obj.toXML ? obj.toXML().replace(/>\s*</g, '>\n    <') : EagleDocument.toXML(obj, depth).split(/\n/g)[0];
}

function testLocator() {
  let l = new EaglePath([3, 'children', 2, 'items', -2]);
  let a = [l.slice(), l.slice()];
  a[1][0] = 'x';
}

function testProxyTree() {
  let tree = Util.proxyTree((path, key, value) => true);
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
  let data = {
    legumes: [
      { name: 'pinto beans', unit: 'lbs', instock: 4 },
      { name: 'lima beans', unit: 'lbs', instock: 21 },
      { name: 'black eyed peas', unit: 'lbs', instock: 13 },
      { name: 'plit peas', unit: 'lbs', instock: 8 }
    ]
  };
  let pointer = ptr.append(ptr.nil, 'legumes', 0);
  let pointer2 = ptr.append(pointer, 'name');
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
  },
  realpath(filename) {
    return fs.realpathSync(filename);
  }
};

let graph, project;

async function testGraph(proj) {
  project = proj;
  const { board } = proj;
  graph = new Graph();

  for(let [name, element] of board.elements) {
    const { x, y } = element;
    const { attributes } = element.raw;
    let lib = board.get(e => e.tagName == 'library' && e.attributes.name == attributes.library);
    const pkg = lib.get(e => e.tagName == 'package' && e.attributes.name == attributes.package);
    const bb = element.getBounds();
    let rect = bb.rect;
    let pos = rect.center;
    pos = pos.round(0.127);
    const n = graph.addNode(element.name);
    Object.assign(n, { ...rect, ...pos, label: element.name });
    n.width = rect.width;
    n.height = rect.height;
    let pads = [...pkg.getAll('pad')];
    let padNames = pads.map(p => p.name);
    for(let pad of pads) {
    }
  }

  for(let [name, signal] of board.signals) {
    for(let contactref of signal.getAll('contactref')) {
      const elementName = contactref.attributes.element;
      const element = board.get({ tagName: 'element', name: elementName });
      const { name } = element.raw.attributes;
      {
      }
    }
  }
}

function updateMeasures(board) {
  if(!board) return false;
  let bounds = board.getBounds();
  let measures = board.getMeasures();

  if(measures) {
    //console.log('got measures:', measures);
  } else {
    let rect = new Rect(bounds.rect);
    let lines = rect.toLines(lines => new LineList(lines));
    let { plain } = board;
    plain.remove(e => e.tagName == 'wire' && e.attributes.layer == '47');
    plain.append(
      ...lines.map(line => ({
        tagName: 'wire',
        attributes: { ...line.toObject(), layer: 47, width: 0 }
      }))
    );
    //console.log('no measures:', { bounds, lines }, [...plain]);
    //plain.remove(e => e.attributes.layer == '51');
  }
  //console.log('board.plain:', board.plain);
  return !measures;
}

function alignItem(item) {
  let geometry = item.geometry();
  let oldPos = geometry.clone();

  let newPos = geometry.clone().round(1.27, 2);

  let diff = newPos.diff(oldPos).round(0.0001, 5);

  let before = item.parentNode.toXML();

  //console.log("geometry:", Object.entries(Object.getOwnPropertyDescriptors(geometry)).map(([name,{value}]) => [name, value && Object.getOwnPropertyDescriptors(value)  ]), geometry.x1);

  geometry.add(diff);

  //geometry.y2 = 0;

  let changed = !diff.isNull();

  if(changed) {
    //console.log('before:', before);
    //console.log('after:', item.parentNode.toXML());
    //console.log('geometry:', geometry);
    //console.log('align\n', item.xpath(), '\n newPos:', newPos, '\n diff:', diff, '\n attr:', item.raw.attributes);
  }
  return changed;
}

function alignAll(doc) {
  if(!doc) return false;

  let items = doc.getAll(doc.type == 'brd' ? 'element' : 'instance');
  let changed = false;
  for(let item of items) changed |= alignItem(item);
  let signals_nets = doc.getAll(/(signals|nets)/);
  //console.log('signals_nets:', signals_nets);
  for(let net of signals_nets) for (let item of net.getAll('wire')) changed |= alignItem(item);
  return !!changed;
}

async function testEagle(filename) {
  let proj = new EagleProject(filename, filesystem);
  /*
  LogJS.addAppender(
    class extends LogJS.BaseAppender {
      log(type, time, msg) {
        //console.log(msg);
      }
    }
  );*/
  console.log('Project loaded: ' + !proj.failed);

  //if(proj.failed) return false;

  //console.log('failed  :', failed);
  //console.log('proj.documents', proj.documents);

  let { board, schematic } = proj;

  const packages = {
    board: (board && [...board.elements].map(([name, e]) => e.package)) || [],
    schematic: (schematic && [...schematic.sheets].map(e => [...e.instances].map(([name, i]) => i.part.device.package).filter(p => p !== undefined)).flat()) || []
  };
  let parts = (schematic && schematic.parts) || [];
  let sheets = (schematic && schematic.sheets) || [];
  let libraries = (board && board.libraries) || [];
  let elements = (board && board.elements) || [];

  for(let [libName, lib] of libraries) {
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
  for(let [name, elem] of elements) {
    cmds.push(`MOVE ${elem.name} ${elem.pos};`);
    if(elem.rot) cmds.push(`ROTATE ${elem.rot} ${elem.name};`);
  }
  /*  for(let description of board.getAll('description')) {
  }*/

  if(updateMeasures(proj.board) || alignAll(board) || alignAll(schematic)) console.log('Saved:', await proj.saveTo('tmp', true));

  console.log('documents', proj.documents);

  console.log('saved:', await proj.saveTo('tmp', true));

  //for(let sheet of board.get('sheet'))
  //console.log('sheet', sheet, sheet.xpath());
  /*
  for(let instance of schematic.getAll(e => e.tagName == 'instance')) {
    const { part, gate } = instance;
  }
  let gates = [...schematic.getAll('gate')];
  let p = gates[0];
  while(p) {
    p = p.parentNode;
  }
*/
  let desc = proj.documents.map(doc => [doc.filename, doc.find('description')]);

  desc = desc.map(([file, e]) => [file, e && e.xpath().toCode('', { spacing: '', function: true })]);
  desc = new Map(desc);
  console.log('descriptions', [...Util.map(desc, ([k, v]) => [k, v])]);

  return proj;
}
(async () => {
  let args = process.argv.slice(2);
  if(args.length == 0) args.unshift('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3');
  for(let arg of args) {
    arg = arg.replace(/\.(brd|sch|lbr)$/i, '');
    try {
      let project = await testEagle(arg);
    } catch(err) {
      console.log('Err:', err.message, typeof err.stack == 'string' ? err.stack : [...err.stack].map(f => f + ''));
      process.exit(1);
    }
  }
  process.exit(0);
})();
