import deep from './lib/deep.js';
import { EagleDocument, EagleProject } from './lib/eagle.js';
import { Graph } from './lib/fd-graph.js';
import { LineList, Rect } from './lib/geom.js';
import ptr from './lib/json-ptr.js';
import { toXML } from './lib/json.js';
let filesystem = fs;

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

function UpdateMeasures(board) {
  if(!board) return false;
  let bounds = board.getBounds();
  let measures = board.getMeasures();

  if(measures) {
    console.log('got measures:', measures);
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

function AlignItem(item) {
  console.debug('AlignItem', item);
  let geometry = item.geometry;
  let oldPos = geometry.clone();

  let newPos = geometry.clone().round(1.27, 2);

  let diff = newPos.diff(oldPos).round(0.0001, 5);

  let before = item.parentNode.toXML();

  //console.log('geometry:', Object.entries(Object.getOwnPropertyDescriptors(geometry)).map(([name, { value }]) => [name, value && Object.getOwnPropertyDescriptors(value)]), geometry.x1);

  geometry.add(diff);

  //geometry.y2 = 0;

  let changed = !diff.isNull();

  if(changed) {
    console.log('before:', Util.abbreviate(before));
    console.log('after:', Util.abbreviate(item.parentNode.toXML()));
    //console.log('geometry:', geometry);
    // console.log('align\n', item.xpath(), '\n newPos:', newPos, '\n diff:', diff, '\n attr:', item.raw.attributes );
  }
  return changed;
}

function AlignAll(doc) {
  if(!doc) return false;

  let items = doc.getAll(doc.type == 'brd' ? 'element' : 'instance');
  let changed = false;
  for(let item of items) changed |= AlignItem(item);
  let signals_nets = doc.getAll(/(signals|nets)/);
  //console.log('signals_nets:', signals_nets);
  for(let net of signals_nets) for (let item of net.getAll('wire')) changed |= AlignItem(item);
  return !!changed;
}

async function testEagle(filename) {
  console.log('testEagle: ', filesystem);
  let proj = new EagleProject(filename, filesystem);

  /*
  LogJS.addAppender(class extends LogJS.BaseAppender {
      log(type, time, msg) {
        //console.log(msg);
      }
    }
  );*/
  console.log('Project loaded: ', !proj.failed);
  console.log('Project: ', proj);

  //if(proj.failed) return false;

  //console.log('failed  :', failed);
  console.log('proj.documents', proj.documents);

  let { board, schematic } = proj;

  const packages = {
    board: (board && board.elements && [...board.elements].map(([name, e]) => e.package)) || [],
    schematic: (schematic && schematic.sheets && [...schematic.sheets].map(e => [...e.instances].map(([name, i]) => i.part.device.package).filter(p => p !== undefined)).flat()) || []
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
  console.log('proj.board', proj.board);
  if(proj.board) UpdateMeasures(proj.board);

  if(AlignAll(board) || AlignAll(schematic)) console.log('Saved:', await proj.saveTo('tmp', true));

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
  let documents = [board, schematic].filter(d => d);
  console.log('documents', documents);

  for(let doc of documents) {
    let changed = false;
    console.log('eagle:', Util.className(doc.find('eagle')));

    for(let pkg of doc.find('eagle').getAll('package')) {
      let indexes = [...pkg.children].map((child, i, a) =>
        a
          .slice(i + 1)
          .map((child2, i2) => [i2 + i + 1, Util.equals(child.raw, child2.raw)])
          .filter(([index, equal]) => equal)
          .map(([index]) => index)
      );

      indexes = indexes.flat().reverse();
      let paths = indexes.map(i => pkg.path.down('children', i));

      //console.log("pkg.children", pkg.children.toXML());
      console.log('indexes', pkg.name, paths);

      //console.log('remove', paths.map(i => deep.get(doc.raw, [...i])));

      paths.forEach(i => deep.unset(doc.raw, [...i]));
      changed = changed || indexes.length > 0;

      //console.log("indexes:", new Map(indexes.map((j,i) => [i, j]).filter(([i,j]) => j.length).map(([i,j]) => [i,pkg.children[j]])));
    }
    if(changed) {
      doc.saveTo(doc.filename);
      console.log('Saved:', doc.filename);
    }
  }

  let desc = documents.map(doc => [doc.filename, doc.find('description')]);
  console.log('desc', desc);

  desc = desc.map(([file, e]) => [file, e && e.xpath()]).map(([file, xpath]) => [file, xpath && xpath.toCode('', { spacing: '', function: true })]);
  desc = new Map(desc);
  console.log('descriptions', [...Util.map(desc, ([k, v]) => [k, v])]);

  return proj;
}

async function main(...args) {
  if(args.length == 0) args.unshift('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3');
  for(let arg of args) {
    //arg = arg.replace(/\.(brd|sch|lbr)$/i, '');
    try {
      let project = await testEagle(arg);
    } catch(err) {
      console.log('Err:', err.message, typeof err.stack == 'string' ? err.stack : [...err.stack].map(f => f + ''));
      process.exit(1);
    }
  }
}

main(...scriptArgs.slice(1));