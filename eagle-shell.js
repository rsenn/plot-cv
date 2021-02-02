import { EagleSVGRenderer, SchematicRenderer, BoardRenderer, LibraryRenderer, EagleNodeList, useTrkl, RAD2DEG, DEG2RAD, VERTICAL, HORIZONTAL, HORIZONTAL_VERTICAL, DEBUG, log, setDebug, PinSizes, EscapeClassName, UnescapeClassName, LayerToClass, ElementToClass, ClampAngle, AlignmentAngle, MakeRotation, EagleAlignments, Alignment, SVGAlignments, AlignmentAttrs, RotateTransformation, LayerAttributes, InvertY, PolarToCartesian, CartesianToPolar, RenderArc, CalculateArcRadius, LinesToPath, MakeCoordTransformer, useAttributes, EagleDocument, EagleReference, EagleRef, makeEagleNode, EagleNode, Renderer, EagleProject, EagleElement, makeEagleElement, EagleElementProxy, EagleNodeMap, ImmutablePath, DereferenceError } from './lib/eagle.js';
import PortableFileSystem from './lib/filesystem.js';
import { LineList, Rect } from './lib/geom.js';
import { toXML } from './lib/json.js';
import Util from './lib/util.js';
import deep from './lib/deep.js';
import { Graph } from './lib/fd-graph.js';
import ptr from './lib/json-ptr.js';
import LogJS from './lib/log.js';
 import ConsoleSetup from './lib/consoleSetup.js';
  import REPL from './repl.js';

let  filesystem;
 

function updateMeasures(board) {
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
    plain.append(...lines.map(line => ({
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
  console.debug('alignItem', item);
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
    console.log('align\n',
      item.xpath(),
      '\n newPos:',
      newPos,
      '\n diff:',
      diff,
      '\n attr:',
      item.raw.attributes
    );
  }
  return changed;
}

function alignAll(doc = globalThis.document) {
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
  console.log('testEagle: ', filename);
  let proj = new EagleProject(filename, filesystem);
  console.log('Project loaded: ', !proj.failed);
  console.log('Project: ', proj);
  console.log('proj.documents', proj.documents);
  let { board, schematic } = proj;
  const packages = {
    board: (board && board.elements && [...board.elements].map(([name, e]) => e.package)) || [],
    schematic: (schematic &&
        schematic.sheets &&
        [...schematic.sheets]
          .map(e =>
            [...e.instances].map(([name, i]) => i.part.device.package).filter(p => p !== undefined)
          )
          .flat()) ||
      []
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
  console.log('proj.board', proj.board);
  if(proj.board) updateMeasures(proj.board);
  if(alignAll(board) || alignAll(schematic)) console.log('Saved:', await proj.saveTo('tmp', true));
  console.log('documents', proj.documents);
  console.log('saved:', await proj.saveTo('tmp', true));
  for(let doc of proj.documents) {
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
      console.log('indexes', pkg.name, paths);
      paths.forEach(i => deep.unset(doc.raw, [...i]));
      changed = changed || indexes.length > 0;
    }
    if(changed) {
      doc.saveTo(doc.filename);
      console.log('Saved:', doc.filename);
    }
  }
  let desc = proj.documents.map(doc => [doc.filename, doc.find('description')]);
  console.log('desc', desc);
  desc = desc
    .map(([file, e]) => [file, e && e.xpath()])
    .map(([file, xpath]) => [file, xpath && xpath.toCode('', { spacing: '', function: true })]);
  desc = new Map(desc);
  console.log('descriptions', [...Util.map(desc, ([k, v]) => [k, v])]);
  return proj;
}



async function main(...args) {
    await ConsoleSetup({ breakLength: 120, depth: 10 });
   await PortableFileSystem(fs => (filesystem = fs));
   Object.assign(globalThis, { EagleSVGRenderer, SchematicRenderer, BoardRenderer, LibraryRenderer, EagleNodeList, useTrkl, RAD2DEG, DEG2RAD, VERTICAL, HORIZONTAL, HORIZONTAL_VERTICAL, DEBUG, log, setDebug, PinSizes, EscapeClassName, UnescapeClassName, LayerToClass, ElementToClass, ClampAngle, AlignmentAngle, MakeRotation, EagleAlignments, Alignment, SVGAlignments, AlignmentAttrs, RotateTransformation, LayerAttributes, InvertY, PolarToCartesian, CartesianToPolar, RenderArc, CalculateArcRadius, LinesToPath, MakeCoordTransformer, useAttributes, EagleDocument, EagleReference, EagleRef, makeEagleNode, EagleNode, Renderer, EagleProject, EagleElement, makeEagleElement, EagleElementProxy, EagleNodeMap, ImmutablePath, DereferenceError });

Object.assign(globalThis, {
  load(filename) {
return globalThis.document =    new EagleDocument(std.loadFile(filename));
  },
  updateMeasures, alignItem,alignAll
});

 REPL(globalThis);
}

Util.callMain(main, true);
