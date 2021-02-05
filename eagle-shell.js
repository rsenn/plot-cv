import { EagleSVGRenderer, SchematicRenderer, BoardRenderer, LibraryRenderer, EagleNodeList, useTrkl, RAD2DEG, DEG2RAD, VERTICAL, HORIZONTAL, HORIZONTAL_VERTICAL, DEBUG, log, setDebug, PinSizes, EscapeClassName, UnescapeClassName, LayerToClass, ElementToClass, ClampAngle, AlignmentAngle, MakeRotation, EagleAlignments, Alignment, SVGAlignments, AlignmentAttrs, RotateTransformation, LayerAttributes, InvertY, PolarToCartesian, CartesianToPolar, RenderArc, CalculateArcRadius, LinesToPath, MakeCoordTransformer, useAttributes, EagleDocument, EagleReference, EagleRef, makeEagleNode, EagleNode, Renderer, EagleProject, EagleElement, makeEagleElement, EagleElementProxy, EagleNodeMap, ImmutablePath, DereferenceError } from './lib/eagle.js';
import PortableFileSystem from './lib/filesystem.js';
import { toXML } from './lib/json.js';
import Util from './lib/util.js';
import deep from './lib/deep.js';
import path from './lib/path.js';
import { LineList, Point, Circle, Rect, Size, Line, TransformationList, Rotation, Translation, Scaling, Matrix } from './lib/geom.js';
import ConsoleSetup from './lib/consoleSetup.js';
import REPL from './repl.js';
import { BinaryTree, BucketStore, BucketMap, ComponentMap, CompositeMap, Deque, Enum, HashList, Multimap, Shash, SortedMap, HashMultimap, MultiBiMap, MultiKeyMap, DenseSpatialHash2D, SpatialHash2D, HashMap, SpatialH, SpatialHash, SpatialHashMap, BoxHash } from './lib/container.js';

let filesystem;

Util.define(Array.prototype, {
  findLastIndex(predicate) {
    for(let i = this.length - 1; i >= 0; --i) {
      const x = this[i];
      if(predicate(x, i, this)) {
        return i;
      }
    }
    return -1;
  },
  rotateRight(n) {
    this.unshift(...this.splice(n, this.length - n));
    return this;
  },
  rotateLeft(n) {
    this.push(...this.splice(0, n));
    return this;
  },
  at(index) {
    return this[Util.mod(index, this.length)];
  },
  get head() {
    return this[this.length-1];
  },
  get tail() {
    return this[this.length-1];
  }
});

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
  let changed;
  // console.log('alignItem', { item });
  let offsetPos = new Point(0, 0);
  let geometry = item.geometry;
  if(item.tagName == 'element') {
    let pkg = item['package'];
    let transformation = item.transformation().filter(tr => tr.type != 'translate');
    let matrix = transformation.toMatrix();
    //console.log('alignItem:', { transformation, matrix });
    offsetPos = new Point(pkg.pads[0]).transform(matrix);
    let inchPos = offsetPos.quot(2.54);
    // console.log('alignItem:', { offsetPos, inchPos });
    let oldPos = new Point(item);
    inchPos = oldPos.quot(2.54);
    // console.log('alignItem:', { oldPos, inchPos });
    let padPos = oldPos.sum(offsetPos);
    inchPos = padPos.quot(2.54);
    // console.log('alignItem:', { padPos, inchPos });
    let newPos = padPos.round(2.54).diff(offsetPos).round(0.0001, 4);
    let diff = newPos.diff(oldPos);
    let before = item.parentNode.toXML();
    //console.log('geometry:', Object.entries(Object.getOwnPropertyDescriptors(geometry)).map(([name, { value }]) => [name, value && Object.getOwnPropertyDescriptors(value)]), geometry.x1);
    inchPos = newPos.quot(2.54);
    //console.log('alignItem:', { newPos, diff, inchPos });
    geometry.add(diff);
    changed = !diff.isNull();
  }
  if(item.tagName == 'wire') {
    let oldCoord = geometry.clone();
    let inchCoord = oldCoord.quot(2.54);
    //console.log('alignItem:', { oldCoord, inchCoord });
    let newCoord = oldCoord.clone().round(2.54);
    inchCoord = newCoord.quot(2.54);
    //console.log('alignItem:', { newCoord, inchCoord });
    let diff = newCoord.diff(oldCoord);
    //console.log('alignItem:', { diff });
    changed = !diff.isNull();
    geometry.add(diff);
  }
  if(changed) {
    console.log(item);
    /*    console.log('after:', Util.abbreviate(item.parentNode.toXML()));
     console.log('align\n', item.xpath(), '\n newPos:', newPos, '\n diff:', diff, '\n attr:', item.raw.attributes);*/
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

function fixValue(element) {
  let value = element.value;
  let newValue;

  switch (element.name[0]) {
    case 'R': {
      newValue = value.replace(/^([0-9.]+)([mkM]?)(?:\xEF\xBF\xBD|\xC2\xA9|\x26\xC2*\xA9+|\u2126?[\x80-\xFF]+)([\x00-\x7F]*)/,
        '$1$2\u2126$3'
      );
      break;
    }
    case 'L': {
      newValue = value.replace(/^([0-9.]+)(?:[\x7F-\xFF]*\xB5|\xEF\xBF\xBD)(H.*)/, '$1\u00B5$2');
      break;
    }
    case 'C': {
      newValue = value.replace(/^([0-9.]+)(?:[\x7F-\xFF]*\xB5|\xEF\xBF\xBD)(F.*)/, '$1\u00B5$2');
      break;
    }
  }
  if(newValue && newValue != value) {
    console.log(`element ${element} value changed from '${value}' to '${newValue}'`);
    element.attributes['value'] = newValue;
  }
}

function fixValues(doc) {
  if(doc.type == 'brd') doc.elements.forEach(fixValue);
  else if(doc.type == 'sch') doc.parts.forEach(fixValue);
}

function coordMap(doc) {
  let map = new Multimap();

  if(doc.type == 'brd') {
    for(let [, signal] of doc.signals) {
      for(let wire of signal.wires) {
        let line = new Line(wire.geometry);
        let points = line.toPoints();
        let [a, b] = points.map(p => new Point(p));

        console.log(`signal '${signal.name}' wire #${signal.wires.indexOf(wire)}:`, points);
        console.log(`signal '${signal.name}' wire #${signal.wires.indexOf(wire)}:`, { a, b });

        map.set(a.toString(), [signal.name, wire, b]);
        map.set(b.toString(), [signal.name, wire, a]);
      }
    }
  }
  console.log('coordMap', { map });
  return map;
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
  await ConsoleSetup({ /*breakLength: 240, */ depth: 10 });
  await PortableFileSystem(fs => (filesystem = fs));

  const base = path.basename(Util.getArgv()[1], /\.[^.]*$/);
  const histfile = `.${base}-history`;

  Object.assign(globalThis, {
    EagleSVGRenderer,
    SchematicRenderer,
    BoardRenderer,
    LibraryRenderer,
    EagleNodeList,
    useTrkl,
    RAD2DEG,
    DEG2RAD,
    VERTICAL,
    HORIZONTAL,
    HORIZONTAL_VERTICAL,
    DEBUG,
    log,
    setDebug,
    PinSizes,
    EscapeClassName,
    UnescapeClassName,
    LayerToClass,
    ElementToClass,
    ClampAngle,
    AlignmentAngle,
    MakeRotation,
    EagleAlignments,
    Alignment,
    SVGAlignments,
    AlignmentAttrs,
    RotateTransformation,
    LayerAttributes,
    InvertY,
    PolarToCartesian,
    CartesianToPolar,
    RenderArc,
    CalculateArcRadius,
    LinesToPath,
    MakeCoordTransformer,
    useAttributes,
    EagleDocument,
    EagleReference,
    EagleRef,
    makeEagleNode,
    EagleNode,
    Renderer,
    EagleProject,
    EagleElement,
    makeEagleElement,
    EagleElementProxy,
    EagleNodeMap,
    ImmutablePath,
    DereferenceError
  });

  Object.assign(globalThis, {
    load(filename) {
      return (globalThis.document = new EagleDocument(std.loadFile(filename)));
    }
  });

  Object.assign(globalThis, {
    updateMeasures,
    alignItem,
    alignAll,
    fixValue,
    fixValues,
    coordMap,
    Util,
    LineList,
    Point,
    Circle,
    Rect,
    Size,
    Line,
    TransformationList,
    Rotation,
    Translation,
    Scaling,
    Matrix
  });

  Object.assign(globalThis, {
    BinaryTree,
    BucketStore,
    BucketMap,
    ComponentMap,
    CompositeMap,
    Deque,
    Enum,
    HashList,
    Multimap,
    Shash,
    SortedMap,
    HashMultimap,
    MultiBiMap,
    MultiKeyMap,
    DenseSpatialHash2D,
    SpatialHash2D,
    HashMap,
    SpatialH,
    SpatialHash,
    SpatialHashMap,
    BoxHash
  });
  console.log('REPL now');

  let repl = (globalThis.repl = new REPL(base));
  repl.exit = Util.exit;

  repl.history_set(JSON.parse(std.loadFile(histfile) || '[]'));

  Util.atexit(() => {
    let hist = repl.history_get().filter((item, i, a) => a.lastIndexOf(item) == i);

    filesystem.writeFile(histfile, JSON.stringify(hist, null, 2));

    console.log(`EXIT (wrote ${hist.length} history entries)`);
  });
  await repl.run();
  console.log('REPL done');
}

Util.callMain(main, true);
