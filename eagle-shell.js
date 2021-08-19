import {
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
} from './lib/eagle.js';
import Util from './lib/util.js';
import * as deep from './lib/deep.js';
import path from './lib/path.js';
import { LineList, Point, Circle, Rect, Size, Line, TransformationList, Rotation, Translation, Scaling, Matrix, BBox } from './lib/geom.js';
import ConsoleSetup from './lib/consoleSetup.js';
import REPL from './xrepl.js';
import { BinaryTree, BucketStore, BucketMap, ComponentMap, CompositeMap, Deque, Enum, HashList, Multimap, Shash, SortedMap, HashMultimap, MultiBiMap, MultiKeyMap, DenseSpatialHash2D, SpatialHash2D, HashMap, SpatialH, SpatialHash, SpatialHashMap, BoxHash } from './lib/container.js';
import PortableFileSystem from './lib/filesystem.js';
import { Pointer } from './lib/pointer.js';
import { read as fromXML, write as toXML } from './lib/xml.js';
import inspect from './lib/objectInspect.js';
import { ReadFile, LoadHistory, ReadJSON, MapFile, ReadBJSON, WriteFile, WriteJSON, WriteBJSON, DirIterator, RecursiveDirIterator } from './io-helpers.js';

let cmdhist;

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
  /* prettier-ignore */ get head() {
    return this[this.length-1];
  },
  /* prettier-ignore */ get tail() {
    return this[this.length-1];
  }
});

function Terminate(exitCode) {
  console.log('Terminate', exitCode);

  Util.exit(exitCode);
}

/*function LoadHistory(filename) {
  let contents = fs.readFileSync(filename, 'utf-8');
  let data;

  const parse = () => {
    try {
      data = JSON.parse(contents);
    } catch(e) {}
    if(data) return data;
    try {
      data = contents.split(/\n/g);
    } catch(e) {}
    if(data) return data;
  };

  return (parse() ?? []).filter(entry => (entry + '').trim() != '').map(entry => entry.replace(/\\n/g, '\n'));
}

function ReadJSON(filename) {
  let data = std.loadFile(filename);

  if(data) console.log(`ReadJSON('${filename}') ${data.length} bytes read`);
  return data ? JSON.parse(data) : null;
}*/

async function importModule(moduleName, ...args) {
  //console.log('importModule', moduleName, args);
  let done = false;
  return await import(moduleName)
    .then(module => {
      //console.log('import', { module });
      done = true;
      Object.assign(globalThis, { [moduleName]: module });
      return module;
    })
    .catch(e => {
      console.error(moduleName + ':', e);
      done = true;
    });
  // while(!done) std.sleep(50);
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
  let changed;
  // console.log('AlignItem', { item });
  let offsetPos = new Point(0, 0);
  let geometry = item.geometry;
  if(item.tagName == 'element') {
    let pkg = item['package'];
    let transformation = item.transformation().filter(tr => tr.type != 'translate');
    let matrix = transformation.toMatrix();
    //console.log('AlignItem:', { transformation, matrix });
    offsetPos = new Point(pkg.pads[0]).transform(matrix);
    let inchPos = offsetPos.quot(2.54);
    // console.log('AlignItem:', { offsetPos, inchPos });
    let oldPos = new Point(item);
    inchPos = oldPos.quot(2.54);
    // console.log('AlignItem:', { oldPos, inchPos });
    let padPos = oldPos.sum(offsetPos);
    inchPos = padPos.quot(2.54);
    // console.log('AlignItem:', { padPos, inchPos });
    let newPos = padPos.round(2.54).diff(offsetPos).round(0.0001, 4);
    let diff = newPos.diff(oldPos);
    let before = item.parentNode.toXML();
    //console.log('geometry:', Object.entries(Object.getOwnPropertyDescriptors(geometry)).map(([name, { value }]) => [name, value && Object.getOwnPropertyDescriptors(value)]), geometry.x1);
    inchPos = newPos.quot(2.54);
    //console.log('AlignItem:', { newPos, diff, inchPos });
    geometry.add(diff);
    changed = !diff.isNull();
  }
  if(item.tagName == 'wire') {
    let oldCoord = geometry.clone();
    let inchCoord = oldCoord.quot(2.54);
    //console.log('AlignItem:', { oldCoord, inchCoord });
    let newCoord = oldCoord.clone().round(2.54);
    inchCoord = newCoord.quot(2.54);
    //console.log('AlignItem:', { newCoord, inchCoord });
    let diff = newCoord.diff(oldCoord);
    //console.log('AlignItem:', { diff });
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

function AlignAll(doc = globalThis.document) {
  if(!doc) return false;

  let items = doc.getAll(doc.type == 'brd' ? 'element' : 'instance');
  let changed = false;
  items = [...items];
  console.log('items:', items);
  for(let item of items) changed |= AlignItem(item);
  let signals_nets = doc.getAll(/(signals|nets)/);
  //console.log('signals_nets:', signals_nets);
  for(let net of signals_nets) for (let item of net.getAll('wire')) changed |= AlignItem(item);
  return !!changed;
}

function fixValue(element) {
  let value = element.value;
  let newValue;

  switch (element.name[0]) {
    case 'R': {
      newValue = value.replace(/^([0-9.]+)([mkM]?)(?:\xEF\xBF\xBD|\xC2\xA9|\x26\xC2*\xA9+|\u2126?[\x80-\xFF]+)([\x00-\x7F]*)/, '$1$2\u2126$3');
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

        //   console.log(`signal '${signal.name}' wire #${signal.wires.indexOf(wire)}:`, points);
        console.log(`signal '${signal.name}' wire #${signal.wires.indexOf(wire)}:`, { a, b });

        map.set(a.toString(), [signal.name, wire, b]);
        map.set(b.toString(), [signal.name, wire, a]);
      }
    }
    for(let element of doc.board.elements.children) {
      let pos = new Point(element.geometry);
      let transform = element.transformation().filter(t => t.type != 'translate');
      console.log(`element '${element.name}':`, pos, transform);
      let i = 0;
      let { contactrefs } = element;
      console.log(`contactrefs `, contactrefs);
      for(let pad of element.pads.list) {
        let { geometry } = pad;
        console.log(`pad '${element.name}.${pad.name}':`, geometry);
        let padPos = new Point(geometry);
        let cref = contactrefs[pad.name];
        map.set(padPos.toString(), [cref?.parentNode ?? null, element, pad /*, cref*/]);
      }
    }
  }
  globalThis.mapCoords = map;
  //  console.log('coordMap', { map });
  //  return map;
}

function GetPolygons(d = doc) {
  return [...d.getAll(e => e.tagName == 'polygon' && [1, 16].indexOf(+e.attributes.layer) != -1)];
}

function FindPolygons() {
  return (globalThis.polygons = docs.map(doc => [doc, GetPolygons(doc).map(e => e.path)]));
}

function RemovePolygons(p = polygons) {
  polygons.forEach(([doc, list]) => {
    list.forEach(path => deep.unset(doc.raw, path));

    doc.saveTo();
  });
}

function GetNames(doc, pred) {
  let list, names;
  if(typeof pred != 'function') pred = e => !!e.package;

  switch (doc.type) {
    case 'sch': {
      list = doc.sheets.map(sheet => [...sheet.instances.list]).flat();
      list = [...list].filter(pred);
      names = list.map(e => e.attributes.part);
      break;
    }
    case 'brd': {
      list = doc.elements.list;
      list = [...list].filter(pred);
      names = list.map(e => e.attributes.name);
      break;
    }
    default: {
      names = deep
        .select(doc.raw, e => e.attributes.name, deep.RETURN_VALUE_PATH)
        .filter(([v, p]) => ['symbol', 'device', 'package'].indexOf(v.tagName) != -1)
        .map(([v, p]) => v.attributes.name);
      break;
    }
  }
  return Util.unique(names);
}

let nameMaps = (() => {
  let assoc = new WeakMap();

  return Util.memoize(doc => {
    let map;
    switch (doc.type) {
      case 'sch': {
        map = new Map(doc.sheets.map(sheet => [...sheet.instances]).flat());
        break;
      }
      case 'brd': {
        map = new Map([...doc.elements]);
        break;
      }
    }
    return map;
  });
})();

function GetByName(doc, name) {
  let map = nameMaps(doc);

  //console.log("GetByName", map);
  return map.get(name);
}

function CorrelateSchematicAndBoard(schematic, board) {
  if(!schematic) schematic = project.schematic;
  if(!board) board = project.board;
  let documents = [schematic, board];
  let names = documents.map(d => GetNames(d));
  let allNames = Math.max(...names.map(n => n.length));
  let intersection = Util.intersect(...names);

  if(allNames.length > intersection.length) console.warn(`WARNING: Only ${intersection.length} names of ${allNames.length} correlate`);
  console.log(`intersection`, intersection);

  return /*new Map*/ intersection.map(name => [name, documents.map(doc => GetByName(doc, name))]);
}

function SaveLibraries() {
  const { schematic, board } = project;
  const layerMap = /*Object.values*/ [...schematic.layers, ...board.layers].filter(([n, e]) => e.active).reduce((acc, [n, e]) => ({ ...acc, [e.number]: e.raw }), {});
  const entities = ['symbols', 'packages', 'devicesets'];

  let layerIds = deep
    .select([schematic.raw, board.raw], e => e && e.layer)
    .map(e => +e.layer)
    .concat(Util.range(17, 49))
    .sort((a, b) => a - b);
  layerIds = Util.unique(layerIds);
  let layers = layerIds.map(id => layerMap[id]);

  //  console.log('layers', layers);
  console.log(
    'layerIds',
    console.config({ compact: 2 }),
    layerIds.map(id => [id, layerMap[id].attributes.name])
  );

  const libraryNames = Util.unique([...schematic.libraries, ...board.libraries].map(([n, e]) => n));
  console.log('libraryNames', libraryNames);

  const libraries = libraryNames.map(name => [name, schematic.libraries[name], board.libraries[name]]);
  for(let [name, ...libs] of libraries) {
    let obj = { symbols: [], packages: [], devicesets: [] };

    let xml = {
      tagName: 'library',
      children: [{ tagName: 'description', attributes: {}, children: [`${name}.lbr library`] }],
      attributes: { name }
    };

    for(let lib of libs) {
      if(lib) {
        for(let entity of entities) {
          if(lib[entity]) obj[entity] = [...obj[entity], ...lib[entity]];
        }
      }
    }
    for(let entity of entities) {
      obj[entity] = obj[entity].reduce((acc, [n, e]) => ({ ...acc, [n]: e.raw }), {});
    }

    for(let entity of entities) {
      obj[entity] = Object.values(obj[entity]);

      xml.children.push({ tagName: entity, children: obj[entity] });
    }

    // console.log('', console.config({ compact: 3, depth: 4 }), xml);

    xml = {
      tagName: '?xml',
      attributes: { version: '1.0', encoding: 'utf-8' },
      children: [
        { tagName: '!DOCTYPE eagle SYSTEM "eagle.dtd"', attributes: {}, children: [] },
        {
          tagName: 'eagle',
          attributes: { version: '6.4.1' },
          children: [
            {
              tagName: 'drawing',
              attributes: {},
              children: [
                {
                  tagName: 'settings',
                  attributes: {},
                  children: [
                    { tagName: 'setting', attributes: { alwaysvectorfont: 'no' } },
                    { tagName: 'setting', attributes: { verticaltext: 'up' } }
                  ]
                },
                {
                  tagName: 'grid',
                  attributes: {
                    distance: '0.3175',
                    unitdist: 'mm',
                    unit: 'mm',
                    style: 'lines',
                    multiple: '1',
                    display: 'yes',
                    altdistance: '0.025',
                    altunitdist: 'mm',
                    altunit: 'mm'
                  }
                },
                {
                  tagName: 'layers',
                  attributes: {},
                  children: layers
                },
                xml
              ]
            }
          ]
        }
      ]
    };

    //    console.log('xml', console.config({ compact: 3, depth: 9 }), xml);
    WriteFile(`${name}.lbr`, toXML(xml));
  }

  return xml;
  //console.log('libraries', libraries);
}

async function testEagle(filename) {
  console.log('testEagle: ', filename);
  let proj = new EagleProject(filename, fs);
  console.log('Project loaded: ', !proj.failed);
  console.log('Project: ', proj);
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
  console.log('proj.board', proj.board);
  if(proj.board) UpdateMeasures(proj.board);
  if(AlignAll(board) || AlignAll(schematic)) console.log('Saved:', await proj.saveTo('tmp', true));
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
  desc = desc.map(([file, e]) => [file, e && e.xpath()]).map(([file, xpath]) => [file, xpath && xpath.toCode('', { spacing: '', function: true })]);
  desc = new Map(desc);
  console.log('descriptions', [...Util.map(desc, ([k, v]) => [k, v])]);
  return proj;
}

async function main(...args) {
  await ConsoleSetup({
    inspectOptions: { depth: 3, compact: 3, maxArrayLength: 10, maxStringLength: 10 }
  });

  let fs;
  let debugLog;

  if(Util.getPlatform() == 'quickjs') {
    globalThis.std = await import('std');
    globalThis.os = await import('os');
    globalThis.fs = fs = await import('./lib/filesystem.js');
    // console.log('quickjs', { std, os, fs });
  } else {
    //globalThis.fs = fs=await import('fs');
    //  console.log('fs', globalThis.fs);
    const cb = filesystem => {
      globalThis.fs = fs = filesystem;
    };
    await PortableFileSystem(cb);
  }

  debugLog = fs.openSync('debug.log', 'a');

  const progName = Util.getArgv()[1];
  const base = path.basename(progName, path.extname(progName));
  const histfile = `.${base}-history`;

  let params = Util.getOpt(
    {
      debug: [false, null, 'x'],
      'output-dir': [true, null, 'd'],
      '@': 'input'
    },
    args
  );

  Object.assign(globalThis, {
    SaveLibraries,
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
    DereferenceError,
    GetNames,
    GetByName,
    CorrelateSchematicAndBoard,
    AlignItem,
    AlignAll,
    UpdateMeasures,
    ReadFile,
    LoadHistory,
    ReadJSON,
    MapFile,
    ReadBJSON,
    WriteFile,
    WriteJSON,
    WriteBJSON,
    DirIterator,
    RecursiveDirIterator
  });

  Object.assign(globalThis, {
    load(filename, project = globalThis.project) {
      globalThis.document = new EagleDocument(fs.readFileSync(filename, 'utf-8'), project, filename, null, fs);
    },
    newProject(filename) {
      if(!globalThis.project) globalThis.project = new EagleProject(null);

      project.lazyOpen(filename);
    }
  });
  // globalThis.docs = args.map(arg => (globalThis.doc = load(arg)));

  Object.assign(globalThis, {
    UpdateMeasures,
    AlignItem,
    AlignAll,
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
    Matrix,
    BBox,
    fs,
    Pointer,
    deep
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
    BoxHash,
    ReadJSON,
    GetPolygons,
    FindPolygons,
    RemovePolygons,
    quit(arg) {
      repl.cleanup();
      Util.exit(arg ?? 0);
    }
  });

  cmdhist = `.${base}-cmdhistory`;

  let repl = (globalThis.repl = new REPL(base));

  //console.log(`repl`, repl);
  //console.log(`debugLog`, Util.getMethods(debugLog, Infinity, 0));
  repl.history = LoadHistory(cmdhist);

  repl.printStatus(/*console.log*/ `LOAD (read ${repl.history.length} history entries)`);

  repl.debugLog = debugLog;
  repl.exit = Terminate;
  repl.importModule = importModule;
  repl.debug = (...args) => {
    let s = '';
    for(let arg of args) {
      if(s) s += ' ';
      if(typeof arg != 'strping' || arg.indexOf('\x1b') == -1) s += inspect(arg, { depth: Infinity, depth: 6, compact: false });
      else s += arg;
    }
    fs.writeSync(debugLog, fs.bufferFrom(s + '\n'));

    //    debugLog.puts(s + '\n');
    fs.flushSync(debugLog);
  };
  repl.show = value => {
    if(Util.isObject(value) && value instanceof EagleNode) {
      console.log(value.inspect());
    } else {
      console.log(value);
    }
  };

  // repl.historySet(JSON.parse(std.loadFile(histfile) || '[]'));

  repl.cleanup = () => {
    let hist = repl.history.filter((item, i, a) => a.lastIndexOf(item) == i);

    //    fs.writeFileSync(cmdhist, JSON.stringify(hist, null, 2));
    fs.writeFileSync(
      cmdhist,
      hist
        .filter(entry => (entry + '').trim() != '')
        .map(entry => entry.replace(/\n/g, '\\\\n') + '\n')
        .join('')
    );

    console.log(`EXIT (wrote ${hist.length} history entries)`);
    Terminate(0);
  };
  /*
  Util.atexit(() => {
    let hist = repl.history.filter((item, i, a) => a.lastIndexOf(item) == i);

    fs.writeFileSync(histfile, JSON.stringify(hist, null, 2));

    console.log(`EXIT (wrote ${hist.length} history entries)`);
  });
*/

  for(let file of params['@']) {
    repl.printStatus(`Loading '${file}'...`);

    newProject(file);
  }

  //globalThis.project = new EagleProject(params['@']);
  //console.log('globalThis.project', globalThis.project);

  await repl.run();
}

Util.callMain(main, true);
/*
try {
  main(...Util.getArgs().slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  Util.exit(1);
} finally {
  console.log('SUCCESS');
}
*/
