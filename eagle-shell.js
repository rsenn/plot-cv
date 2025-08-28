#!/usr/bin/env qjsm
import child_process from 'child_process';
import * as fs from 'fs';
import { className, define, entries, getOpt, glob, GLOB_BRACE, intersect, difference, isObject, lazyProperties, memoize, range, unique, weakDefine } from 'util';
import { Table } from './cli-helpers.js';
import { DirIterator, ReadDirRecursive, RecursiveDirIterator } from './dir-helpers.js';
import { GetElements, GetInstances, GetParts, GetPositions, num2color, scientific } from './eagle-commands.js';
import { CopyToClipboard, FdReader, Filter, FilterImages, IfDebug, LoadHistory, LogCall, LogIfDebug, ReadBJSON, ReadFd, ReadFile, ReadJSON, ReadXML, SortFiles, StatFiles, WriteBJSON, WriteFile, WriteJSON, WriteXML, } from './io-helpers.js';
import { BinaryTree, BoxHash, BucketMap, BucketStore, ComponentMap, CompositeMap, DenseSpatialHash2D, Deque, Enum, HashList, HashMap, HashMultimap, MultiBiMap, MultiKeyMap, Multimap, Shash, SortedMap, SpatialH, SpatialHash, SpatialHash2D, SpatialHashMap, } from './lib/container.js';
import * as deep from './lib/deep.js';
import { forwardRef, Fragment, h, React, ReactComponent, render, toChildArray } from './lib/dom/preactComponent.js';
import { EagleSVGRenderer, SchematicRenderer, BoardRenderer, LibraryRenderer, RAD2DEG, DEG2RAD, VERTICAL, HORIZONTAL, HORIZONTAL_VERTICAL, DEBUG, setDebug, EscapeClassName, UnescapeClassName, LayerToClass, ElementToClass, ClampAngle, AlignmentAngle, MakeRotation, EagleAlignments, Alignment, SVGAlignments, AlignmentAttrs, RotateTransformation, LayerAttributes, InvertY, PolarToCartesian, CartesianToPolar, RenderArc, CalculateArcRadius, LinesToPath, MakeCoordTransformer, useAttributes, EagleRef, EagleNode, makeEagleElement, DereferenceError, EagleNodeList, useTrkl, EagleDocument, EagleReference, makeEagleNode, Renderer, EagleProject, EagleElement, EagleElementProxy, EagleNodeMap, ImmutablePath, } from './lib/eagle.js';
import { ElementNameToComponent, ElementToComponent, PinSizes } from './lib/eagle/components.js';
import CircuitJS from './lib/eda/circuitjs.js';
import { GetColorBands, GetFactor, GetMultipliers } from './lib/eda/colorCoding.js';
import { GetExponent, GetMantissa, NumberToValue, ValueToNumber } from './lib/eda/values.js';
import { EventEmitter, eventify, EventTarget } from './lib/events.js';
import { BBox, Circle, Line, LineList, Matrix, Point, Rect, Rotation, Scaling, Size, TransformationList, Translation } from './lib/geom.js';
import { Edge, Graph, Node } from './lib/fd-graph.js';
import * as path from './lib/path.js';
import { Pointer } from './lib/pointer.js';
import renderToString from './lib/preact-render-to-string.js';
import { ImmutableXPath, MutableXPath as XPath, parseXPath } from './lib/xml/xpath.js';
import { ExecTool, Spawn } from './os-helpers.js';
import { EagleToGerber, GerberToGcode } from './pcb-conversion.js';
import { Console } from 'console';
import inspect from 'inspect';
import { Predicate } from 'predicate';
import { REPL } from 'repl';
import { read as fromXML, write as writeXML } from 'xml';
import extendArray from 'extendArray';
import * as glfw from 'glfw';
import * as nanovg from 'nanovg';
import { DrawCircle, DrawLine } from './draw-utils.js';

let cmdhist;

extendArray();

function Window(width = 1280, height = 900) {
  for(let [prop, value] of [
    [glfw.CONTEXT_VERSION_MAJOR, 3],
    [glfw.CONTEXT_VERSION_MINOR, 2],
    [glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE],
    [glfw.OPENGL_FORWARD_COMPAT, true],
    [glfw.RESIZABLE, true],
    [glfw.SAMPLES, 4],
  ])
    glfw.Window.hint(prop, value);

  let window = (glfw.context.current = new glfw.Window(width, height, 'eagle-shell'));

  nanovg.CreateGL3(nanovg.STENCIL_STROKES | nanovg.ANTIALIAS | nanovg.DEBUG);
  nanovg.CreateFont('Century Gothic', '/home/roman/.fonts/gothic.ttf');

  define(window, {
    clear(color = nanovg.RGB(0, 0, 0)) {
      const { size } = this;
      nanovg.Save();
      nanovg.BeginPath();
      nanovg.Rect(0, 0, ...size);
      nanovg.FillColor(color);
      nanovg.Fill();
      nanovg.Restore();
    },
    begin() {
      const { width, height } = this.size;
      nanovg.BeginFrame(width, height, 1);
    },
    end() {
      nanovg.EndFrame();
      window.swapBuffers();
      glfw.poll();
    },
    handleMouseButton(button, action, mods) {
      repl.printStatus('handleMouseButton', { button, action, mods });
    },
    handleCursorPos(x, y) {
      repl.printStatus('handleCursorPos', { x, y });
    },
    handleCursorEnter(cur) {
      repl.printStatus('handleCursorEnter', { cur });
    },
  });

  window.handleCharMods = (char, mods) => {
    console.log(`handleCharMods`, { char, mods });
  };
  return window;
}

const GetGlobalFunctions = (() => {
  const a = Object.getOwnPropertyNames(globalThis);
  return () => {
    const b = Object.getOwnPropertyNames(globalThis);

    return difference(a, b)[1];
  };
})();

function toXML(obj) {
  deep.forEach(obj, a => Array.isArray(a.children) && a.children.length == 0 && delete a.children);
  return writeXML(obj);
}

function renderToXML(component) {
  return fromXML(renderToString(component));
}

function GetFiletime(file, field = 'mtime') {
  if(Array.isArray(file)) return file.map(file => [file, GetFiletime(file, field)]);
  let ms = fs.statSync(file)?.[field];
  return new Date(ms);
}

function FindProjects(dirPtn = '../*/eagle', sortDirection = -1, sortField = 'mtime') {
  let files = glob(dirPtn + '/*.{sch,brd}', GLOB_BRACE);
  let entries = SortFiles(files, sortDirection, sortField);

  let names = unique(files.map(fn => fn.replace(/\.(sch|brd)$/i, '')));

  const minIndex = name =>
    Math.min(
      entries.findIndex(file => file == name + '.sch'),
      entries.findIndex(file => file == name + '.brd'),
    );
  const hasBoth = name => minIndex(name) >= 0;

  return names
    .map(name => [name, minIndex(name)])
    .filter(([name, index]) => index >= 0)
    .sort((a, b) => b[1] - a[1])
    .map(([name, index]) => name);
}

function SortFiles(files, direction = 1, statField = 'mtime') {
  if(typeof direction == 'boolean') direction = direction ? 1 : -1;
  return GetFiletime(files, statField)
    .sort((a, b) => direction * (a[1] - b[1]))
    .map(([file]) => file);
}

function pick(it, n = 1) {
  let ret = new Array();

  if(typeof n != 'function') {
    let num = n;
    n = i => i < num;
  }
  let i = 0;
  for(i = 0; ; i++) {
    let { done, value } = it.next();
    if(done) break;
    if(n(i)) {
      ret.push(value);
    } else break;
  }
  return ret;
}

function append(tag, attrs, children, parent, element) {
  console.log('append', {
    tag,
    attrs,
    children,
    parent,
    element,
  });

  let obj;
  obj = { tagName: tag, attributes: attrs, children };

  if(parent) parent.children.push(obj);

  return obj;
}

function render(doc, filename) {
  if(doc instanceof EagleProject) {
    render(doc.schematic);
    render(doc.board);
    return;
  }
  let renderer = new Renderer(doc, ReactComponent.append);
  let str;
  let svg = renderer.render(doc);
  try {
    str = renderToString(svg);
  } catch(e) {
    console.log('ERROR:', e);
  }
  console.log('render', { str });

  let xml = fromXML(str);

  filename ??= path.basename(doc.filename, '.' + doc.type) + '-' + { sch: 'schematic', brd: 'board', lbr: 'library' }[doc.type] + '.svg';

  if(filename) {
    let ret;
    ret = WriteFile(filename, (str = toXML(xml)));
    console.log(`Saving to '${filename}'...`, ret);
  }
  return str;
}

function CollectParts(doc = project.schematic) {
  return [...doc.parts.raw]
    .map(e => e.attributes)
    .filter(attr => !(attr.value === undefined && attr.device === '') || /^IC/.test(attr.name))
    .map(({ name, deviceset, device, value }) => ({
      name,
      deviceset,
      device,
      value: value ?? '-',
    }));
}

function CollectElements(doc = project.board) {
  return doc.elements.raw
    .map(e => e.attributes)
    .map(({ name, library, package: pkg, value }) => ({
      name,
      library,
      ['package']: pkg,
      value,
    }));
}

function CollectPartsElements(proj = project) {
  return proj.board.elements.raw
    .map(e => [e, proj.schematic.parts[e.name]?.raw])
    .map(a => a.map(e => e?.attributes ?? {}))
    .map(([{ x, y, ...element }, part]) => weakDefine(element, part));
}

function ListParts(doc = project.schematic) {
  let parts = CollectParts(doc);
  let valueLen = Math.max(...parts.map(p => p.value.length));

  return parts.map(({ name, deviceset, device, value }) => value.padStart(valueLen) + ' ' + device);
}

function ShowParts(doc = project.schematic) {
  return Table(
    CollectParts(doc).map(({ name, deviceset, device, value }) => [name, deviceset, device, value ?? '-']),
    ['name', 'deviceset', 'device', 'value'],
  );
}

function EaglePrint(file, output) {
  output ??= ModifyPath(file, (dir, base, ext) => [dir, base, ext + '.pdf']);
  let argv = ['/opt/eagle-7.2.0/bin/eagle', '-N-', '-C', `PRINT landscape 0.8 -1 -0 -caption FILE '${output}' sheets all paper a4; QUIT`, file];

  return child_process.spawn(argv[0], argv, {
    block: false,
  });
}

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      maxArrayLength: 100,
      colors: true,
      depth: 10,
      compact: 0,
      customInspect: true,
    },
  });

  let debugLog = fs.openSync('debug.log', 'a');

  const base = path.basename(scriptArgs[0], path.extname(scriptArgs[0]));
  const histfile = `.${base}-history`;

  let params = getOpt(
    {
      debug: [false, null, 'x'],
      'output-dir': [true, null, 'd'],
      '@': 'input',
    },
    args,
  );

  Object.assign(globalThis, { Console, GetGlobalFunctions, className });

  Object.assign(globalThis, {
    child_process,
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
    LoadHistory,
    IfDebug,
    LogIfDebug,
    ReadFd,
    ReadFile,
    LoadHistory,
    ReadJSON,
    ReadXML,
    WriteFile,
    WriteJSON,
    WriteXML,
    ReadBJSON,
    WriteBJSON,
    DirIterator,
    RecursiveDirIterator,
    ReadDirRecursive,
    Filter,
    FilterImages,
    SortFiles,
    StatFiles,
    FdReader,
    CopyToClipboard,
    LogCall,
    Spawn,
    CopyToClipboard,
    CircuitJS,
    PutRowsColumns,
    GetLibrary,
    ElementName,
    GetUsedPackages,
    GetPackagePitch,
    Package2Circuit,
    GetRotation,
    Eagle2Circuit,
    Eagle2CircuitJS,
    ModifyPath,
    AppendToFilename,
    SetSVGBackground,
    SVGFileSetBackground,
    SVGResave,
    FileFunction,
    toNumber(n) {
      return isNaN(+n) ? n : +n;
    },
    path,
    EventEmitter,
    EventTarget,
    eventify,
    Graph,
    Edge,
    Node,
    xml,
    XPath,
    ImmutableXPath,
    parseXPath,
    Predicate,
    render,
    pick,
    fromXML,
    toXML,
    writeXML,
  });

  Object.assign(globalThis, {
    GetExponent,
    GetMantissa,
    ValueToNumber,
    NumberToValue,
    GetMultipliers,
    GetFactor,
    GetColorBands,
    UpdateMeasures,
    AlignItem,
    AlignAll,
    scientific,
    num2color,
    GetParts,
    GetInstances,
    GetPositions,
    GetElements,
    GetSheets,
  });

  Object.assign(globalThis, {
    define,
    isObject,
    memoize,
    unique,
    FindProjects,
    Table,
    CollectParts,
    ListParts,
    ShowParts,
    CollectElements,
    CollectPartsElements,
    EaglePrint,
    setDebug,
  });

  Object.assign(globalThis, {
    h,
    forwardRef,
    Fragment,
    React,
    ReactComponent,
    toChildArray,
    GetFiletime,
    Terminate,
    Contactref2Circuit,
    Signal2Circuit,
    Element2Circuit,
    SortFiles,
    BoardFromSchematic,
    InitBoard,
    MakeGraph,
    DrawGraph,
    Window,
    DrawCircle,
    DrawLine,
  });

  Object.assign(globalThis, {
    wire(...args) {
      let l = new Line(...args.splice(0, 4)).roundTo(0.00254);

      let [layer, width, curve] = args;
      let obj = { ...l.toObject() };

      if(typeof layer != 'undefined') obj.layer = layer;
      if(typeof width != 'undefined') obj.width = width;

      if(typeof curve != 'undefined') obj.curve = curve;

      return { tagName: 'wire', attributes: obj };
    },
    coil({ rect, n, diameter, width }) {
      if(rect !== undefined) rect = new Rect(rect);
      else {
        rect = new Rect(0, 0, diameter + 2 * width, width * n);
      }

      rect = rect.align(new Point(0, 0));

      let a = [];
      let step = rect.height / (n - 1);
      let slope = step; /*/ 2*/

      for(let i = 0; i < n; i++) {
        let y = i * step + rect.y1;

        a.push(wire(rect.x1, y - slope / 2, rect.x2, y + slope / 2, 21, width / 2));
      }

      return a;
    },
  });

  Object.assign(globalThis, {
    load(filename, project = globalThis.project) {
      globalThis.document = new EagleDocument(ReadFile(filename, 'utf-8'), project, filename, null, fs);
    },
    newProject(filename) {
      if(globalThis.project) {
        globalThis.project.closeAll();
        delete globalThis.project;
        delete globalThis.sch;
        delete globalThis.brd;
      }

      globalThis.project = new EagleProject(filename);

      //project.lazyOpen(filename);

      lazyProperties(globalThis, {
        sch: () => project.schematic,
        brd: () => project.board,
      });
    },
    nextProject() {
      if(globalThis.projectNames.length > globalThis.projectIndex + 1) {
        globalThis.newProject(globalThis.projectNames[++globalThis.projectIndex]);
        return globalThis.project;
      }
    },
  });

  lazyProperties(globalThis, {
    projectNames() {
      this.projectIndex = -1;
      return FindProjects(undefined, 1);
    },
  });
  // globalThis.docs = args.map(arg => (globalThis.doc = load(arg)));

  Object.assign(globalThis, {
    UpdateMeasures,
    AlignItem,
    AlignAll,
    fixValue,
    fixValues,
    coordMap,
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
    deep,
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
    GetPolygons,
    FindPolygons,
    RemovePolygons,
    quit(arg) {
      repl.cleanup();
      process.exit(arg ?? 0);
    },
  });

  Object.assign(globalThis, {
    ElementNameToComponent,
    ElementToComponent,
    EagleToGerber,
    GerberToGcode,
    ExecTool,
  });
  Object.assign(globalThis, {
    renderToString(arg) {
      //  return renderToString(arg);
      return writeXML(fromXML(renderToString(arg)));
    },
    renderToXML,
  });

  cmdhist = `.${base}-cmdhistory`;

  let name = (process.env['NAME'] ?? base)
    .replace(/.*\//, '')
    .replace(/-/g, ' ')
    .replace(/\.[^\/.]*$/, '');
  let [prefix, suffix] = name.split(' ');

  let repl = (globalThis.repl = new REPL(`\x1b[38;5;165m${prefix} \x1b[38;5;39m${suffix}\x1b[0m`, false));

  repl.history = LoadHistory(cmdhist);
  repl.loadSaveOptions();

  let log = console.log;

  console.log = (...args) => repl.printStatus(() => log(...args));

  repl.debugLog = debugLog;
  repl.exit = () => {
    repl.cleanup();
    Terminate();
  };
  repl.debug = (...args) => {
    let s = '';
    for(let arg of args) {
      if(s) s += ' ';
      if(typeof arg != 'string' || arg.indexOf('\x1b') == -1)
        s += inspect(arg, {
          depth: Infinity,
          depth: 6,
          compact: false,
        });
      else s += arg;
    }
    fs.writeSync(debugLog, fs.bufferFrom(s + '\n'));

    //    debugLog.puts(s + '\n');
    fs.flushSync(debugLog);
  };
  repl.show = value => {
    if(isObject(value)) {
      let insp = value.inspect ?? value[Symbol.inspect];
      if(typeof insp == 'function') return insp.call(value, 0, repl.inspectOptions);
    }
    return inspect(value, { customInspect: false, /*protoChain: true,*/ getters: true, ...repl.inspectOptions });
  };
  // repl.historySet(JSON.parse(std.loadFile(histfile) || '[]'));

  repl.addCleanupHandler(() => {
    let hist = repl.history.filter((item, i, a) => a.lastIndexOf(item) == i);

    //    WriteFile(cmdhist, JSON.stringify(hist, null, 2));
    WriteFile(
      cmdhist,
      hist
        .filter(entry => (entry + '').trim() != '')
        .map(entry => entry.replace(/\n/g, '\\\\n') + '\n')
        .join(''),
    );

    console.log(`EXIT (wrote ${hist.length} history entries)`);
    Terminate(0);
  });

  repl.run();

  for(let file of params['@']) {
    repl.printStatus(`Loading '${file}'...`);
    newProject(file);
  }
}

function Terminate(exitCode) {
  console.log('Terminate', exitCode);

  std.exit(exitCode);
}

function xml(strings, expressions) {
  let [tag] = strings;
  return e => e.tagName == tag;
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
        attributes: {
          ...line.toObject(),
          layer: 47,
          width: 0,
        },
      })),
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
    /*    console.log('after:', abbreviate(item.parentNode.toXML()));
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
  return unique(names);
}

let nameMaps = (() => {
  let assoc = new WeakMap();

  return memoize(doc => {
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
  let intersection = intersect(...names);

  if(allNames.length > intersection.length) console.warn(`WARNING: Only ${intersection.length} names of ${allNames.length} correlate`);
  console.log(`intersection`, intersection);

  return /*new Map*/ intersection.map(name => [name, documents.map(doc => GetByName(doc, name))]);
}

function GetSheets(doc_or_proj) {
  if(!(doc_or_proj instanceof EagleDocument)) doc_or_proj = doc_or_proj.schematic;

  return [...doc_or_proj.schematic.sheets];
}

function SaveLibraries() {
  const { schematic, board } = project;
  const layerMap = /*Object.values*/ [...schematic.layers, ...board.layers].filter(([n, e]) => e.active).reduce((acc, [n, e]) => ({ ...acc, [e.number]: e.raw }), {});
  const entities = ['symbols', 'packages', 'devicesets'];

  let layerIds = deep
    .select([schematic.raw, board.raw], e => e && e.layer)
    .map(e => +e.layer)
    .concat(range(17, 49))
    .sort((a, b) => a - b);
  layerIds = unique(layerIds);
  let layers = layerIds.map(id => layerMap[id]);

  //  console.log('layers', layers);
  console.log(
    'layerIds',
    console.config({ compact: 2 }),
    layerIds.map(id => [id, layerMap[id].attributes.name]),
  );

  const libraryNames = unique([...schematic.libraries, ...board.libraries].map(([n, e]) => n));
  console.log('libraryNames', libraryNames);

  const libraries = libraryNames.map(name => [name, schematic.getLibrary(name), board.getLibrary(name)]);
  for(let [name, ...libs] of libraries) {
    let obj = { symbols: [], packages: [], devicesets: [] };

    let xml = {
      tagName: 'library',
      children: [
        {
          tagName: 'description',
          attributes: {},
          children: [`${name}.lbr library`],
        },
      ],
      attributes: { name },
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

      xml.children.push({
        tagName: entity,
        children: obj[entity],
      });
    }

    // console.log('', console.config({ compact: 3, depth: 4 }), xml);

    xml = {
      tagName: '?xml',
      attributes: { version: '1.0', encoding: 'utf-8' },
      children: [
        {
          tagName: '!DOCTYPE eagle SYSTEM "eagle.dtd"',
          attributes: {},
          children: [],
        },
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
                    {
                      tagName: 'setting',
                      attributes: { alwaysvectorfont: 'no' },
                    },
                    {
                      tagName: 'setting',
                      attributes: { verticaltext: 'up' },
                    },
                  ],
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
                    altunit: 'mm',
                  },
                },
                {
                  tagName: 'layers',
                  attributes: {},
                  children: layers,
                },
                xml,
              ],
            },
          ],
        },
      ],
    };

    //    console.log('xml', console.config({ compact: 3, depth: 9 }), xml);
    WriteFile(`${name}.lbr`, toXML(xml));
  }

  return xml;
  //console.log('libraries', libraries);
}

function BoardFromSchematic(doc = project.schematic, f = 1, r = 0.5) {
  let a = [];

  for(let e of [...doc.sheets[0].instances.list]) {
    const { part, x, y } = e;
    let p = new Point(x, y).mul(f * 0.03937).round(r);

    a.push(`MOVE '${part.name}' (${[...p].join(' ')});`);
  }

  return a.join('\n');
}

function InitBoard(doc = project.board) {
  let commands = [...doc.elements].map(([name, e]) => {
    const p = new Point(e).div(25.4).round(0.005);

    let s = `MOVE '${name}' (${[...p].join(' ')});`;

    const { rot } = e;

    if(rot !== undefined) s += ` ROTATE =${rot} '${name}';`;

    return s;
  });

  return commands.join('\n');
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
    schematic: (schematic && schematic.sheets && [...schematic.sheets].map(e => [...e.instances].map(([name, i]) => i.part.device.package).filter(p => p !== undefined)).flat()) || [],
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
    console.log('eagle:', className(doc.find('eagle')));
    for(let pkg of doc.find('eagle').getAll('package')) {
      let indexes = [...pkg.children].map((child, i, a) =>
        a
          .slice(i + 1)
          .map((child2, i2) => [i2 + i + 1, child.raw === child2.raw])
          .filter(([index, equal]) => equal)
          .map(([index]) => index),
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
  console.log('descriptions', [...map(desc, ([k, v]) => [k, v])]);
  return proj;
}

function ModifyPath(p, fn = (dir, base, ext) => [dir, base, ext]) {
  let [dir, base, ext] = (() => {
    let { dir, base, ext } = path.toObject(p);
    return fn(dir, base, ext);
  })();

  return path.fromObject({ dir, base, ext });
}

function AppendToFilename(p, str = '') {
  return ModifyPath(p, (dir, base, ext) => [dir, base + str, ext]);
}

function SetSVGBackground(xml, color = '#ffffff') {
  let svgNode;
  let bgRect = deep.find(xml, (e, n) => e.tagName == 'rect' && e.attributes.id == 'background-rect', deep.RETURN_PATH);

  if(bgRect) throw new Error('background-rect alreay set');

  if((svgNode = deep.find(xml, (e, n) => e.tagName == 'svg', deep.RETURN_VALUE))) {
    let defs = deep.find(xml, (e, n) => e.tagName == 'defs', deep.RETURN_PATH);
    let gpath = deep.find(xml, (e, n) => e.tagName == 'g', deep.RETURN_PATH);
    let children = deep.get(xml, defs.slice(0, -1));

    //console.log('SetSVGBackground',{gpath});

    let { viewBox } = svgNode.attributes;
    let rect = Rect.fromString(viewBox);
    let pos = defs[defs.length - 1] ?? 0;
    //console.log('SetSVGBackground',{viewBox,rect,pos});

    children.splice(pos + 1, 0, {
      tagName: 'rect',
      attributes: {
        id: 'background-rect',
        fill: '#fff',
        ...rect.toObject(),
      },
    });

    return xml;
  }
}

const FileFunction = (fn, rfn = ReadFile, wfn = WriteFile, namefn = n => n, ...args) => {
  return (filename, ...args) => {
    let data = rfn(filename, ...args);

    let output = fn(data, ...args);

    return wfn(namefn(filename), output, ...args);
  };
};

const SVGFileSetBackground = FileFunction(SetSVGBackground, ReadXML, WriteXML, n => AppendToFilename(n, '.with-background'), false);

const SVGResave = FileFunction(
  data => data,
  ReadXML,
  WriteXML,
  n => n,
  false,
);

function PutRowsColumns(rows) {
  let columnLength = rows.reduce((acc, row) => {
    for(let i = 0; i < row.length; i++) acc[i] = Math.max(acc[i] | 0, (row[i] + '').length);
    return acc;
  }, []);
  console.log('columnLength', columnLength);
  columnLength[columnLength.length - 1] = 0;

  return rows.map(row => row.map((col, i) => (col + '').padEnd(columnLength[i], ' ')).join(' ')).join('\n');
}

function GetLibrary(e) {
  let depth = 0;
  while((e = e.parentNode)) {
    const { tagName } = e;
    if(tagName == 'library') return e;
    if(tagName == 'eagle') break;
    //console.log('GetLibrary', { e, depth });
    depth++;
  }
}

function ElementName(e) {
  let n = e.name;
  n = n.replace(/[^A-Za-z0-9_]/g, '_');

  if(!/^[A-Za-z]/i.test(n)) {
    if(e.tagName == 'package') n = n.replace(/^[0-9]+_/g, '');
    let lib = GetLibrary(e);
    if(lib && lib.name) n = lib.name + n;
  }
  n = n.replace(/([0-9])_([A-Za-z])/g, '$1$2');

  if(e.tagName == 'package') n = n.toLowerCase();
  if(e.tagName == 'package') n = n.replace(/_[0-9]+x[0-9]+$/g, '');
  return n;
}

function GetRotation(element) {
  let { rot } = element;
  let mirror = false;
  let angle = 0;

  if(typeof rot == 'string' && rot.indexOf('M') != -1) {
    mirror = true;
    rot = rot.replace(/M/g, '');
  }
  if(typeof rot == 'string' && rot.indexOf('R') != -1) {
    rot = rot.replace(/R/g, '');
    angle = +rot;
  }
  return [mirror, angle];
}

function Package2Circuit(p) {
  let points = [...p.pads.list]
    .map(({ x, y }) => new Point(x, y))
    .map(pt => pt.div(2.54))
    .map(pt => pt.round(1));
  //.map(pt => pt.toFixed(1))
  let xpoints = points.map(({ x }) => x),
    ypoints = points.map(({ y }) => y);

  if(unique(ypoints).length > unique(xpoints).length) {
    points = points.map(pt => pt.rotate(Math.PI / 2).toFixed(1));
    console.log('Package2Circuit(1)', { points });
  }

  let half = points.filter(pt => [...pt].some(coord => coord != 0 && Math.abs(coord) < 1));

  if(half.length) {
    let t = new Translation(...half[0]).invert();
    // console.log('t', t);    points = points.map(pt => pt.transform(t));
  }

  let xoffs = unique(xpoints.map(x => Math.abs(x % 1)));
  let yoffs = unique(ypoints.map(y => Math.abs(y % 1)));

  if(xoffs.length == 1 && xoffs[0] != 0) {
    points = points.map(({ x, y }) => new Point(x - xoffs[0], y));
  }
  if(yoffs.length == 1 && yoffs[0] != 0) {
    points = points.map(({ x, y }) => new Point(x, y - yoffs[0]));
  }

  if(xoffs.length > 1 || yoffs.length > 1 /* || p.name.startsWith('DIL28')*/) console.log('Package2Circuit(2)', { points, xpoints: unique(xpoints), ypoints: unique(ypoints) });

  return [ElementName(p), points.join(' ')];
}

function Contactref2Circuit(cref) {
  let padIndex = cref.element.pads.list.raw.filter(e => e.tagName == 'pad').indexOf(cref.pad.raw);
  let { name } = cref.element;
  return `${name}.${padIndex + 1}`;
}

function Signal2Circuit(s) {
  let { name, contactrefs } = s;

  let contacts = '';
  let [firstContact, ...restOfContacts] = contactrefs;

  if(restOfContacts.length == 0) return '';

  for(let contact of restOfContacts) {
    contacts += `${Contactref2Circuit(firstContact)}\t${Contactref2Circuit(contact)}\n`;
  }
  return `# Signal ${name}\n` + contacts + '\n';
}

function Element2Circuit(element) {
  const { x, y, package: pkg } = element;

  let [packageName] = Package2Circuit(pkg);
  let name = ElementName(element);

  return `${name.padEnd(8)} ${packageName.padEnd(16)}${(x / 2.54).toFixed(0)},${(y / 2.54).toFixed(0)}\n`;

  // return [ElementName(element), [...element.pads.list].map(({ x, y }) => `${x / 2.54},${y / 2.54}`).join(' ')];
}

function GetUsedPackages(doc = project.board) {
  return unique([...doc.elements.list].map(e => e.package));
}

function GetPackagePitch(pkg) {
  let points = pkg.pads.list.map(pad => new Point(pad));

  if(points.length == 2) return Point.distance(...points);

  let smallest = Infinity;

  for(let i = 0; i < points.length; i++) {
    for(let j = 0; j < points.length; j++) {
      if(i == j) continue;

      let dist = Point.distance(points[i], points[j]);
      if(smallest > dist) smallest = dist;
    }
  }

  if(Number.isFinite(smallest)) return smallest;
}

function Eagle2Circuit(doc = project.board, width = 100, height = 100) {
  let o = '';

  o += `# Stripboard
# board <width>,<height>

board ${width},${height}

`;

  o += `# Packages
# <package name> <pin coordinates relative to pin 0>

`;

  o += PutRowsColumns(GetUsedPackages(doc).map(Package2Circuit));
  o += '\n\n';

  o += `# Components
# <component name> <package name> <absolute position of component pin 0>
`;
  for(let element of doc.elements.list) o += Element2Circuit(element);

  o += `
# Connections
# <from component name>.<pin index> <to component name>.<pin index>
`;
  for(let signal of doc.signals.list) o += Signal2Circuit(signal);

  return o;
}

function Eagle2CircuitJS(doc = project.schematic, scale = 50, sheet = 0) {
  let circ = new CircuitJS();
  let sheets = GetSheets(doc);
  console.log('sheets', sheets);
  let sh = (globalThis.sheet = sheets[sheet]);
  console.log('sh', sh);
  let tree = sh.raw;

  for(let [elm, ptr] of deep.iterate(tree, n => 'x1' in n.attributes && n.attributes.layer == 91)) {
    let ln = new Line(elm.attributes).mul(scale / 2.54).round();

    ptr = ptr.slice(0, -2);
    let segmentIndex = ptr[ptr.length - 1];
    let segment = deep.get(tree, ptr);
    let net = deep.get(tree, ptr.slice(0, -2));

    //console.log('ln',ln,elm.attributes.layer);
    console.log('segment/net', {
      segment: segmentIndex,
      net: net.attributes.name,
    });

    circ.add('w', ...ln, 0);
  }

  /*for(let [elm, ptr] of deep.iterate(tree, n => 'x' in n.attributes)) {
    let pt = new Point(elm.attributes).mul(scale / 2.54).round();
    console.log(elm.tagName, elm.attributes);

    circ.add('r', ...pt, 0);
  }*/
  let instances = sh.children.find(e => e.tagName == 'instances').children;

  console.log('instances', instances);
  for(let [name, instance] of instances) {
    console.log(`instance '${name}'`, console.config({ depth: 0 }), instance);
  }
  return circ;
}

function MakeGraph(board = project.board, ignore = /^(GND|[-+][0-9]+V)$/) {
  let g = new Graph({
    size: new Size(900, 900),
    spacing: 30,
    timestep: 50,
    charge: 100,
    mass: 200,
    damping: 0.000005,
    gravitate_to_origin: false,
  });
  let nodes = {};
  let nodeSignals = {};
  let signalNodes = {};

  for(let [name, element] of project.board.elements) {
    let n = (nodes[name] = g.addNode(name));

    nodeSignals[name] = [];
  }

  for(let [name, signal] of project.board.signals) {
    if(ignore && ignore.test(name)) continue;

    signalNodes[name] = [];
    for(let ref of signal.contactrefs) {
      nodeSignals[ref.element.name].push(name);
      signalNodes[name].push(ref.element.name);
    }
  }

  for(let node in nodeSignals) {
    for(let signal of nodeSignals[node]) {
      for(let node2 of signalNodes[signal]) {
        if(node != node2) {
          let r = g.addEdge(nodes[node], nodes[node2]);

          console.log('r', r);
        }
      }
    }
  }

  console.log('MakeGraph', { nodeSignals, signalNodes });

  return g;
}

function DrawGraph(g, w) {
  const { size } = w;

  nanovg.Save();
  nanovg.Translate(size.width / 2, size.height / 2);

  for(let edge of g.edges) {
    DrawLine(edge.a, edge.b, 1, [255, 255, 0]);
  }

  nanovg.TextAlign(nanovg.ALIGN_CENTER | nanovg.ALIGN_MIDDLE);
  nanovg.FontSize(12);

  for(let node of g.nodes) {
    DrawCircle([node.x, node.y], 20, 2, [255, 0, 0], [0, 0, 0, 255]);

    nanovg.FillColor(nanovg.RGB(255, 255, 255));
    nanovg.Text(node.x, node.y, node.label);
  }

  nanovg.Restore();
}

//callMain(main, true);

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  process.exit(1);
}
