import PortableFileSystem from './lib/filesystem.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import ConsoleSetup from './lib/consoleSetup.js';
import REPL from './repl.js';
import { BinaryTree, BucketStore, BucketMap, ComponentMap, CompositeMap, Deque, Enum, HashList, Multimap, Shash, SortedMap, HashMultimap, MultiBiMap, MultiKeyMap, DenseSpatialHash2D, SpatialHash2D, HashMap, SpatialH, SpatialHash, SpatialHashMap, BoxHash } from './lib/container.js';
import * as std from 'std';
import { cv, draw, Contour, Line, Mat, Point, PointIterator, Rect, Size, TickMeter, VideoCapture } from './lib/opencv.js';

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

async function main(...args) {
  await ConsoleSetup({ /*breakLength: 240, */ depth: 10 });
  await PortableFileSystem(fs => (filesystem = fs));

  const base = path.basename(Util.getArgv()[1], /\.[^.]*$/);
  const histfile = `.${base}-history`;

  Object.assign(globalThis, {
    cv,
    draw,
    Contour,
    Line,
    Mat,
    Point,
    PointIterator,
    Rect,
    Size,
    TickMeter,
    VideoCapture
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

  let repl = (globalThis.repl = new REPL('OpenCV'));
  repl.exit = Util.exit;
  repl.importModule = importModule;

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
