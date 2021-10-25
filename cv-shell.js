import * as cv from 'opencv';
import Util from './lib/util.js';
import { toArrayBuffer, toString, escape, quote, define, extendArray } from './lib/misc.js';
import * as deep from './lib/deep.js';
import path from './lib/path.js';
import { Console } from 'console';
import REPL from './xrepl.js';
import * as fs from './lib/filesystem.js';
import { Pointer } from './lib/pointer.js';
import * as Terminal from './terminal.js';
import { read as fromXML, write as toXML } from './lib/xml.js';
import inspect from './lib/objectInspect.js';
import {
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
} from './io-helpers.js';
import { VideoSource, ImageSequence } from './qjs-opencv/js/cvVideo.js';

let cmdhist;

extendArray();

function* Filter(gen, regEx = /.*/) {
  for(let item of gen) if(regEx.test(item)) yield item;
}

function* ReadDirRecursive(dir, maxDepth = Infinity) {
  for(let file of fs.readdirSync(dir)) {
    if(['.', '..'].indexOf(file) != -1) continue;

    let entry = `${dir}/${file}`;
    let isDir = false;
    let st = fs.statSync(entry);

    isDir = st && st.isDirectory();

    yield isDir ? entry + '/' : entry;

    if(maxDepth > 0 && isDir) yield* ReadDirRecursive(entry, maxDepth - 1);
  }
}

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

function StartREPL(prefix = path.basename(Util.getArgs()[0], '.js'), suffix = '') {
  let repl = new REPL(`\x1b[38;5;165m${prefix} \x1b[38;5;39m${suffix}\x1b[0m`, fs, false);

  repl.historyLoad(null, false);
  repl.inspectOptions = { ...console.options, compact: 2 };

  repl.help = () => {};
  let { log } = console;
  repl.show = arg => std.puts((typeof arg == 'string' ? arg : inspect(arg, repl.inspectOptions)) + '\n');

  repl.cleanup = () => {
    repl.readlineRemovePrompt();
    Terminal.mousetrackingDisable();
    let numLines = repl.historySave();

    repl.printStatus(`EXIT (wrote ${numLines} history entries)`, false);

    std.exit(0);
  };

  console.log = repl.printFunction((...args) => {
    log(console.config(repl.inspectOptions), ...args);
  });

  repl.run();
  return repl;
}

function main(...args) {
  globalThis.console = new Console(std.err, {
    inspectOptions: { compact: 2, customInspect: true }
  });
  let debugLog;

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

  //Object.assign(globalThis, cv);

  Object.assign(globalThis, {
    quit(arg) {
      repl.cleanup();
      std.exit(arg ?? 0);
    }
  });

  cmdhist = `.${base}-cmdhistory`;

  let repl = StartREPL();
  Object.assign(globalThis, {
    cv,
    fs,
    Pointer,
    deep,
    VideoSource,
    ImageSequence,
    repl,
    ReadDirRecursive,
    Filter,
    Util,
    toArrayBuffer,
    toString,
    escape,
    quote,
    define,
    extendArray,
    path,
    Console,
    REPL,
    Terminal,
    fromXML,
    toXML,
    inspect,
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
}

Util.callMain(main, true);
