import * as cv from 'opencv';
import Util from './lib/util.js';
import { toArrayBuffer, toString, escape, quote, define, extendArray, memoize, getFunctionArguments, glob, GLOB_TILDE, fnmatch, wordexp } from './lib/misc.js';
import * as misc from './lib/misc.js';
import * as util from 'util';
import * as deep from './lib/deep.js';
import path from './lib/path.js';
import { Console } from 'console';
import REPL from './xrepl.js';
import * as fs from './lib/filesystem.js';
import { Pointer } from './lib/pointer.js';
import * as Terminal from './terminal.js';
import { read as fromXML, write as toXML } from './lib/xml.js';
import inspect from './lib/objectInspect.js';
import { ReadFile, LoadHistory, ReadJSON, MapFile, ReadBJSON, WriteFile, WriteJSON, WriteBJSON, DirIterator, RecursiveDirIterator, ReadDirRecursive } from './io-helpers.js';
import { VideoSource, ImageSequence } from './qjs-opencv/js/cvVideo.js';
import { ImageInfo } from './lib/image-info.js';
import { MouseEvents, MouseFlags, Mouse, Window, TextStyle, DrawText } from './qjs-opencv/js/cvHighGUI.js';
//import {   DirIterator, RecursiveDirIterator, ReadDirRecursive, Filter, FilterImages, SortFiles, StatFiles } from './io-helpers.js';

let cmdhist;

extendArray();

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
  repl.fs = fs;
  repl.historyLoad(null, false);
  repl.inspectOptions = console.options;

  let { log } = console;
  repl.show = arg => std.puts((typeof arg == 'string' ? arg : inspect(arg, repl.inspectOptions)) + '\n');

  repl.cleanup = () => {
    repl.readlineRemovePrompt();
    Terminal.mousetrackingDisable();
    let numLines = repl.historySave();

    repl.printStatus(`EXIT (wrote ${numLines} history entries)`, false);

    std.exit(0);
  };
  repl.directives = {
    load: [
      image => {
        console.log('load', { image });
      },
      'loads an image / video',
      function(line, pos) {
        let arg = line.replace(/^\\*load\s*/, '');
        let start = line.length - arg.length;
        let paths = [];
        let pattern = wordexp(arg, 0)[0];
        //     if(!pattern.endsWith('*')) pattern += '*';

        glob(pattern + '*', GLOB_TILDE, (p, err) => console.log('glob error', { p, err }), paths);

        const tab = paths.filter(p => p.startsWith(pattern)); //.map(p => p.replace(arg, ''));

        console.log('complete', { line, arg, pos });
        return { tab, pos: 0, ctx: {} };
      }
    ]
  };

  console.log = repl.printFunction((...args) => {
    log(console.config(repl.inspectOptions), ...args);
  });

  repl.run();
  return repl;
}

function main(...args) {
  globalThis.console = new Console(std.err, {
    inspectOptions: { compact: 2, customInspect: true, maxArrayLength: 20, maxStringLength: 100, numberBase: 16 }
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
    util,
    misc,
    Pointer,
    deep,
    VideoSource,
    ImageSequence,
    MouseEvents,
    MouseFlags,
    Mouse,
    Window,
    TextStyle,
    DrawText,
    repl,
    Util,
    toArrayBuffer,
    toString,
    escape,
    quote,
    define,
    extendArray,
    memoize,
    getFunctionArguments,
    glob,
    fnmatch,
    wordexp,
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
    RecursiveDirIterator,
    ImageInfo
  });
}

Util.callMain(main, true);
