import { AutoValue } from './autoValue.js';
import { ImagePipeline } from './imagePipeline.js';
import { LoadHistory, ReadBJSON, ReadFile, ReadJSON, WriteBJSON, WriteFile, WriteJSON } from './io-helpers.js';
import { ImageInfo } from './lib/image-info.js';
import { lazyInitializer } from './lib/lazyInitializer.js';
import { define, difference, escape, fnmatch, getFunctionArguments, getOpt, glob, GLOB_TILDE, lazyProperties, memoize, quote, toArrayBuffer, toString, union, wordexp } from './lib/misc.js';
import inspect from './lib/objectInspect.js';
import * as path from './lib/path.js';
import { Pointer } from './lib/pointer.js';
import { read as fromXML, write as toXML } from './lib/xml.js';
import { DrawText, Mouse, MouseEvents, MouseFlags, TextStyle, Window } from './qjs-opencv/js/cvHighGUI.js';
import { ImageSequence, VideoSource } from './qjs-opencv/js/cvVideo.js';
import { Console } from 'console';
import { REPL } from 'repl';
import * as Terminal from 'terminal';
import * as cv from 'opencv';
import extendArray from 'extendArray';

let cmdhist,
  defaultWin = lazyInitializer(() => new Window(path.basename(scriptArgs[0], '.js')));

extendArray();

function getConfFile(base) {
  return std.getenv('HOME') + '/.' + path.basename(scriptArgs[0], '.js') + '_' + base;
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

function StartREPL(prefix = path.basename(scriptArgs[0], '.js'), suffix = '') {
  let repl = new REPL(`\x1b[38;5;165m${prefix} \x1b[38;5;39m${suffix}\x1b[0m`, undefined, false);
  repl.historyLoad(getConfFile('history'));
  repl.loadSaveOptions();
  repl.inspectOptions = console.options;

  let { log } = console;

  /*repl.show = arg => {
    repl.globalKeys();
    if(arg instanceof cv.Mat) {
      console.log('arg', arg);
      if(!arg.empty) {
        let win = defaultWin();
        win.resize(arg.cols, arg.rows);
        win.show(arg);
        cv.waitKey(1);
        return;
      }
    }

    std.puts((typeof arg == 'string' ? arg : inspect(arg, repl.inspectOptions)) + '\n');
  };*/

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

  /*console.log = repl.printFunction((...args) => {
     log(console.config(repl.inspectOptions), ...args);
  });
*/
  repl.run();
  return repl;
}

function main(...args) {
  globalThis.console = new Console(std.err, {
    inspectOptions: { compact: 2, customInspect: true, maxArrayLength: 20, maxStringLength: 100, numberBase: 10 }
  });
  let debugLog;

  debugLog = std.open('debug.log', 'a');

  const progName = scriptArgs[0];
  const base = path.basename(progName, path.extname(progName));
  const histfile = `.${base}-history`;

  let params = getOpt(
    {
      debug: [false, null, 'x'],
      'output-dir': [true, null, 'd'],
      '@': 'input'
    },
    args
  );

  Object.assign(globalThis, { cv });

  Object.assign(globalThis, {
    quit(arg) {
      repl.cleanup();
      std.exit(arg ?? 0);
    }
  });

  cmdhist = `.${base}-cmdhistory`;
  let repl = StartREPL();
  lazyProperties(repl, {
    globalKeys() {
      let a = getKeys();
      function getKeys() {
        return new Set(Object.keys(globalThis));
      }
      return function globalKeys() {
        let newKeys = getKeys();
        let [removed, added] = difference(a, newKeys);
        let changed = union(a, newKeys);
        /* for(let key of removed) console.log(`key '${key}' removed`);
        for(let key of added) console.log(`key '${key}' added`);*/
        if(removed.length || added.length) {
          let newObj = {},
            oldObj = repl.globalValues() ?? {};
          for(let k of a) newObj[k] = oldObj[k];
          for(let k of added) newObj[k] = globalThis[k];
          for(let k of a) newObj[k] = oldObj[k];
          repl.globalValues(newObj);
          a = newKeys;
        }
        //return a;
      };
    },
    globalValues() {
      return AutoValue(getConfFile('global'));
    }
  });

  Object.assign(globalThis, {
    Pointer,
    VideoSource,
    ImageSequence,
    ImagePipeline,
    MouseEvents,
    MouseFlags,
    Mouse,
    Window,
    TextStyle,
    DrawText,
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
    ReadBJSON,
    WriteFile,
    WriteJSON,
    WriteBJSON,
    ImageInfo,
    AutoValue
  });
  repl.globalKeys();

  //setInterval(() => repl.globalKeys(), 500);
}

main(...scriptArgs.slice(1));