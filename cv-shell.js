import * as cv from 'opencv';
import Util from './lib/util.js';
import * as deep from './lib/deep.js';
import path from './lib/path.js';
import ConsoleSetup from './lib/consoleSetup.js';
import REPL from './xrepl.js';
import PortableFileSystem from './lib/filesystem.js';
import { Pointer } from './lib/pointer.js';
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
  await ConsoleSetup({
    inspectOptions: { depth: 3, compact: 3, maxArrayLength: 10, maxStringLength: 10 }
  });

  let fs;
  let debugLog;

  if(Util.getPlatform() == 'quickjs') {
    globalThis.std = await import('std');
    globalThis.os = await import('os');
    globalThis.fs = fs = await import('./lib/filesystem.js');
  } else {
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

  //Object.assign(globalThis, cv);

  Object.assign(globalThis, {
    cv,
    fs,
    Pointer,
    deep,
    VideoSource,
    ImageSequence
  });

  Object.assign(globalThis, {
    quit(arg) {
      repl.cleanup();
      Util.exit(arg ?? 0);
    }
  });

  cmdhist = `.${base}-cmdhistory`;

  let repl = (globalThis.repl = new REPL(base));

  repl.history = LoadHistory(cmdhist);

  repl.printStatus(/*console.log*/ `LOAD (read ${repl.history.length} history entries)`);

  repl.debugLog = debugLog;
  repl.exit = Terminate;
  repl.importModule = importModule;
  repl.debug = (...args) => {
    let s = '';
    for(let arg of args) {
      if(s) s += ' ';
      if(typeof arg != 'strping' || arg.indexOf('\x1b') == -1)
        s += inspect(arg, { depth: Infinity, depth: 6, compact: false });
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

  repl.addCleanupHandler(() => {
    let hist = repl.history.filter((item, i, a) => a.lastIndexOf(item) == i);

    fs.writeFileSync(
      cmdhist,
      hist
        .filter(entry => (entry + '').trim() != '')
        .map(entry => entry.replace(/\n/g, '\\\\n') + '\n')
        .join('')
    );

    console.log(`EXIT (wrote ${hist.length} history entries)`);
    Terminate(0);
  });
  /*
  Util.atexit(() => {
    let hist = repl.history.filter((item, i, a) => a.lastIndexOf(item) == i);

    fs.writeFileSync(histfile, JSON.stringify(hist, null, 2));

    console.log(`EXIT (wrote ${hist.length} history entries)`);
  });
*/

  /*  for(let file of params['@']) {
    repl.printStatus(`Loading '${file}'...`);

    newProject(file);
  }
*/
  //globalThis.project = new EagleProject(params['@']);
  //console.log('globalThis.project', globalThis.project);

  await repl.run();
}

Util.callMain(main, true);
