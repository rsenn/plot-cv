import * as std from 'std';
import * as os from 'os';
import * as deep from './lib/deep.js';
import * as fs from './lib/filesystem.js';
import * as path from './lib/path.js';
import Util from './lib/util.js';
import { Console } from 'console';
import rpc from './quickjs/net/rpc.js';
import REPL from './quickjs/modules/lib/repl.js';
import inspect from './lib/objectInspect.js';
import * as Terminal from './terminal.js';
import { extendArray, define } from './lib/misc.js';

const base = path.basename(Util.getArgv()[0], /\.[a-z]*$/);
const cmdhist = `.history-${base}`;

function LoadHistory(filename) {
  let contents = std.loadFile(filename);
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

  return (parse() ?? [])
    .filter(entry => (entry + '').trim() != '')
    .map(entry => entry.replace(/\\n/g, '\n'));
}

function ReadJSON(filename) {
  let data = std.loadFile(filename);

  if(data) console.log(`ReadJSON('${filename}') ${data.length} bytes read`);
  return data ? JSON.parse(data) : null;
}

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      colors: true,
      depth: Infinity,
      maxArrayLength: 100,
      breakLength: 10000,
      compact: 2 /*,
      customInspect: true*/
    }
  });
  extendArray(Array.prototype, define);
  console.log('[1,2,3,4].last', [1, 2, 3, 4].last);
  console.log('[1,2,3,4].at(-1)', [1, 2, 3, 4].at(-1));
  console.log('console.options', console.options);
  let params = Util.getOpt(
    {
      listen: [false, null, 'l'],
      debug: [false, null, 'x'],
      address: [true, null, 'c'],
      port: [true, null, 'p'],
      '@': 'address,port'
    },
    args
  );
  const { listen } = params;

  const [address = '127.0.0.1', port = 9000] = args;

  let name = Util.getArgs()[0];
  name = name
    .replace(/.*\//, '')
    .replace(/-/g, ' ')
    .replace(/\.[^\/.]*$/, '');
  let repl = new REPL(name);

  repl.help = () => {};
  let { log } = console;
  repl.show = arg => std.puts(typeof arg == 'string' ? arg : inspect(arg, { colors: true }));

  repl.cleanup = () => {
    Terminal.mousetrackingDisable();

    let numLines = repl.historySave();
    console.log(`EXIT (wrote ${numLines} history entries)`);
    std.exit(0);
  };
  let debugLog = fs.fopen('debug.log', 'a');
  repl.debugLog = debugLog;
  console.log = (...args) => repl.printStatus(() => log(...args));

  let client = new rpc.Socket();
  //client.log = (...args) => console.log("RPC Client", ...args); //repl.wrapPrintFunction(log, console);

  import('os').then(os =>
    import('net').then(({ client }) =>
      client.connect((url, callbacks) => client({ ...url, ...callbacks }), os)
    )
  );

  Object.assign(globalThis, { repl, client, Util, rpc });

repl.historyLoad(null, false);
  repl.runSync();
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  console.log('SUCCESS');
}
