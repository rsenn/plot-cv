import * as std from 'std';
import * as os from 'os';
import * as deep from './lib/deep.js';
import * as fs from './lib/filesystem.js';
import * as path from './lib/path.js';
import Util from './lib/util.js';
import { Console } from 'console';
import rpc from './quickjs/net/rpc.js';
import REPL from './repl.js';
import inspect from './lib/objectInspect.js';
import * as Terminal from './terminal.js';

const base = path.basename(Util.getArgv()[0], /\.[a-z]*$/);
const cmdhist = `.history-${base}`;

Util.define(Array.prototype, {
  contains(item) {
    return this.indexOf(item) != -1;
  }
});

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

async function main(...args) {
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
  console.log('repl', repl);
  repl.help = () => {};
  //repl.debug = (...args) => console.debug("REPL", ...args);
  let { log } = console;
  repl.show = arg => std.puts(typeof arg == 'string' ? arg : inspect(arg, { colors: true }));

  repl.cleanup = () => {
    Terminal.mousetrackingDisable();
    let hist = repl.history_get().filter((item, i, a) => a.lastIndexOf(item) == i);
    fs.writeFileSync(
      cmdhist,
      hist
        .filter(entry => (entry + '').trim() != '')
        .map(entry => entry.replace(/\n/g, '\\n') + '\n')
        .join('')
    );
    console.log(`EXIT (wrote ${hist.length} history entries)`);
    std.exit(0);
  };
  let debugLog = fs.fopen('debug.log', 'a');
  repl.debugLog = debugLog;
  console.log = repl.wrapPrintFunction(log, console);

  /*os.ttySetRaw(0, true);
  os.setReadHandler(0, () => repl.term_read_handler());
*/
  console.log('repl.run()', repl.runSync());

  let client = new rpc.Socket();
  client.log = repl.wrapPrintFunction(log, console);

  import('os').then(os =>
    import('net').then(({ client }) =>
      client.connect((url, callbacks) => client({ ...url, ...callbacks }), os)
    )
  );
  console.log('client', client);

  Object.assign(globalThis, { repl, client, Util, rpc });
}
Util.callMain(main, true);
