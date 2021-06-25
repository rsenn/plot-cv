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
import * as net from 'net';
import { Socket, recv, send, errno } from './socket.js';

extendArray();

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      colors: true,
      depth: Infinity,
      compact: 2,
      customInspect: true,
      getters: true
    }
  });
  let params = Util.getOpt(
    {
      verbose: [false, (a, v) => (v | 0) + 1, 'v'],
      listen: [false, null, 'l'],
      connect: [false, null, 'c'],
      debug: [false, null, 'x'],
      address: [true, null, 'a'],
      port: [true, null, 'p'],
      '@': 'address,port'
    },
    args
  );
  //const { listen } = params;

  console.log('params', params);
  const [address = '127.0.0.1', port = 9000] = args;

  const listen = params.connect && !params.listen ? false : true;

  let name = Util.getArgs()[0];
  name = name
    .replace(/.*\//, '')
    .replace(/-/g, ' ')
    .replace(/\.[^\/.]*$/, '');

  let [prefix, suffix] = name.split(' ');

  let repl = new REPL(`\x1b[38;5;165m${prefix} \x1b[38;5;39m${suffix}\x1b[0m`, fs, false);

  repl.historyLoad(null, false);

  repl.help = () => {};
  let { log } = console;
  repl.show = arg => std.puts(typeof arg == 'string' ? arg : inspect(arg, { colors: true }));

  repl.cleanup = () => {
    repl.readlineRemovePrompt();
    Terminal.mousetrackingDisable();
    let numLines = repl.historySave();

    repl.printStatus(`EXIT (wrote ${numLines} history entries)`, false);

    std.exit(0);
  };

  console.log = repl.printFunction(log);
  //  console.log = (...args) => repl.printStatus(() => log(...args));

  let cli = new rpc.Socket('0.0.0.0:9200', rpc.RPCServerConnection, +params.verbose);

  cli.register(Socket);

  const createWS = (url, callbacks, listen) =>
    [net.client, net.server][+listen]({
      mounts: [['/', '.', 'debugger.html']],
      ...url,
      ...callbacks,
      onHttp(sock, url) {
        console.log(url.replace('/', ''));
        //console.log('onHttp', { sock, url }, sock.respond);

        if(url != '/') {
          if(/\.html/.test(url) && !/debugger.html/.test(url))
            sock.redirect(sock.HTTP_STATUS_FOUND, '/debugger.html');

          //if(!/\.(js|css)$/.test(url)) return sock.respond(sock.HTTP_STATUS_NOT_FOUND, 'Not found');
        }
        sock.header('Test', 'blah');
      }
    });

  globalThis[['connection', 'listener'][+listen]] = cli;
  globalThis.connections = cli.fdlist;

  Object.assign(globalThis, {
    repl,
    Util,
    ...rpc,
    quit,
    exit: quit,
    Socket,
    recv,
    send,
    errno,
    cli,
    net,
    std,
    os,
    deep,
    fs,
    path
  });

  [cli.connect, cli.listen][+listen].call(cli, createWS, os);

  function quit(why) {
    repl.cleanup(why);
  }

  repl.runSync();
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  //console.log('SUCCESS');
}
