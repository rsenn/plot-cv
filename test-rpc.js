import * as std from 'std';
import * as os from 'os';
import * as deep from './lib/deep.js';
import * as fs from './lib/filesystem.js';
import * as path from './lib/path.js';
import Util from './lib/util.js';
import { Console } from 'console';
import REPL from './quickjs/modules/lib/repl.js';
import inspect from './lib/objectInspect.js';
import * as Terminal from './terminal.js';
import { extendArray } from './lib/misc.js';
import * as net from 'net';
import { Socket, recv, send, errno } from './socket.js';
import { EventEmitter, EventTarget, eventify } from './lib/events.js';
import { Repeater } from './lib/repeater/repeater.js';
import { fnmatch, PATH_FNM_MULTI } from './lib/fnmatch.js';

import rpc from './quickjs/net/rpc.js';
//import { RPCServer, RPCClient, RPCApi, RPCSocket,RPCFactory } from './quickjs/net/rpc.js';
import * as rpc2 from './quickjs/net/rpc.js';

extendArray();
Object.assign(globalThis, {
  EventEmitter,
  EventTarget,
  eventify,
  Repeater,
  fnmatch,
  PATH_FNM_MULTI,
  ...rpc2,
  rpc
});

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      colors: true,
      depth: Infinity,
      compact: 2,
      customInspect: true,
      getters: false
    }
  });
  let params = Util.getOpt(
    {
      verbose: [false, (a, v) => (v | 0) + 1, 'v'],
      listen: [false, null, 'l'],
      connect: [false, null, 'c'],
      client: [false, null, 'C'],
      server: [false, null, 'S'],
      debug: [false, null, 'x'],
      address: [true, null, 'a'],
      port: [true, null, 'p'],
      '@': 'address,port'
    },
    args
  );
  //const { listen } = params;

  console.log('params', params);
  const { address = '0.0.0.0', port = 9000 } = params;

  const listen = params.connect && !params.listen ? false : true;
  const server = !params.client || params.server;

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

  Object.assign(console.options, { depth: 2, compact: 2, getters: false });

  let cli = (globalThis.sock = new rpc.Socket(
    `${address}:${port}`,
    rpc[`RPC${server ? 'Server' : 'Client'}Connection`],
    +params.verbose
  ));

  cli.register({ Socket, Worker: os.Worker, Repeater, REPL, EventEmitter });

  let connections = new Set();
  const createWS = (globalThis.createWS = (url, callbacks, listen) => {
    console.log('createWS', { url, callbacks, listen });
    return [net.client, net.server][+listen](
      /*new EventLogger*/ {
        mounts: [['/', '.', 'debugger.html']],
        ...url,
        onFd(fd) {
          console.log('onFd', fd);
        },
        onConnect(s) {
          console.log('onConnect', s);
        },
        onOpen(s) {
          console.log('onOpen', s);
        },
        ...callbacks,
        onHttp(sock, url) {
          if(url != '/') {
            if(/\.html/.test(url) && !/debugger.html/.test(url))
              sock.redirect(sock.HTTP_STATUS_FOUND, '/debugger.html');
          }
          sock.header('Test', 'blah');
        },
        ...(url && url.host ? url : {})
      }
    );
  });

  globalThis[['connection', 'listener'][+listen]] = cli;

  define(globalThis, {
    get connections() {
      return [...connections];
    }
  });

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

  define(globalThis, listen ? { server: cli, cli } : { client: cli, cli });

  if(listen) cli.listen(createWS, os);
  else cli.connect(createWS, os);

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
