import REPL from './repl.js';
import ConsoleSetup from './lib/consoleSetup.js';
import PortableFileSystem from './lib/filesystem.js';
import * as Terminal from './terminal.js';

const consoleOpts = { depth: 5, compact: 2, hideKeys: ['range', 'loc'] };

const data = `[ {  "kind": "N_FUNCTION", "name": "fn", "body": [ {  "kind": "N_CMDLIST", "cmds": [ {  "kind": "N_SIMPLECMD", "bngd": 0, "args": [ {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "dump", "pos": "3:3" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "-t", "pos": "3:8" } ] } ] }, {  "kind": "N_SIMPLECMD", "bngd": 0, "args": [ {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "dump", "pos": "4:3" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "-s", "pos": "4:8" } ] } ] }, {  "kind": "N_SIMPLECMD", "bngd": 0, "args": [ {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "/opt/diet/bin/cat", "pos": "5:3" } ] } ] } ] } ] } ]`;

const data2 = `[ {  "kind": "N_SUBSHELL", "cmds": [ {  "kind": "N_SIMPLECMD", "bngd": 0, "args": [ {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "read", "pos": "32:3" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "-r", "pos": "32:8" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "-p", "pos": "32:11" }, {  "kind": "N_ARGSTR", "flag": 1, "stra": "Test: ", "pos": "32:11" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "TEST", "pos": "32:22" } ] } ] }, {  "kind": "N_SIMPLECMD", "bngd": 0, "args": [ {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "echo", "pos": "33:3" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 1, "stra": "TEST=", "pos": "33:8" }, {  "kind": "N_ARGPARAM", "flag": 1, "name": "TEST", "word": "", "numb": 0, "pos": "33:16" }, {  "kind": "N_ARGSTR", "flag": 1, "stra": "", "pos": "33:8" } ] } ], "rdir": [ {  "kind": "N_REDIR", "flag": 10, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "2", "pos": "33:24" } ], "fdes": 1 } ] }, {  "kind": "N_SIMPLECMD", "bngd": 0, "args": [ {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "read", "pos": "34:3" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "-r", "pos": "34:8" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "TEST2", "pos": "34:11" } ] } ] }, {  "kind": "N_SIMPLECMD", "bngd": 0, "args": [ {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "echo", "pos": "36:3" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 1, "stra": "TEST2=", "pos": "36:8" }, {  "kind": "N_ARGPARAM", "flag": 1, "name": "TEST2", "word": "", "numb": 0, "pos": "36:17" }, {  "kind": "N_ARGSTR", "flag": 1, "stra": "", "pos": "36:8" } ] } ], "rdir": [ {  "kind": "N_REDIR", "flag": 10, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "2", "pos": "36:26" } ], "fdes": 1 } ] } ], "rdir": [ {  "kind": "N_REDIR", "flag": 5, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "tmp.txt", "pos": "37:4" } ], "fdes": 0 } ] } ]`;

async function main(...args) {
  await ConsoleSetup(consoleOpts);
  await PortableFileSystem();

  let json = args[0] ? filesystem.readFile(args[0], 'utf-8') : data2;
  let ast = JSON.parse(json);

  console.log('ast:', ast);
}

Util.callMain(main, true);
