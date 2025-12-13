import * as path from 'path';
import PortableSpawn from './lib/spawn.js';

const consoleOpts = { depth: Infinity, compact: 5, hideKeys: ['pos'] };

const data = `[ {  "kind": "N_FUNCTION", "name": "fn", "body": [ {  "kind": "N_CMDLIST", "cmds": [ {  "kind": "N_SIMPLECMD", "bngd": 0, "args": [ {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "dump", "pos": "3:3" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "-t", "pos": "3:8" } ] } ] }, {  "kind": "N_SIMPLECMD", "bngd": 0, "args": [ {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "dump", "pos": "4:3" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "-s", "pos": "4:8" } ] } ] }, {  "kind": "N_SIMPLECMD", "bngd": 0, "args": [ {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "/opt/diet/bin/cat", "pos": "5:3" } ] } ] } ] } ] } ]`;

const data2 = `[ {  "kind": "N_SUBSHELL", "cmds": [ {  "kind": "N_SIMPLECMD", "bngd": 0, "args": [ {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "read", "pos": "32:3" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "-r", "pos": "32:8" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "-p", "pos": "32:11" }, {  "kind": "N_ARGSTR", "flag": 1, "stra": "Test: ", "pos": "32:11" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "TEST", "pos": "32:22" } ] } ] }, {  "kind": "N_SIMPLECMD", "bngd": 0, "args": [ {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "echo", "pos": "33:3" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 1, "stra": "TEST=", "pos": "33:8" }, {  "kind": "N_ARGPARAM", "flag": 1, "name": "TEST", "word": "", "numb": 0, "pos": "33:16" }, {  "kind": "N_ARGSTR", "flag": 1, "stra": "", "pos": "33:8" } ] } ], "rdir": [ {  "kind": "N_REDIR", "flag": 10, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "2", "pos": "33:24" } ], "fdes": 1 } ] }, {  "kind": "N_SIMPLECMD", "bngd": 0, "args": [ {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "read", "pos": "34:3" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "-r", "pos": "34:8" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "TEST2", "pos": "34:11" } ] } ] }, {  "kind": "N_SIMPLECMD", "bngd": 0, "args": [ {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "echo", "pos": "36:3" } ] }, {  "kind": "N_ARG", "flag": 0, "list": [ {  "kind": "N_ARGSTR", "flag": 1, "stra": "TEST2=", "pos": "36:8" }, {  "kind": "N_ARGPARAM", "flag": 1, "name": "TEST2", "word": "", "numb": 0, "pos": "36:17" }, {  "kind": "N_ARGSTR", "flag": 1, "stra": "", "pos": "36:8" } ] } ], "rdir": [ {  "kind": "N_REDIR", "flag": 10, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "2", "pos": "36:26" } ], "fdes": 1 } ] } ], "rdir": [ {  "kind": "N_REDIR", "flag": 5, "list": [ {  "kind": "N_ARGSTR", "flag": 0, "stra": "tmp.txt", "pos": "37:4" } ], "fdes": 0 } ] } ]`;

function WriteFile(name, data) {
  if(Array.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  filesystem.writeFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}

async function main(...args) {
  await PortableSpawn();

  console.options = {
    ...consoleOpts,
    depth: 3,
    compact: 1,
    hideKeys: ['offset']
  };

  let file = args[0] ?? '';
  let ext = path.extname(file);
  let base = path.basename(file, ext);
  let input = '';

  console.log('ext:', ext);
  console.log('base:', base);

  switch (ext) {
    case '.sh': {
      let [rd, wr] = os.pipe();
      let cmd = ['shparse2ast', '-o', base + '.json', file];
      let child = os.exec(cmd, {
        block: true
        //  stdout: os.open(base + '.json', os.O_WRONLY | os.O_CREAT | os.O_TRUNC)
      });
      console.log('cmd:', cmd.join(' '));

      input = await filesystem.readFileSync(base + '.json', 'utf-8');

      break;
    }
    case '.json': {
      input = await filesystem.readFileSync(file, 'utf-8');
      break;
      //    break;
    }
  }
  // console.log('input:', input);

  let json = JSON.parse(input);

  console.log('json:', json);
}

main(...scriptArgs.slice(1));