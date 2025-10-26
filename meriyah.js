#!/usr/bin/env qjsm
import { createWriteStream, readFileSync } from 'fs';
import { getOpt, showHelp } from './lib/misc.js';
import { parseScript } from './lib/meriyah.js';
import process from 'process';

const { stdout, stderr, argv } = process;

let opts,
  params,
  stream = stdout,
  ast;

params = getOpt(
  (opts = {
    help: [false, null, 'h'],
    output: [true, null, 'o'],
    module: [false, null, 'm'],
    next: [false, null, 'n'],
    ranges: [false, null, 'r'],
    loc: [false, null, 'l'],
    indent: [true, null, 'i'],
    '@': '',
  }),
  argv.slice(2),
);

const args = params['@'];

if(args.length < 1 || params.help) showHelp(opts, params.help ? 0 : 1);

if('output' in params) stream = createWriteStream(params.output || args[0] + '.ast.json', 'utf8');

const { indent = stream.isTTY ? 2 : undefined, help, output, ...parseOpts } = params;

const input = readFileSync(args[0], 'utf8');

if(!('next' in parseOpts)) parseOpts.next = true;

for(;;) {
  try {
    ast = parseScript(input, parseOpts);
  } catch(error) {
    if(/module goal/.test(error.message)) {
      parseOpts.module = true;
      continue;
    }
    console.log('ERROR: ' + error.message + '\n' + error.stack);
    throw error;
  }
  break;
}

stream.write(JSON.stringify(ast, ...(indent > 0 ? [null, +indent] : [])) + '\n');
