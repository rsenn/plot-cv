#!/usr/bin/env qjsm
import process from 'process';
import { readFileSync, createWriteStream } from 'fs';
import { getOpt, showHelp } from './lib/misc.js';
import { parseScript } from './lib/meriyah.js';

const { stdout, stderr, argv } = process;

let opts,
  params,
  stream = stdout;

params = getOpt(
  (opts = {
    help: [false, null, 'h'],
    output: [true, null, 'o'],
    module: [false, null, 'm'],
    next: [false, null, 'n'],
    ranges: [false, null, 'r'],
    loc: [false, null, 'l'],
    indent: [true, null, 'i'],
    '@': 'args'
  }),
  argv.slice(2)
);

const args = params['@'];

if(args.length < 1 || params.help) showHelp(opts, params.help ? 0 : 1);

if('output' in params) stream = createWriteStream(params.output || args[0] + '.ast.json', 'utf8');

const { indent = stream.isTTY ? 2 : undefined, help, output, ...parseOpts } = params;

const input = readFileSync(args[0], 'utf8');
const ast = parseScript(input, parseOpts);

stream.write(JSON.stringify(ast, ...(indent > 0 ? [null, +indent] : [])) + '\n');
