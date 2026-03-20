import parseDiff from './lib/parse-diff.js';
import { readFileSync, writeFileSync } from 'fs';
import { getOpt } from 'util';
import { basename, dirname, extname } from 'path';

function FormatName(name) {
  return name.replaceAll(/[\W]/g, '_');
}

function Chunk(chunk) {
  const { content, changes } = chunk;
  let s = '';

  s += content + '\n';

  for(let { content } of changes) {
    s += content + '\n';
  }

  return s;
}

function ProcessDiff(d, filename) {
  let base = basename(filename, extname(filename));
  let i = 1;

  for(let diff of d) {
    const { chunks, from: src, to: dst } = diff;

    let s = `diff --git a/${src} b/${dst}\n--- a/${src}\n+++ b/${dst}\n`;

    for(let chunk of chunks) {
      writeFileSync(base + '-' + FormatName(dst) + `-${i++}.diff`, s + Chunk(chunk));
    }
  }
}

function main(...args) {
  let params = (globalThis.params = getOpt(
    {
      verbose: [false, (a, v) => (v | 0) + 1, 'v'],
      '@': 'files',
    },
    args,
  ));

  for(let arg of params['@']) {
    const d = parseDiff(readFileSync(arg, 'utf-8'));

    console.log(arg, console.config({ compact: false }), d);

    ProcessDiff(d, arg);
  }
}

main(...scriptArgs.slice(1));
